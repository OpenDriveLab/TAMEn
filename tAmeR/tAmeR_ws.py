#!/usr/bin/env python3
import argparse
import asyncio
import contextlib
import json
import time
from typing import Dict, Optional, Set

import cv2
import numpy as np
from aiohttp import web
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class RosImageStripNode(Node):
    def __init__(self, args):
        super().__init__("ros_image_strip_bridge")
        self.frame_state: Dict[str, Optional[np.ndarray]] = {
            "left_camera": None,
            "right_camera": None,
            "right_tactile_camera": None,
            "left_tactile_camera": None,
        }
        self.last_update: Dict[str, Optional[float]] = {k: None for k in self.frame_state.keys()}
        self.subscriptions = []

        topic_map = {
            "left_camera": args.left_topic,
            "right_camera": args.right_topic,
            "right_tactile_camera": args.right_tactile_topic,
            "left_tactile_camera": args.left_tactile_topic,
        }

        for name, topic in topic_map.items():
            sub = self.create_subscription(
                Image,
                topic,
                lambda msg, camera_name=name: self.on_image(camera_name, msg),
                10,
            )
            self.subscriptions.append(sub)

    def on_image(self, camera_name: str, msg: Image):
        try:
            frame = self.msg_to_bgr(msg)
            self.frame_state[camera_name] = frame
            self.last_update[camera_name] = time.time()
        except Exception as exc:
            self.get_logger().error(f"Failed to decode image from {camera_name}: {exc}")

    def msg_to_bgr(self, msg: Image) -> np.ndarray:
        enc = msg.encoding.lower()
        if enc in ("mono8", "8uc1"):
            image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        channels = 4 if enc in ("rgba8", "bgra8") else 3
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, channels))

        if enc in ("rgb8", "rgb16"):
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if enc in ("bgr8", "bgr16"):
            return image
        if enc == "rgba8":
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        if enc == "bgra8":
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

        return cv2.cvtColor(image[:, :, :3], cv2.COLOR_RGB2BGR)

    def build_strip(self, tile_width: int, tile_height: int) -> np.ndarray:
        order = [
            "left_camera",
            "right_camera",
            "right_tactile_camera",
            "left_tactile_camera",
        ]
        tiles = []
        for key in order:
            frame = self.frame_state.get(key)
            if frame is None:
                tile = np.zeros((tile_height, tile_width, 3), dtype=np.uint8)
            else:
                tile = cv2.resize(frame, (tile_width, tile_height), interpolation=cv2.INTER_AREA)
            tiles.append(tile)
        return np.hstack(tiles)

    def health(self):
        return {
            "ok": True,
            "frame_ages_seconds": {
                name: None if ts is None else round(time.time() - ts, 3)
                for name, ts in self.last_update.items()
            },
            "layout": [
                "left_camera",
                "right_camera",
                "right_tactile_camera",
                "left_tactile_camera",
            ],
        }


class JpegStreamHub:
    def __init__(self, ros_node: RosImageStripNode, tile_width: int, tile_height: int, jpeg_quality: int):
        self.ros_node = ros_node
        self.tile_width = tile_width
        self.tile_height = tile_height
        self.jpeg_quality = jpeg_quality
        self.clients: Set[web.WebSocketResponse] = set()

    def encode_frame(self) -> Optional[bytes]:
        strip = self.ros_node.build_strip(self.tile_width, self.tile_height)
        ok, encoded = cv2.imencode(
            ".jpg",
            strip,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            return None
        return encoded.tobytes()

    async def broadcast_loop(self, fps: int):
        frame_interval = 1.0 / max(1, fps)
        while True:
            if self.clients:
                payload = self.encode_frame()
                if payload is not None:
                    dead_clients = []
                    for ws in list(self.clients):
                        try:
                            await ws.send_bytes(payload)
                        except Exception:
                            dead_clients.append(ws)
                    for ws in dead_clients:
                        with contextlib.suppress(Exception):
                            await ws.close()
                        self.clients.discard(ws)
            await asyncio.sleep(frame_interval)



def make_app(args):
    rclpy.init(args=None)
    ros_node = RosImageStripNode(args)
    hub = JpegStreamHub(
        ros_node=ros_node,
        tile_width=args.tile_width,
        tile_height=args.tile_height,
        jpeg_quality=args.jpeg_quality,
    )

    async def spin_ros():
        while True:
            rclpy.spin_once(ros_node, timeout_sec=0.01)
            await asyncio.sleep(0.01)

    async def index(_request):
        return web.json_response({
            "message": "ROS strip WebSocket-JPEG bridge is running",
            "video_ws": "/ws/video",
        })

    async def health(_request):
        data = ros_node.health()
        data["clients"] = len(hub.clients)
        return web.json_response(data)

    async def config(_request):
        return web.json_response({
            "mode": "websocket_jpeg_strip",
            "layout": ["left_camera", "right_camera", "right_tactile_camera", "left_tactile_camera"],
            "strip_size": {
                "width": args.tile_width * 4,
                "height": args.tile_height,
            },
            "tile_size": {
                "width": args.tile_width,
                "height": args.tile_height,
            },
            "fps": args.fps,
            "jpeg_quality": args.jpeg_quality,
            "video_ws": "/ws/video",
        })

    async def video_ws(request):
        ws = web.WebSocketResponse(max_msg_size=2 * 1024 * 1024)
        await ws.prepare(request)
        hub.clients.add(ws)
        await ws.send_str(json.dumps({
            "type": "hello",
            "mode": "websocket_jpeg_strip",
            "width": args.tile_width * 4,
            "height": args.tile_height,
            "fps": args.fps,
        }))

        try:
            async for msg in ws:
                if msg.type == web.WSMsgType.TEXT:
                    if msg.data == "ping":
                        await ws.send_str("pong")
                elif msg.type == web.WSMsgType.ERROR:
                    break
        finally:
            hub.clients.discard(ws)
            with contextlib.suppress(Exception):
                await ws.close()

        return ws

    async def on_shutdown(_app):
        for ws in list(hub.clients):
            with contextlib.suppress(Exception):
                await ws.close()
        hub.clients.clear()
        ros_node.destroy_node()
        rclpy.shutdown()

    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_get("/health", health)
    app.router.add_get("/config", config)
    app.router.add_get("/ws/video", video_ws)
    app.cleanup_ctx.append(lambda app: _background_ctx(app, spin_ros))
    app.cleanup_ctx.append(lambda app: _background_ctx(app, lambda: hub.broadcast_loop(args.fps)))
    app.on_shutdown.append(on_shutdown)
    return app


async def _background_ctx(_app, coro_factory):
    task = asyncio.create_task(coro_factory())
    try:
        yield
    finally:
        task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await task


def main():
    parser = argparse.ArgumentParser(description="ROS2 to WebSocket JPEG strip bridge")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--left-topic", default="/left_camera/color/image_raw")
    parser.add_argument("--right-topic", default="/right_camera/color/image_raw")
    parser.add_argument("--right-tactile-topic", default="/right_tactile_camera/color/image_raw")
    parser.add_argument("--left-tactile-topic", default="/left_tactile_camera/color/image_raw")
    parser.add_argument("--tile-width", type=int, default=320)
    parser.add_argument("--tile-height", type=int, default=240)
    parser.add_argument("--fps", type=int, default=12)
    parser.add_argument("--jpeg-quality", type=int, default=75)
    args = parser.parse_args()

    app = make_app(args)
    web.run_app(app, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
