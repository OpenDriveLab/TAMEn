import rclpy
from rclpy.node import Node
import threading
import socket
from std_msgs.msg import String  # Import the standard string message type
import time

class VrSocketServer(Node):

    def __init__(self):
        super().__init__('vr_socket_server')
        self.publisher_ = self.create_publisher(String, '/vr_raw_data', 10)  # Create publisher
        self.declare_parameter('server_port', 8000)
        self.server_port = self.get_parameter('server_port').get_parameter_value().integer_value
        self.server_thread = None
        self.server_running = False
        self.server_socket = None  # Keep a socket reference for shutdown
        self.active_connections = []  # Track all active connections

    def start_server(self, host='0.0.0.0', port=8000):
        self.server_running = True
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            # Enable SO_REUSEADDR so the port can be reused immediately
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            # Enable SO_REUSEPORT on Linux so multiple sockets can bind the same port
            try:
                self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except AttributeError:
                pass  # Windows does not support SO_REUSEPORT
            # Set an accept timeout so shutdown is not blocked
            self.server_socket.settimeout(1.0)
            self.server_socket.bind((host, port))
            self.server_socket.listen()
            self.get_logger().info(f"Server is listening on {host}:{port}")
            while self.server_running:
                try:
                    conn, addr = self.server_socket.accept()
                    self.get_logger().info(f"Connected by {addr}")
                    # Set receive timeout (5 seconds)
                    conn.settimeout(5.0)
                    self.active_connections.append(conn)  # Track the connection
                    client_thread = threading.Thread(
                        target=self.handle_client, 
                        args=(conn, addr),
                        daemon=True
                    )
                    client_thread.start()
                except socket.timeout:
                    # Accept timeout is expected and used to check server_running
                    continue
                except Exception as e:
                    if self.server_running:  # Only log errors while the server is still running
                        self.get_logger().info(f"Error accepting connection: {e}")
        finally:
            if self.server_socket:
                self.server_socket.close()
                self.server_socket = None

    def handle_client(self, conn, addr):
        buffer = ""
        packet_count = 0
        start_time = time.time()
        try:
            while self.server_running:
                data = conn.recv(1024)
                if not data:
                    self.get_logger().info(f"Client {addr} disconnected.")
                    break
                buffer += data.decode('utf-8')
                while '\n' in buffer:
                    index = buffer.index('\n') + 1
                    decoded_data = buffer[:index].strip()
                    buffer = buffer[index:]
                    # Publish the message to the /vr_raw_data topic
                    msg = String()
                    msg.data = decoded_data
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Published to /vr_raw_data: '{msg.data}'")
        finally:
            # Remove the connection from the active list
            if conn in self.active_connections:
                self.active_connections.remove(conn)
            conn.close()

    def start_server_thread(self):
        self.server_thread = threading.Thread(target=self.start_server, args=('0.0.0.0', self.server_port))
        self.server_thread.daemon = True
        self.server_thread.start()

    def stop_server(self):
        self.server_running = False
        
        # Close all active client connections
        for conn in self.active_connections[:]:  # Iterate over a copy
            try:
                conn.close()
            except:
                pass
        self.active_connections.clear()
        
        # Close the server socket
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
            self.server_socket = None
        
        # Wait for the server thread to exit
        if self.server_thread:
            self.server_thread.join(timeout=3.0)  # Wait up to 3 seconds
            if self.server_thread.is_alive():
                self.get_logger().warn("Server thread did not stop gracefully")
        
        self.get_logger().info("Server stopped.")

def main(args=None):
    rclpy.init(args=args)
    vr_socket_server = VrSocketServer()
    vr_socket_server.start_server_thread()

    try:
        rclpy.spin(vr_socket_server)
    except KeyboardInterrupt:
        vr_socket_server.get_logger().info("KeyboardInterrupt caught. Stopping server...")
    finally:
        vr_socket_server.stop_server()
        vr_socket_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()