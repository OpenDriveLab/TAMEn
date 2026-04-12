import rclpy
from rclpy.node import Node
import threading
import socket
from std_msgs.msg import String  # Import the standard string message type

class VrSocketServer(Node):

    def __init__(self):
        super().__init__('vr_socket_server')
        self.publisher_ = self.create_publisher(String, '/vr_raw_data', 10)  # Create publisher
        self.declare_parameter('server_port', 8000)
        self.server_port = self.get_parameter('server_port').get_parameter_value().integer_value
        self.start_server_thread()

    def start_server(self, host='0.0.0.0', port=8000):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            # Enable SO_REUSEADDR to quickly reuse the address and port
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                s.bind((host, port))
                s.listen(5)
                self.get_logger().info(f"Server started on {host}:{port}")
                while True:
                    conn, addr = s.accept()
                    with conn:
                        self.get_logger().info(f"Connected by {addr}")
                        while True:
                            data = conn.recv(1024)
                            if not data:
                                break
                            message = data.decode()
                            self.get_logger().info(f"Received: {message}")
                            
                            # Publish the message to the /vr_data topic
                            msg = String()
                            msg.data = message
                            self.publisher_.publish(msg)
                            self.get_logger().info(f"Published to /vr_raw_data: '{msg.data}'")

                            # Echo back to the client (optional)
                            # conn.sendall(data)
            except KeyboardInterrupt:
                self.get_logger().info("Caught keyboard interrupt, exiting")
            finally:
                s.close()  # Make sure the socket is closed in all cases

    def start_server_thread(self):
        server_thread = threading.Thread(target=self.start_server(port=self.server_port))
        server_thread.daemon = True
        server_thread.start()

def main(args=None):
    rclpy.init(args=args)
    vr_socket_server = VrSocketServer()

    try:
        rclpy.spin(vr_socket_server)
    except KeyboardInterrupt:
        pass
    finally:
        vr_socket_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()