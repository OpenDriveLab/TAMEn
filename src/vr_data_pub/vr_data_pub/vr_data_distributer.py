import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class VRDataDistributor(Node):

    def __init__(self):
        super().__init__('vr_data_distributor')
        
        # Create a subscriber for the /vr_raw_data topic
        self.subscription = self.create_subscription(
            String,
            '/vr_raw_data',
            self.listener_callback,
            10)
        self.subscription  # Prevent the unused-variable warning
        
        # Create 10 publishers for the split data streams
        self.publisher_dict = {}
        self.topic_name_array = ['/time_stamp1', '/vr_left_side_button', '/vr_right_side_button', 
                                 '/vr_left_front_button', '/vr_right_front_button', '/left_arm/vr_pose', 'right_arm/vr_pose', 
                                 '/vr_x_button', '/vr_a_button', '/vr_y_button', '/vr_b_button']
                                ##  ,'/vr_reserve3', '/vr_reserve4', '/vr_reserve5']
        for i in range(len(self.topic_name_array)):
            topic_name = self.topic_name_array[i]
            self.publisher_dict[topic_name] = self.create_publisher(String, topic_name, 10)

    def listener_callback(self, msg):
        """Process incoming messages, split them, and publish them to different topics."""
        raw_data = msg.data
        parts = raw_data.split(';')  # Split using semicolons as separators
        print("data length", len(parts))
        
#        if len(parts) != 12:
#            self.get_logger().warn(f"Received message does not contain exactly 10 parts: {raw_data}")
#            return
        
        for i, part in enumerate(parts, start=0):
            topic_name = self.topic_name_array[i]
            publisher = self.publisher_dict[topic_name]
            new_msg = String()
            new_msg.data = part.strip()  # Remove possible extra whitespace
            publisher.publish(new_msg)
            self.get_logger().info(f'Published to {topic_name}: {new_msg.data}')

def main(args=None):
    rclpy.init(args=args)

    vr_data_distributor = VRDataDistributor()

    try:
        rclpy.spin(vr_data_distributor)
    except KeyboardInterrupt:
        pass

    vr_data_distributor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()