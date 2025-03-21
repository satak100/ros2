import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class InputNode(Node):
    def __init__(self):
        super().__init__('input_node')
        self.publisher_ = self.create_publisher(String, 'input_topic', 10)
        self.file_path = '/mnt/c/shared/audio_input.txt'  # Update the path to your input file

        # Timer to read the file periodically
        self.timer = self.create_timer(1.0, self.check_file)

    def check_file(self):
        if os.path.getsize(self.file_path) > 0:
            with open(self.file_path, 'r') as file:
                content = file.read()
                self.get_logger().info(f'Reading from file: {content}')

                # Publish the content to the input topic
                msg = String()
                msg.data = content
                self.publisher_.publish(msg)

            # Clear the file content
            with open(self.file_path, 'w') as file:
                file.truncate(0)


def main(args=None):
    rclpy.init(args=args)
    input_node = InputNode()
    rclpy.spin(input_node)
    input_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
