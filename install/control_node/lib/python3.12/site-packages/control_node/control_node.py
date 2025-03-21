import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.subscription = self.create_subscription(String, 'input_topic', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.energy = 100  # Robot's energy

    def listener_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.data}')

        if self.energy <= 0:
            self.get_logger().info("Energy is depleted. Please say 'Heal' to recharge.")
            return

        command = msg.data.lower()

        twist = Twist()

        if 'left' in command:
            self.get_logger().info(f'Linear velocity: {twist.linear}, Angular velocity: {twist.angular.z}')
            twist.angular.z = 1.0  # Rotate left
            print("LEFT -> angular is set to 1.0")
            self.get_logger().info(f'Linear velocity: {twist.linear}, Angular velocity: {twist.angular.z}')

        elif 'right' in command:
            self.get_logger().info(f'Linear velocity: {twist.linear}, Angular velocity: {twist.angular.z}')
            twist.angular.z = -1.0  # Rotate right
            print("RIGHT -> angular is set to -1.0")
            self.get_logger().info(f'Linear velocity: {twist.linear}, Angular velocity: {twist.angular.z}')

        elif 'forward' in command:
            duration = 2 if 'second' not in command else int(command.split()[1])
            self.move_forward(duration)
            return

        if twist.angular.z != 0:
            self.publisher_.publish(twist)
            self.get_logger().info(f'Energy was: {self.energy}')
            self.energy -= 20  # Consumes 20 energy for each movement
            self.get_logger().info(f'Energy remaining: {self.energy}')

        if 'heal' in command:
            self.energy = 100
            self.get_logger().info(f'Energy fully restored: {self.energy}')

    def move_forward(self, duration):
        twist = Twist()
        twist.linear.x = 0.5  # Move forward
        self.publisher_.publish(twist)
        self.get_logger().info(f'Moving forward for {duration} seconds.')
        time.sleep(duration)
        twist.linear.x = 0.0  # Stop the robot
        self.publisher_.publish(twist)
        self.energy -= 20  # Consumes 20 energy for each movement
        self.get_logger().info(f'Energy remaining: {self.energy}')


def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
