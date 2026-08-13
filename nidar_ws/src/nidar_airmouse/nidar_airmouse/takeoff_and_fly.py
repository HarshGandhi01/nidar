import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time

class TakeoffAndFlyNode(Node):
    def __init__(self):
        super().__init__('takeoff_and_fly_node')

        self.pub_enable = self.create_publisher(Bool, '/airmouse/enable', 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_airmouse_cmd = self.create_publisher(Twist, '/airmouse/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.state = 'TAKEOFF'
        self.counter = 0

        self.get_logger().info('Takeoff & Flight Controller Node Started')

        # Enable flight motors
        msg = Bool()
        msg.data = True
        self.pub_enable.publish(msg)

    def control_loop(self):
        cmd = Twist()
        self.counter += 1

        if self.state == 'TAKEOFF':
            # Vertical ascent thrust
            cmd.linear.z = 0.8
            if self.counter > 30: # 3 seconds of ascent
                self.state = 'HOVER'
                self.counter = 0
                self.get_logger().info('Hover altitude reached (~1.5m). Holding position.')

        elif self.state == 'HOVER':
            cmd.linear.z = 0.0 # Maintain hover altitude
            if self.counter > 30: # 3 seconds hover
                self.state = 'FORWARD'
                self.counter = 0
                self.get_logger().info('Moving forward through corridor...')

        elif self.state == 'FORWARD':
            cmd.linear.x = 0.5
            cmd.linear.z = 0.0
            if self.counter > 50: # 5 seconds forward
                self.state = 'TURN'
                self.counter = 0
                self.get_logger().info('Turning corner...')

        elif self.state == 'TURN':
            cmd.angular.z = 0.5
            cmd.linear.z = 0.0
            if self.counter > 30:
                self.state = 'FORWARD'
                self.counter = 0

        self.pub_cmd.publish(cmd)
        self.pub_airmouse_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TakeoffAndFlyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
