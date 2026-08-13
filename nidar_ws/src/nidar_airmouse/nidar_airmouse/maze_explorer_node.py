import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class MazeExplorerNode(Node):
    def __init__(self):
        super().__init__('maze_explorer_node')

        self.sub_scan = self.create_subscription(
            LaserScan,
            '/airmouse/scan',
            self.scan_callback,
            10
        )

        self.sub_odom = self.create_subscription(
            Odometry,
            '/airmouse/odom',
            self.odom_callback,
            10
        )

        self.pub_cmd_vel = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.current_pose = (0.0, 0.0, 0.0)
        self.state = 'EXPLORE' # EXPLORE, TURN_CORNER, RETURNING
        self.linear_speed = 0.35
        self.angular_speed = 0.45

        self.get_logger().info('AirMouse Indoor Maze Autonomous Explorer Started')

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.current_pose = (pos.x, pos.y, 0.0)

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        # Filter invalid ranges
        ranges = np.nan_to_num(ranges, nan=15.0, posinf=15.0, neginf=0.1)

        num_samples = len(ranges)
        if num_samples == 0:
            return

        # Divide scan into FOV sectors (Front, Left, Right, Rear)
        front_sector = np.min(np.concatenate((ranges[:30], ranges[-30:])))
        left_sector = np.min(ranges[60:120])
        right_sector = np.min(ranges[240:300])

        twist = Twist()

        # Wall avoidance & corridor navigation logic
        if front_sector < 0.6:
            # Obstacle/wall ahead -> turn towards open space
            twist.linear.x = 0.05
            if left_sector > right_sector:
                twist.angular.z = self.angular_speed
            else:
                twist.angular.z = -self.angular_speed
        else:
            # Move forward while keeping distance from side walls
            twist.linear.x = self.linear_speed
            if left_sector < 0.5:
                twist.angular.z = -0.2
            elif right_sector < 0.5:
                twist.angular.z = 0.2
            else:
                twist.angular.z = 0.0

        self.pub_cmd_vel.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MazeExplorerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
