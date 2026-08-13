import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from nidar_interfaces.msg import Survivor
import math

class SurvivorDetectorNode(Node):
    def __init__(self):
        super().__init__('survivor_detector_node')
        
        self.bridge = CvBridge()
        self.current_pose = (0.0, 0.0, 0.0) # x, y, yaw
        self.detected_survivors = [] # list of (x, y, grid_coord)
        self.survivor_id_counter = 1

        self.sub_odom = self.create_subscription(
            Odometry,
            '/airmouse/odom',
            self.odom_callback,
            10
        )

        self.sub_cam = self.create_subscription(
            Image,
            '/airmouse/camera/image_raw',
            self.image_callback,
            10
        )

        self.pub_survivor = self.create_publisher(
            Survivor,
            '/airmouse/survivor_detected',
            10
        )

        self.get_logger().info('AirMouse Survivor Detector & Grid Mapper Node Started')

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        
        # Quaternion to yaw
        siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.current_pose = (pos.x, pos.y, yaw)

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        # Red target / human dummy detection in HSV space
        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 1200: # Significant visual detection
                M = cv2.moments(cnt)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                    # Estimate distance from bounding box / area
                    est_dist = math.sqrt(20000.0 / area)
                    
                    # Compute global world coordinate based on drone pose + estimated distance
                    rx, ry, ryaw = self.current_pose
                    target_x = rx + est_dist * math.cos(ryaw)
                    target_y = ry + est_dist * math.sin(ryaw)

                    # Map to NIDAR Grid Box (15x15m grid divided into 1.5m cells -> Rows A-J, Cols 1-10)
                    col_idx = int(clamp(target_x, 0.0, 14.99) / 1.5) + 1
                    row_char = chr(ord('A') + int(clamp(target_y, 0.0, 14.99) / 1.5))
                    grid_coord = f"{row_char}{col_idx}"

                    # Deduplicate nearby survivor detections (< 1.2m apart)
                    if not self.is_duplicate(target_x, target_y):
                        self.detected_survivors.append((target_x, target_y, grid_coord))
                        
                        surv_msg = Survivor()
                        surv_msg.id = self.survivor_id_counter
                        surv_msg.track_id = 'AirMouse'
                        surv_msg.local_pos.x = float(target_x)
                        surv_msg.local_pos.y = float(target_y)
                        surv_msg.local_pos.z = 0.0
                        surv_msg.grid_coordinate = grid_coord
                        surv_msg.confidence = 0.92
                        surv_msg.status = 'DETECTED'

                        self.pub_survivor.publish(surv_msg)
                        self.get_logger().info(
                            f'Survivor #{self.survivor_id_counter} DETECTED at ({target_x:.2f}, {target_y:.2f}) -> Grid {grid_coord}'
                        )
                        self.survivor_id_counter += 1

    def is_duplicate(self, x, y):
        for (sx, sy, _) in self.detected_survivors:
            if math.hypot(x - sx, y - sy) < 1.2:
                return True
        return False

def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))

def main(args=None):
    rclpy.init(args=args)
    node = SurvivorDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
