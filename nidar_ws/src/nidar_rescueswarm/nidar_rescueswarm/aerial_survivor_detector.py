import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, NavSatFix
from nidar_interfaces.msg import Survivor

class AerialSurvivorDetectorNode(Node):
    def __init__(self):
        super().__init__('aerial_survivor_detector_node')

        self.bridge = CvBridge()
        self.drone_gps = {} # drone_id -> (lat, lon, alt)
        self.survivors = []
        self.survivor_count = 1

        self.create_subscription(Image, '/drone_1/downward_camera/image_raw', lambda msg: self.cam_cb(msg, 'drone_1'), 10)
        self.create_subscription(Image, '/drone_2/downward_camera/image_raw', lambda msg: self.cam_cb(msg, 'drone_2'), 10)
        
        self.create_subscription(NavSatFix, '/drone_1/navsat', lambda msg: self.gps_cb(msg, 'drone_1'), 10)
        self.create_subscription(NavSatFix, '/drone_2/navsat', lambda msg: self.gps_cb(msg, 'drone_2'), 10)

        self.pub_survivor = self.create_publisher(Survivor, '/rescueswarm/survivor_detected', 10)

        self.get_logger().info('RescueSwarm Aerial Survivor Geotagging Detector Started')

    def gps_cb(self, msg, drone_id):
        self.drone_gps[drone_id] = (msg.latitude, msg.longitude, msg.altitude)

    def cam_cb(self, msg, drone_id):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            return

        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
        mask = mask1 | mask2

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            if cv2.contourArea(cnt) > 300:
                if drone_id in self.drone_gps:
                    lat, lon, alt = self.drone_gps[drone_id]

                    # Deduplicate nearby geotags (< 0.0001 deg ~ 10m)
                    if not self.is_duplicate_geotag(lat, lon):
                        self.survivors.append((lat, lon))

                        surv = Survivor()
                        surv.id = self.survivor_count
                        surv.track_id = 'RescueSwarm'
                        surv.latitude = float(lat)
                        surv.longitude = float(lon)
                        surv.confidence = 0.95
                        surv.status = 'AWAITING_DELIVERY'

                        self.pub_survivor.publish(surv)
                        self.get_logger().info(
                            f'Survivor #{self.survivor_count} GEOTAGGED by {drone_id} at Lat: {lat:.6f}, Lon: {lon:.6f}'
                        )
                        self.survivor_count += 1

    def is_duplicate_geotag(self, lat, lon):
        for (slat, slon) in self.survivors:
            if math.hypot(lat - slat, lon - slon) < 0.0001:
                return True
        return False

def main(args=None):
    rclpy.init(args=args)
    node = AerialSurvivorDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
