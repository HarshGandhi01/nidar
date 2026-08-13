import rclpy
from rclpy.node import Node
import time
from nidar_interfaces.msg import DroneStatus, Survivor
from std_msgs.msg import String

class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager_node')

        self.start_time = time.time()
        self.max_flight_time_sec = 1800 # 30 minutes
        self.survivors_found = 0

        self.pub_status = self.create_publisher(
            DroneStatus,
            '/airmouse/status',
            10
        )

        self.sub_survivor = self.create_subscription(
            Survivor,
            '/airmouse/survivor_detected',
            self.survivor_callback,
            10
        )

        self.sub_abort = self.create_subscription(
            String,
            '/airmouse/abort',
            self.abort_callback,
            10
        )

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.mission_state = 'EXPLORING'

        self.get_logger().info('AirMouse Mission Lifecycle Manager Started')

    def survivor_callback(self, msg):
        self.survivors_found += 1
        self.get_logger().info(f'Mission Manager recorded survivor count: {self.survivors_found}/6')
        if self.survivors_found >= 6:
            self.get_logger().info('All 6 survivors located! Triggering return to exit point.')
            self.mission_state = 'RETURNING_TO_EXIT'

    def abort_callback(self, msg):
        self.get_logger().warn(f'SAFETY ABORT TRIGGERED: {msg.data}')
        self.mission_state = 'ABORTED'

    def timer_callback(self):
        elapsed = time.time() - self.start_time
        remaining = self.max_flight_time_sec - elapsed

        if remaining <= 0 and self.mission_state != 'ABORTED':
            self.get_logger().warn('30-Minute Flight Limit Reached! Returning to exit.')
            self.mission_state = 'TIMEOUT_RETURN'

        status_msg = DroneStatus()
        status_msg.drone_id = 'AirMouse_1'
        status_msg.battery_percentage = max(0.0, 100.0 - (elapsed / 18.0)) # Simulated battery discharge over 30 min
        status_msg.state = self.mission_state
        status_msg.payload_present = False

        self.pub_status.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
