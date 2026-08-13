import rclpy
from rclpy.node import Node
from nidar_interfaces.msg import Survivor, DroneStatus
from std_msgs.msg import String

class PayloadDeliveryControllerNode(Node):
    def __init__(self):
        super().__init__('payload_delivery_controller_node')

        self.pending_deliveries = []
        self.delivered_survivors = []

        self.create_subscription(
            Survivor,
            '/rescueswarm/survivor_detected',
            self.survivor_cb,
            10
        )

        self.pub_delivery_status = self.create_publisher(
            String,
            '/rescueswarm/delivery_status',
            10
        )

        self.timer = self.create_timer(3.0, self.process_deliveries)

        self.get_logger().info('RescueSwarm Payload Delivery & Drop Controller Started')

    def survivor_cb(self, msg):
        self.pending_deliveries.append(msg)
        self.get_logger().info(f'Received delivery request for Survivor #{msg.id} at Lat: {msg.latitude:.6f}, Lon: {msg.longitude:.6f}')

    def process_deliveries(self):
        if not self.pending_deliveries:
            return

        surv = self.pending_deliveries.pop(0)
        assigned_drone = 'drone_1' if (surv.id % 2 == 1) else 'drone_2'

        self.delivered_survivors.append(surv.id)
        
        status_msg = String()
        status_msg.data = f"DELIVERED: Payload (200g 20x10x5cm) dropped near Survivor #{surv.id} by {assigned_drone} (Lat: {surv.latitude:.6f}, Lon: {surv.longitude:.6f})"
        
        self.pub_delivery_status.publish(status_msg)
        self.get_logger().info(status_msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = PayloadDeliveryControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
