import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Point
from nidar_interfaces.msg import SwarmTask

class AreaPartitionerNode(Node):
    def __init__(self):
        super().__init__('area_partitioner_node')

        self.pub_task = self.create_publisher(
            SwarmTask,
            '/rescueswarm/tasks',
            10
        )

        # 10-hectare search area (316m x 316m)
        self.active_drones = ['drone_1', 'drone_2']
        self.area_width = 316.0
        self.area_height = 316.0

        self.timer = self.create_timer(5.0, self.distribute_tasks)
        self.tasks_assigned = False

        self.get_logger().info('RescueSwarm Area Partitioner & Swarm Task Scheduler Started')

    def distribute_tasks(self):
        if self.tasks_assigned:
            return

        num_drones = len(self.active_drones)
        sub_width = self.area_width / num_drones

        for idx, drone_id in enumerate(self.active_drones):
            x_min = -self.area_width/2.0 + idx * sub_width
            x_max = x_min + sub_width
            y_min = -self.area_height/2.0
            y_max = self.area_height/2.0

            # Generate lawnmower sweep center waypoint
            center_x = (x_min + x_max) / 2.0
            center_y = (y_min + y_max) / 2.0

            task = SwarmTask()
            task.task_id = f"SCOUT_SUBSECTION_{idx+1}"
            task.assigned_drone = drone_id
            task.task_type = "SCOUT"
            task.target_location.x = center_x
            task.target_location.y = center_y
            task.target_location.z = 15.0 # Flight altitude 15m

            # Mock GPS coordinates for 10-ha area (Center lat=28.6139, lon=77.2090)
            task.latitude = 28.6139 + (center_y / 111111.0)
            task.longitude = 77.2090 + (center_x / (111111.0 * math.cos(math.radians(28.6139))))

            self.pub_task.publish(task)
            self.get_logger().info(
                f'Assigned 10-ha Sub-region {idx+1} to {drone_id}: Center=({center_x:.1f}, {center_y:.1f}), Alt=15m'
            )

        self.tasks_assigned = True

def main(args=None):
    rclpy.init(args=args)
    node = AreaPartitionerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
