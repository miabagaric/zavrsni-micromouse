import math  # noqa: I001

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid  # maknut Odometry (ne treba više)
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from micromouse_mapping.maze_map import MazeMap


class MappingNode(Node):
    """PERCEPCIJA: cita senzore + pozu (iz lokalizacije), gradi mapu,
    objavljuje mapu. Ne planira, ne racuna pozu, ne zna nista o prikazu."""

    def __init__(self):
        super().__init__("mapping_node")

        self.ir_names = [
            "ir_left",
            "ir_left_diag",
            "ir_front",
            "ir_right_diag",
            "ir_right",
        ]
        self.ir_ranges = {name: float("inf") for name in self.ir_names}
        for name in self.ir_names:
            self.create_subscription(
                LaserScan, "/" + name, lambda msg, n=name: self.ir_callback(msg, n), 10
            )

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = math.pi / 2
        self.create_subscription(PoseStamped, "/robot_pose", self.pose_callback, 10)

        self.map = MazeMap(16)
        self.grid_pub = self.create_publisher(OccupancyGrid, "/maze_map", 10)
        # [FIX] maknut self.pose_pub — pozu sad objavljuje localization_node
        self.suspect_pub = self.create_publisher(Bool, "/localization_suspect", 10)
        self.suspect = False
        self.create_timer(0.2, self.update_and_publish)

    def ir_callback(self, msg, name):
        self.ir_ranges[name] = msg.ranges[0] if msg.ranges else float("inf")

    def pose_callback(self, msg):
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y
        q = msg.pose.orientation
        self.robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def status_callback(self, msg):  # [FIX] puni self.robot_status
        self.robot_status = msg.data

    def update_and_publish(self):
        result = self.map.update_from_sensors(
            self.robot_x, self.robot_y, self.robot_yaw, self.ir_ranges
        )
        if result is True:
            self.suspect = True
            cx, cy, d, known, sensed = self.map.last_conflict
            self.get_logger().warn(
                f"NESLAGANJE u ({cx},{cy}) smjer {d}: mapa kaze {known}, "
                f"senzor {sensed} -> vjerojatno kriva pozicija (mapa NIJE prepisana)"
            )
        elif result is False:
            self.suspect = False

        # mapu i suspect objavljujemo UVIJEK (izvan IDLE gejta)
        self.suspect_pub.publish(Bool(data=self.suspect))
        self.grid_pub.publish(self.build_grid())

    def build_grid(self):
        sub, cell = 3, 0.18
        n = self.map.size * sub
        grid = self.map.to_grid(sub)

        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = cell / sub
        msg.info.width = n
        msg.info.height = n
        msg.info.origin.position.x = -cell / 2
        msg.info.origin.position.y = -cell / 2
        msg.info.origin.orientation.w = 1.0

        flat = []
        for row in grid:
            flat.extend(row)
        msg.data = flat
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
