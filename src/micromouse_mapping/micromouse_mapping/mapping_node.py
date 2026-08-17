import math  # noqa: I001

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from micromouse_mapping.maze_map import MazeMap


class MappingNode(Node):
    """PERCEPCIJA: cita senzore + odometriju, gradi mapu, objavljuje mapu (podatke)
    i pozu robota. Ne planira i ne zna nista o prikazu."""

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
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        # poza robota u okviru 'map' (poravnata na poznatu startnu pozu)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = math.pi / 2  # start: sjever
        self.yaw_offset = None
        self.start_ox = 0.0
        self.start_oy = 0.0

        self.map = MazeMap(16)
        self.grid_pub = self.create_publisher(OccupancyGrid, "/maze_map", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/robot_pose", 10)
        self.suspect_pub = self.create_publisher(Bool, "/localization_suspect", 10)
        self.suspect = False
        self.create_timer(0.2, self.update_and_publish)

    def ir_callback(self, msg, name):
        self.ir_ranges[name] = msg.ranges[0] if msg.ranges else float("inf")

    def odom_callback(self, msg):
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        oyaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        # poravnanje na poznatu startnu pozu: (0,0) gledajuci sjever (+90°)
        if self.yaw_offset is None:
            self.yaw_offset = math.pi / 2 - oyaw
            self.start_ox = ox
            self.start_oy = oy

        c, s = math.cos(self.yaw_offset), math.sin(self.yaw_offset)
        dx, dy = ox - self.start_ox, oy - self.start_oy
        self.robot_x = c * dx - s * dy
        self.robot_y = s * dx + c * dy
        self.robot_yaw = oyaw + self.yaw_offset

    def update_and_publish(self):
        result = self.map.update_from_sensors(
            self.robot_x, self.robot_y, self.robot_yaw, self.ir_ranges
        )
        # result: True=neslaganje, False=cisto ocitanje, None=nije mjereno (nije poravnat)
        if result is True:
            self.suspect = True
            cx, cy, d, known, sensed = self.map.last_conflict
            self.get_logger().warn(
                f"NESLAGANJE u ({cx},{cy}) smjer {d}: mapa kaze {known}, senzor {sensed} "
                f"-> vjerojatno kriva pozicija (mapa NIJE prepisana)"
            )
        elif result is False:
            self.suspect = False

        self.suspect_pub.publish(Bool(data=self.suspect))
        self.grid_pub.publish(self.build_grid())
        self.pose_pub.publish(self.build_pose())

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

    def build_pose(self):
        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = self.robot_x
        p.pose.position.y = self.robot_y
        p.pose.orientation.z = math.sin(self.robot_yaw / 2.0)
        p.pose.orientation.w = math.cos(self.robot_yaw / 2.0)
        return p


def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
