import rclpy
import json
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from micromouse_common.maze_map import MazeMap, quat_to_heading

N, E, S, W = 0, 1, 2, 3
LEFT_OF = {N: W, W: S, S: E, E: N}
RIGHT_OF = {N: E, E: S, S: W, W: N}
CELL = 0.18
WALL_THR = 0.12
WALL = 1
FREE = 2


class MappingNode(Node):
    def __init__(self):
        super().__init__("mapping_node")
        self.ir_front = float("inf")
        self.ir_left = float("inf")
        self.ir_right = float("inf")
        self.create_subscription(
            LaserScan, "/ir_front", lambda m: self._ir("ir_front", m), 10
        )
        self.create_subscription(
            LaserScan, "/ir_left", lambda m: self._ir("ir_left", m), 10
        )
        self.create_subscription(
            LaserScan, "/ir_right", lambda m: self._ir("ir_right", m), 10
        )

        self.cx = 0
        self.cy = 0
        self.heading = N
        self.create_subscription(PoseStamped, "/robot_pose", self.pose_cb, 10)

        self.status = "UNKNOWN"
        self.create_subscription(String, "/robot_status", self.status_cb, 10)

        self.map = MazeMap(16)
        self.grid_pub = self.create_publisher(OccupancyGrid, "/maze_map", 10)
        self.walls_pub = self.create_publisher(String, "/maze_walls", 10)
        self.mapped_pub = self.create_publisher(String, "/mapped_cell", 10)
        self.last_mapped = None
        self.create_timer(0.1, self.tick)

    def _ir(self, name, msg):
        setattr(self, name, msg.ranges[0] if msg.ranges else float("inf"))

    def pose_cb(self, msg):
        self.cx = round(msg.pose.position.x / CELL)
        self.cy = round(msg.pose.position.y / CELL)
        self.heading = quat_to_heading(msg.pose.orientation)

    def status_cb(self, msg):
        self.status = msg.data

    def tick(self):
        if self.status == "IDLE" and self.last_mapped != (self.cx, self.cy):
            self.map_current_cell()
            self.last_mapped = (self.cx, self.cy)
            self.walls_pub.publish(String(data=json.dumps(self.map.walls)))
            self.mapped_pub.publish(String(data=f"{self.cx},{self.cy}"))
        self.grid_pub.publish(self.build_grid())

    def map_current_cell(self):
        readings = {
            self.heading: self.ir_front,
            LEFT_OF[self.heading]: self.ir_left,
            RIGHT_OF[self.heading]: self.ir_right,
        }
        for direction, r in readings.items():
            state = WALL if r < WALL_THR else FREE
            self.map.set_wall(self.cx, self.cy, direction, state)
        self.get_logger().info(
            f"mapirao ({self.cx},{self.cy}) F={self.ir_front:.3f} L={self.ir_left:.3f} R={self.ir_right:.3f}"
        )

    def build_grid(self):
        sub, cell = 3, CELL
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
    rclpy.spin(MappingNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
