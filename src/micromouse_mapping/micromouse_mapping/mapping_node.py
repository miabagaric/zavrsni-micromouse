#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from micromouse_mapping.maze_map import MazeMap


# smjerovi i rotacije (iste konvencije kao lokalizacija)
N, E, S, W = 0, 1, 2, 3
LEFT_OF = {N: W, W: S, S: E, E: N}
RIGHT_OF = {N: E, E: S, S: W, W: N}

CELL = 0.18
WALL_THR = 0.12  # ir < ovo => zid postoji ispred senzora

# stanje zida u MazeMap (pretpostavljene konstante iz maze_map.py)
# ako se tvoje zovu drukcije, promijeni ovdje
WALL = 1
FREE = 2


def quat_to_heading(q):
    """Poza iz lokalizacije nosi tocan kardinalni kut -> vrati N/E/S/W."""
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    deg = math.degrees(yaw) % 360
    # zaokruzi na najblizi kardinalni (poza je vec kardinalna, ovo je samo citanje)
    if 45 <= deg < 135:
        return N
    if 135 <= deg < 225:
        return W
    if 225 <= deg < 315:
        return S
    return E


class MappingNode(Node):
    """Cita diskretnu pozu (celija + smjer) i senzore. Kad robot miruje,
    upisuje zidove za trenutnu celiju. Objavljuje mapu. Ne racuna pozu."""

    def __init__(self):
        super().__init__("mapping_node")

        # senzori za mapiranje zidova: prednji, lijevi, desni
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

        # diskretna poza iz lokalizacije
        self.cx = 0
        self.cy = 0
        self.heading = N
        self.create_subscription(PoseStamped, "/robot_pose", self.pose_cb, 10)

        # status: mapiramo samo kad robot miruje (IDLE)
        self.status = "UNKNOWN"
        self.create_subscription(String, "/robot_status", self.status_cb, 10)

        self.map = MazeMap(16)
        self.grid_pub = self.create_publisher(OccupancyGrid, "/maze_map", 10)
        self.mapped_pub = self.create_publisher(String, "/mapped_cell", 10)

        # pamti zadnju ozidanu celiju da ne upisujemo isto svaki tick
        self.last_mapped = None

        self.create_timer(0.1, self.tick)

    def _ir(self, name, msg):
        setattr(self, name, msg.ranges[0] if msg.ranges else float("inf"))

    def pose_cb(self, msg):
        # poza je savrsena: centar celije + kardinalni kut
        self.cx = round(msg.pose.position.x / CELL)
        self.cy = round(msg.pose.position.y / CELL)
        self.heading = quat_to_heading(msg.pose.orientation)

    def status_cb(self, msg):
        self.status = msg.data

    def tick(self):
        # mapiraj samo kad robot miruje i tek jednom po celiji
        if self.status == "IDLE" and self.last_mapped != (self.cx, self.cy):
            self.map_current_cell()
            self.last_mapped = (self.cx, self.cy)

        # mapu objavljujemo uvijek (za RViz/planner)
        self.grid_pub.publish(self.build_grid())

    def map_current_cell(self):
        # tri senzora -> tri apsolutna smjera zida (relativno na heading)
        front_dir = self.heading
        left_dir = LEFT_OF[self.heading]
        right_dir = RIGHT_OF[self.heading]

        readings = {
            front_dir: self.ir_front,
            left_dir: self.ir_left,
            right_dir: self.ir_right,
        }
        for direction, r in readings.items():
            state = WALL if r < WALL_THR else FREE
            self.map.set_wall(self.cx, self.cy, direction, state)

        self.get_logger().info(
            f"mapirao ({self.cx},{self.cy}) "
            f"F={self.ir_front:.3f} L={self.ir_left:.3f} R={self.ir_right:.3f}"
        )

        self.mapped_pub.publish(String(data=f"{self.cx},{self.cy}"))

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
