#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from micromouse_mapping.maze_map import (
    N,
    E,
    S,
    W,
    WALL,
    FREE,
    UNKNOWN,
    LEFT_OF,
    RIGHT_OF,
    OPPOSITE,
)
from micromouse_mapping.flood_fill import FloodFill
from micromouse_mapping.wall_follower import WallFollower


CELL = 0.18

# izbor algoritma istrazivanja: "flood" ili "wall"
ALGO = "flood"


def quat_to_heading(q):
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    deg = math.degrees(yaw) % 360
    if 45 <= deg < 135:
        return N
    if 135 <= deg < 225:
        return W
    if 225 <= deg < 315:
        return S
    return E


class PlannerNode(Node):
    """ODLUCIVANJE: cita mapu + stanje + status. Kad robot miruje, odabrani
    algoritam bira sljedeci smjer i salje JEDNU komandu. Ne racuna pozu, ne vozi."""

    def __init__(self):
        super().__init__("planner_node")
        self.size = 16
        self.sub = 3

        self.walls = None
        self.cx = 0
        self.cy = 0
        self.heading = N
        self.status = "UNKNOWN"

        self.start_cell = (0, 0)
        self.phase = "TO_CENTER"  # samo za flood fill: TO_CENTER -> TO_START -> DONE
        self.last_commanded_state = None  # (cx, cy, heading) kad smo zadnji put poslali

        # --- odabir algoritma ---
        self.algo_name = ALGO
        if ALGO == "flood":
            self.algo = FloodFill(self.size)
        else:
            self.algo = WallFollower(self.size)
        self.get_logger().info(f"Algoritam istrazivanja: {self.algo_name}")

        self.mapped_cell = None
        self.create_subscription(String, "/mapped_cell", self.mapped_cb, 10)
        self.create_subscription(OccupancyGrid, "/maze_map", self.map_cb, 10)
        self.create_subscription(PoseStamped, "/robot_pose", self.pose_cb, 10)
        self.create_subscription(String, "/robot_status", self.status_cb, 10)
        self.cmd_pub = self.create_publisher(String, "/maze_commands", 10)

        self.create_timer(0.1, self.decide)

    def mapped_cb(self, msg):
        parts = msg.data.split(",")
        self.mapped_cell = (int(parts[0]), int(parts[1]))

    def map_cb(self, msg):
        self.walls = self.grid_to_walls(msg)

    def pose_cb(self, msg):
        self.cx = round(msg.pose.position.x / CELL)
        self.cy = round(msg.pose.position.y / CELL)
        self.heading = quat_to_heading(msg.pose.orientation)

    def status_cb(self, msg):
        self.status = msg.data

    def grid_to_walls(self, msg):
        w = msg.info.width
        data = msg.data
        sub, size = self.sub, self.size
        walls = [
            [[UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN] for _ in range(size)]
            for _ in range(size)
        ]

        def pix(px, py):
            return data[py * w + px]

        edges = {
            N: lambda bx, by: (bx + sub // 2, by + sub - 1),
            S: lambda bx, by: (bx + sub // 2, by + 0),
            E: lambda bx, by: (bx + sub - 1, by + sub // 2),
            W: lambda bx, by: (bx + 0, by + sub // 2),
        }
        for x in range(size):
            for y in range(size):
                bx, by = x * sub, y * sub
                for d, fn in edges.items():
                    px, py = fn(bx, by)
                    v = pix(px, py)
                    if v == 100:
                        walls[x][y][d] = WALL
                    elif v == 0:
                        walls[x][y][d] = FREE
        return walls

    def decide(self):
        if self.walls is None or self.status != "IDLE":
            return
        if self.mapped_cell != (self.cx, self.cy):
            return
        current_state = (self.cx, self.cy, self.heading)
        # ne salji novu komandu dok se stanje nije promijenilo od zadnje poslane
        if current_state == self.last_commanded_state:
            return

        cx, cy = self.cx, self.cy
        if not (0 <= cx < self.size and 0 <= cy < self.size):
            return

        # dvije grane odlucivanja, ovisno o algoritmu
        if self.algo_name == "flood":
            best = self.decide_flood(cx, cy)
        else:
            best = self.decide_wall(cx, cy)

        if best is None:
            return  # algoritam je gotov (cilj, petlja, ili faza DONE) — vec logirano

        # --- apsolutni smjer -> relativna komanda (ZAJEDNICKO za oba) ---
        if best == self.heading:
            cmd = "FORWARD"
        elif best == LEFT_OF[self.heading]:
            cmd = "TURN_LEFT"
        else:
            cmd = "TURN_RIGHT"

        self.cmd_pub.publish(String(data=cmd))
        self.last_commanded_state = current_state  # zapamti gdje smo poslali
        self.get_logger().info(
            f"[{self.algo_name}] ({cx},{cy}) {['N', 'E', 'S', 'W'][self.heading]} -> {cmd}"
        )

    def decide_flood(self, cx, cy):
        """Flood fill grana: cilj-detekcija + faze + get_best_move."""
        if (cx, cy) in self.algo.goal_cells:
            if self.phase == "TO_CENTER":
                self.phase = "TO_START"
                self.algo.set_goal([self.start_cell])
                self.get_logger().info(
                    f"CILJ ({cx},{cy})! Povratak na {self.start_cell}."
                )
            elif self.phase == "TO_START":
                self.phase = "DONE"
                self.get_logger().info(f"POVRATAK ZAVRSEN ({cx},{cy}). Stop.")
            return None
        if self.phase == "DONE":
            return None

        self.algo.update_distances(self.walls)
        best = self.algo.get_best_move(cx, cy, self.walls)
        if best is None:
            self.get_logger().warn(f"nema poteza iz ({cx},{cy})")
        return best

    def decide_wall(self, cx, cy):
        wcell = self.walls[cx][cy]
        self.get_logger().info(
            f"[wall] ({cx},{cy}) h={self.heading} walls N={wcell[N]} E={wcell[E]} S={wcell[S]} W={wcell[W]}"
        )
        best = self.algo.get_best_move(cx, cy, self.heading, self.walls)
        if (
            best is None
            and self.algo.finished
            and not getattr(self, "_wall_reported", False)
        ):
            self._wall_reported = True
            if self.algo.reached_goal:
                self.get_logger().info(f"[wall] NASAO CENTAR, koraka={self.algo.steps}")
            else:
                self.get_logger().warn(
                    f"[wall] ODUSTAO (petlja/limit), koraka={self.algo.steps}"
                )
        return best


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PlannerNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
