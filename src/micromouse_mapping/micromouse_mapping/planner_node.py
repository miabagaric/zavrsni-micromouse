import math  # noqa: I001

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool

from micromouse_mapping.maze_map import (
    N,
    E,
    S,
    W,
    WALL,
    FREE,
    UNKNOWN,
    yaw_to_heading_strict,
    DELTA,
)
from micromouse_mapping.flood_fill import FloodFill

LEFT_OF = {N: W, W: S, S: E, E: N}
RIGHT_OF = {N: E, E: S, S: W, W: N}
OPPOSITE = {N: S, E: W, S: N, W: E}


class PlannerNode(Node):
    def __init__(self):
        super().__init__("planner_node")
        self.size = 16
        self.sub = 3
        self.cell = 0.18
        self.pos_tolerance = 0.036

        self.walls = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = math.pi / 2
        self.robot_state = "UNKNOWN"
        self.reached_goal = False
        self.suspect = False

        # Taktičke i navigacijske zastavice
        self.just_turned = False
        self.just_wiggled = False
        self.straight_count = 0
        self.prev_cx = 0
        self.prev_cy = 0
        self.virtual_walls = []

        self.flood_fill = FloodFill(self.size)

        self.create_subscription(OccupancyGrid, "/maze_map", self.map_cb, 10)
        self.create_subscription(PoseStamped, "/robot_pose", self.pose_cb, 10)
        self.create_subscription(String, "/robot_status", self.status_cb, 10)
        self.create_subscription(Bool, "/localization_suspect", self.suspect_cb, 10)
        self.command_pub = self.create_publisher(String, "/maze_commands", 10)

        self.create_timer(0.1, self.decide)

    def map_cb(self, msg):
        self.walls = self.grid_to_walls(msg)

    def pose_cb(self, msg):
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y
        q = msg.pose.orientation
        self.robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def status_cb(self, msg):
        self.robot_state = msg.data

    def suspect_cb(self, msg):
        self.suspect = msg.data

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

        for x in range(size):
            for y in range(size):
                bx, by = x * sub, y * sub
                edges = {
                    N: (bx + sub // 2, by + sub - 1),
                    S: (bx + sub // 2, by + 0),
                    E: (bx + sub - 1, by + sub // 2),
                    W: (bx + 0, by + sub // 2),
                }
                for d, (px, py) in edges.items():
                    v = pix(px, py)
                    if v == 100:
                        walls[x][y][d] = WALL
                    elif v == 0:
                        walls[x][y][d] = FREE

        # Primjena virtualnih zidova za zaključavanje centra
        for vx, vy, vd in self.virtual_walls:
            walls[vx][vy][vd] = WALL
            dx, dy = DELTA[vd]
            nx, ny = vx + dx, vy + dy
            if 0 <= nx < size and 0 <= ny < size:
                walls[nx][ny][OPPOSITE[vd]] = WALL

        return walls

    def decide(self):
        if self.walls is None or self.robot_state != "IDLE":
            return

        cx = round(self.robot_x / self.cell)
        cy = round(self.robot_y / self.cell)
        if not (0 <= cx < self.size and 0 <= cy < self.size):
            return

        heading = yaw_to_heading_strict(self.robot_yaw, tolerance_deg=10)
        if heading is None:
            return

        # Provjera centra i zaključavanje
        if not self.reached_goal and (cx, cy) in self.flood_fill.goal_cells:
            self.reached_goal = True

            dx = cx - self.prev_cx
            dy = cy - self.prev_cy
            enter_dir = None
            if dx == 1:
                enter_dir = W
            elif dx == -1:
                enter_dir = E
            elif dy == 1:
                enter_dir = S
            elif dy == -1:
                enter_dir = N

            PERIMETER = [
                (7, 7, S),
                (7, 7, W),
                (8, 7, S),
                (8, 7, E),
                (7, 8, N),
                (7, 8, W),
                (8, 8, N),
                (8, 8, E),
            ]
            self.virtual_walls = [
                edge for edge in PERIMETER if edge != (cx, cy, enter_dir)
            ]
            self.flood_fill.goal_cells = [(0, 0)]
            self.get_logger().info(
                "CILJ DOSEGNUT! Zaključavam centar i vraćam se na start."
            )

        elif self.reached_goal and (cx, cy) == (0, 0):
            self.get_logger().info("POVRATAK ZAVRSEN! Spreman za Fast Run.")
            return

        # Ažuriramo prethodnu poziciju
        self.prev_cx = cx
        self.prev_cy = cy

        self.flood_fill.update_distances(self.walls)
        best = self.flood_fill.get_best_move(cx, cy, self.walls)
        if best is None:
            return

        # --- TAKTIČKI WIGGLE ---
        front_wall = self.walls[cx][cy][heading] == WALL
        left_wall = self.walls[cx][cy][LEFT_OF[heading]] == WALL
        right_wall = self.walls[cx][cy][RIGHT_OF[heading]] == WALL

        if not self.just_wiggled:
            do_wiggle = False

            if (
                self.just_turned
                or best != heading
                and front_wall
                or (
                    best == heading
                    and self.straight_count >= 3
                    and (left_wall or right_wall)
                )
            ):
                do_wiggle = True

            if do_wiggle:
                self.just_wiggled = True
                self.just_turned = False
                msg = String()
                msg.data = "WIGGLE"
                self.get_logger().info("Odluka: WIGGLE (taktičko poravnanje)")
                self.command_pub.publish(msg)
                return

        self.just_wiggled = False
        msg = String()

        if best == heading:
            msg.data = "FORWARD"
            self.just_turned = False
            self.straight_count += 1
        elif best == LEFT_OF[heading]:
            msg.data = "TURN_LEFT"
            self.just_turned = True
            self.straight_count = 0
        elif best == RIGHT_OF[heading]:
            msg.data = "TURN_RIGHT"
            self.just_turned = True
            self.straight_count = 0
        elif best == OPPOSITE[heading]:
            msg.data = "TURN_RIGHT"
            self.just_turned = True
            self.straight_count = 0

        if msg.data:
            self.get_logger().info(f"Odluka: {msg.data}")
            self.command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
