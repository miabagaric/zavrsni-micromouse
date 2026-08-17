import math

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
    LEFT_OF,
    RIGHT_OF,
    OPPOSITE,
    yaw_to_heading_strict,
)
from micromouse_mapping.flood_fill import FloodFill


class PlannerNode(Node):
    def __init__(self):
        super().__init__("planner_node")
        self.size = 16
        self.sub = 3
        self.cell = 0.18

        self.walls = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = math.pi / 2
        self.robot_state = "UNKNOWN"
        self.reached_goal = False
        self.suspect = False
        self.start_cell = (0, 0)
        self.phase = "TO_CENTER"  # TO_CENTER -> TO_START

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
        return walls

    def decide(self):
        # planiraj samo kad: imamo mapu, robot je slobodan (handshake),
        # i lokalizacija nije sumnjiva
        if self.walls is None or self.robot_state != "IDLE":
            return
        if self.suspect:
            return

        # poza dolazi iz lokalizacije: vec korigirana i yaw-snapana.
        cx = round(self.robot_x / self.cell)
        cy = round(self.robot_y / self.cell)
        if not (0 <= cx < self.size and 0 <= cy < self.size):
            return

        heading = yaw_to_heading_strict(self.robot_yaw, tolerance_deg=10)
        if heading is None:
            return  # sigurnosna; s lokalizacijom bi kut vec trebao biti cist

        # --- cilj dosegnut? ---
        if (cx, cy) in self.flood_fill.goal_cells:
            if self.phase == "TO_CENTER":
                self.phase = "TO_START"
                self.flood_fill.set_goal([self.start_cell])
                self.flood_fill.update_distances(self.walls)
                self.get_logger().info(
                    f"CILJ DOSEGNUT ({cx},{cy})! Povratak na {self.start_cell}."
                )
            elif not self.reached_goal:
                self.reached_goal = True
                self.get_logger().info(f"POVRATAK ZAVRSEN ({cx},{cy}). Stop.")
            return

        # --- flood fill: koji je najbolji sljedeci smjer ---
        self.flood_fill.update_distances(self.walls)
        best = self.flood_fill.get_best_move(cx, cy, self.walls)
        if best is None:
            return

        # --- prevedi apsolutni smjer u komandu relativnu na trenutni heading ---
        msg = String()
        if best == heading:
            msg.data = "FORWARD"
        elif best == LEFT_OF[heading]:
            msg.data = "TURN_LEFT"
        else:  # RIGHT_OF ili OPPOSITE -> okreni desno (iduci ciklus nastavi)
            msg.data = "TURN_RIGHT"

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
