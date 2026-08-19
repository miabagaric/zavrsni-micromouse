#!/usr/bin/env python3
import rclpy
import json
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from micromouse_common.maze_map import (
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
    quat_to_heading,
)
from micromouse_planning.flood_fill import FloodFill
from micromouse_planning.wall_follower import WallFollower

CELL = 0.18
ALGO = "flood"  # "flood" ili "wall"


class PlannerNode(Node):
    def __init__(self):
        super().__init__("planner_node")
        self.size = 16
        self.walls = None
        self.cx = 0
        self.cy = 0
        self.heading = N
        self.status = "UNKNOWN"
        self.start_cell = (0, 0)
        self.phase = "TO_CENTER"

        self.algo_name = ALGO
        if ALGO == "flood":
            self.algo = FloodFill(self.size)
        else:
            self.algo = WallFollower(self.size)
        self._wall_reported = False
        self.get_logger().info(f"Algoritam istrazivanja: {self.algo_name}")

        self.mapped_cell = None
        self.last_commanded_state = None
        self.create_subscription(String, "/mapped_cell", self.mapped_cb, 10)
        self.create_subscription(String, "/maze_walls", self.walls_cb, 10)
        self.create_subscription(PoseStamped, "/robot_pose", self.pose_cb, 10)
        self.create_subscription(String, "/robot_status", self.status_cb, 10)
        self.cmd_pub = self.create_publisher(String, "/maze_commands", 10)
        self.create_timer(0.1, self.decide)

    def mapped_cb(self, msg):
        parts = msg.data.split(",")
        self.mapped_cell = (int(parts[0]), int(parts[1]))

    def walls_cb(self, msg):
        self.walls = json.loads(msg.data)

    def pose_cb(self, msg):
        self.cx = round(msg.pose.position.x / CELL)
        self.cy = round(msg.pose.position.y / CELL)
        self.heading = quat_to_heading(msg.pose.orientation)

    def status_cb(self, msg):
        self.status = msg.data

    def decide(self):
        if self.walls is None or self.status != "IDLE":
            return
        if self.algo_name == "wall" and self.algo.finished:
            return
        if self.mapped_cell != (self.cx, self.cy):
            return
        current_state = (self.cx, self.cy, self.heading)
        if current_state == self.last_commanded_state:
            return

        cx, cy = self.cx, self.cy
        if not (0 <= cx < self.size and 0 <= cy < self.size):
            return

        if self.algo_name == "flood":
            best = self.decide_flood(cx, cy)
        else:
            best = self.decide_wall(cx, cy)
        if best is None:
            return

        if best == self.heading:
            cmd = "FORWARD"
        elif best == LEFT_OF[self.heading]:
            cmd = "TURN_LEFT"
        else:
            cmd = "TURN_RIGHT"

        self.cmd_pub.publish(String(data=cmd))
        self.last_commanded_state = current_state
        self.get_logger().info(
            f"[{self.algo_name}] ({cx},{cy}) {['N', 'E', 'S', 'W'][self.heading]} -> {cmd}"
        )

    def decide_flood(self, cx, cy):
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
        best = self.algo.get_best_move(cx, cy, self.heading, self.walls)
        if best is None and self.algo.finished and not self._wall_reported:
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
