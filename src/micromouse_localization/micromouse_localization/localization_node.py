#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

N, E, S, W = 0, 1, 2, 3
DELTA = {N: (0, 1), E: (1, 0), S: (0, -1), W: (-1, 0)}
LEFT_OF = {N: W, W: S, S: E, E: N}
RIGHT_OF = {N: E, E: S, S: W, W: N}
HEADING_YAW = {N: math.pi / 2, E: 0.0, S: -math.pi / 2, W: math.pi}
CELL = 0.18


class LocalizationNode(Node):
    def __init__(self):
        super().__init__("localization_node")
        self.cx = 0
        self.cy = 0
        self.heading = N
        self.pending_cmd = None
        self.prev_status = "IDLE"
        self.create_subscription(String, "/maze_commands", self.command_cb, 10)
        self.create_subscription(String, "/robot_status", self.status_cb, 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/robot_pose", 10)
        self.create_timer(0.1, self.publish_pose)

    def command_cb(self, msg):
        if msg.data in ("FORWARD", "TURN_LEFT", "TURN_RIGHT"):
            self.pending_cmd = msg.data

    def status_cb(self, msg):
        status = msg.data
        finished = status == "IDLE" and self.prev_status != "IDLE"
        self.prev_status = status
        if finished and self.pending_cmd is not None:
            self.apply(self.pending_cmd)
            self.pending_cmd = None

    def apply(self, cmd):
        if cmd == "FORWARD":
            dx, dy = DELTA[self.heading]
            self.cx += dx
            self.cy += dy
        elif cmd == "TURN_LEFT":
            self.heading = LEFT_OF[self.heading]
        elif cmd == "TURN_RIGHT":
            self.heading = RIGHT_OF[self.heading]
        self.get_logger().info(
            f"stanje: ({self.cx}, {self.cy}) gledam {['N', 'E', 'S', 'W'][self.heading]}"
        )

    def publish_pose(self):
        yaw = HEADING_YAW[self.heading]
        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = self.cx * CELL
        p.pose.position.y = self.cy * CELL
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        self.pose_pub.publish(p)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LocalizationNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
