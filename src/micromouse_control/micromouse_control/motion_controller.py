#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import transforms3d


class MotionController(Node):
    def __init__(self):
        super().__init__("motion_controller")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/robot_status", 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(String, "/maze_commands", self.command_cb, 10)

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

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.have_odom = False

        # Blok A
        self.CELL = 0.180
        self.FRONT_TARGET = 0.0078
        self.FRONT_WALL_THR = 0.12
        self.V_MAX = 0.15
        self.V_BRAKE = -0.06
        self.STOP_ODOM = 0.003
        self.STOP_WALL = 0.0005
        self.FWD_TOL = 0.05
        self.kp_lin = 3.0
        self.kd_lin = 0.35

        # Blok B (centriranje)
        self.TARGET_SIDE = 0.042
        self.SIDE_WALL_THR = 0.12
        self.kp_center = 4.0
        self.kp_yaw = 1.5
        self.CENTER_DEADBAND = 0.001
        self.W_LIMIT = 1.0

        # Blok C (okret)
        self.TURN_TARGET = math.pi / 2
        self.kp_turn = 2.0
        self.kd_turn = 0.3
        self.TURN_STOP = 0.008
        self.TURN_W_MAX = 1.2
        self.TURN_W_BRAKE = 0.3

        # FSM
        self.state = "IDLE"
        self.anchor_x = 0.0
        self.anchor_y = 0.0
        self.anchor_yaw = 0.0
        self.prev_err = 0.0
        self.prev_front_wall = False
        self.turn_target_yaw = 0.0
        self.prev_turn_err = 0.0

        self.create_timer(0.02, self.control_loop)
        self.create_timer(0.10, self.publish_status)

    def _ir(self, name, msg):
        setattr(self, name, msg.ranges[0] if msg.ranges else float("inf"))

    @staticmethod
    def ang_diff(a, b):
        return math.atan2(math.sin(a - b), math.cos(a - b))

    def odom_cb(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.odom_yaw = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])
        self.have_odom = True

    def publish_status(self):
        m = String()
        m.data = self.state
        self.status_pub.publish(m)

    def command_cb(self, msg):
        if self.state != "IDLE":
            return
        cmd = msg.data
        if cmd == "FORWARD":
            if not self.have_odom:
                return
            self.anchor_x = self.odom_x
            self.anchor_y = self.odom_y
            self.anchor_yaw = self.odom_yaw
            self.prev_err = 0.0
            self.prev_front_wall = self.ir_front < self.FRONT_WALL_THR
            self.state = "FORWARD"
        elif cmd == "TURN_LEFT":
            if not self.have_odom:
                return
            self.turn_target_yaw = self.odom_yaw + self.TURN_TARGET
            self.prev_turn_err = 0.0
            self.state = "TURN"
        elif cmd == "TURN_RIGHT":
            if not self.have_odom:
                return
            self.turn_target_yaw = self.odom_yaw - self.TURN_TARGET
            self.prev_turn_err = 0.0
            self.state = "TURN"

    def centering_correction(self):
        L, R = self.ir_left, self.ir_right
        thr = self.SIDE_WALL_THR
        if L < thr and R < thr:
            err = L - R
        elif L < thr:
            err = L - self.TARGET_SIDE
        elif R < thr:
            err = self.TARGET_SIDE - R
        else:
            err_yaw = self.ang_diff(self.anchor_yaw, self.odom_yaw)
            return self.kp_yaw * err_yaw
        if abs(err) < self.CENTER_DEADBAND:
            return 0.0
        return self.kp_center * err

    def control_loop(self):
        cmd = Twist()
        if self.state == "IDLE":
            self.cmd_pub.publish(cmd)
            return
        if self.state == "FORWARD":
            travelled = math.hypot(
                self.odom_x - self.anchor_x, self.odom_y - self.anchor_y
            )
            front_wall = self.ir_front < self.FRONT_WALL_THR
            if front_wall:
                err = self.ir_front - self.FRONT_TARGET
                stop_tol = self.STOP_WALL
            else:
                err = self.CELL - travelled
                stop_tol = self.STOP_ODOM
            if front_wall != self.prev_front_wall:
                self.prev_err = err
            self.prev_front_wall = front_wall
            if err <= stop_tol:
                dev = travelled - self.CELL
                if abs(dev) > self.FWD_TOL:
                    self.get_logger().warn(
                        f"FORWARD divergencija: presao {travelled * 100:.1f} cm "
                        f"(ocekivano {self.CELL * 100:.1f}, odstupanje {dev * 1000:+.0f} mm) "
                        f"-> lokalizacija se mozda razisla"
                    )
                self.state = "IDLE"
                self.cmd_pub.publish(Twist())
                return
            d_err = err - self.prev_err
            self.prev_err = err
            v = self.kp_lin * err + self.kd_lin * d_err
            cmd.linear.x = max(min(v, self.V_MAX), self.V_BRAKE)
            corr = self.centering_correction()
            cmd.angular.z = max(min(corr, self.W_LIMIT), -self.W_LIMIT)
        elif self.state == "TURN":
            err = self.ang_diff(self.turn_target_yaw, self.odom_yaw)
            if abs(err) < self.TURN_STOP:
                self.state = "IDLE"
                self.cmd_pub.publish(Twist())
                return
            d_err = err - self.prev_turn_err
            self.prev_turn_err = err
            w = self.kp_turn * err + self.kd_turn * d_err
            if w > 0:
                cmd.angular.z = min(w, self.TURN_W_MAX)
            else:
                cmd.angular.z = max(w, -self.TURN_W_MAX)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MotionController())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
