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

        # --- ROS 2 komunikacija ---
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/robot_status", 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(String, "/maze_commands", self.command_callback, 10)

        # bocni + prednji senzor
        self.ir_left = float("inf")
        self.ir_right = float("inf")
        self.ir_front = float("inf")
        self.create_subscription(
            LaserScan, "/ir_left", lambda m: self._ir("ir_left", m), 10
        )
        self.create_subscription(
            LaserScan, "/ir_right", lambda m: self._ir("ir_right", m), 10
        )
        self.create_subscription(
            LaserScan, "/ir_front", lambda m: self._ir("ir_front", m), 10
        )

        self.timer = self.create_timer(0.02, self.control_loop)  # 50 Hz kontrola
        self.status_timer = self.create_timer(0.1, self.publish_status)  # 10 Hz status

        # --- poza (kalibrirana, 'map' okvir) ---
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = math.pi / 2  # start: sjever
        self.raw_x = 0.0
        self.raw_y = 0.0
        self.raw_yaw = math.pi / 2

        # jednokratna kalibracija sirove odometrije na poznatu startnu pozu
        self.yaw_offset = None
        self.start_ox = 0.0
        self.start_oy = 0.0

        # fina korekcija iz relokalizacije (ako je koristis; inace 0)
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_yaw = 0.0

        # --- stanje ---
        self.state = "IDLE"
        self.start_x = 0.0
        self.start_y = 0.0
        self.target_distance = 0.0
        self.target_yaw = 0.0
        self.wiggle_ticks = 0

        # --- PID parametri ---
        self.kp_linear, self.kd_linear = 2.0, 0.2
        self.kp_heading = 1.5
        self.kp_angular, self.kd_angular = 2.0, 0.3
        self.prev_error_linear = 0.0
        self.prev_error_angular = 0.0

        # --- centriranje po senzorima ---
        self.kp_center = 2.5  # jacina bocne ispravke (ugadjati!)
        self.target_side = 0.048  # udaljenost do bocnog zida kad je centriran (m)
        self.side_wall_thr = 0.12  # ispod ovoga postoji bocni zid
        self.front_target = (
            0.045  # udaljenost prednjeg senzora do zida pri zaustavljanju (m)
        )
        self.front_wall_thr = 0.12  # ispod ovoga postoji zid ispred

    # ================= POMOCNE =================
    def _ir(self, name, msg):
        setattr(self, name, msg.ranges[0] if msg.ranges else float("inf"))

    def snap_cardinal(self, yaw):
        """Zaokruzi kut na najblizi visekratnik od 90° (N/E/S/W)."""
        return round(yaw / (math.pi / 2)) * (math.pi / 2)

    @staticmethod
    def ang_diff(a, b):
        return math.atan2(math.sin(a - b), math.cos(a - b))

    def odom_callback(self, msg):
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, oyaw = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])

        # jednokratna kalibracija: robot starta u (0,0), gleda sjever (+90°)
        if self.yaw_offset is None:
            self.yaw_offset = math.pi / 2 - oyaw
            self.start_ox = ox
            self.start_oy = oy

        # sirova odom -> kalibrirani 'map' okvir (brzo, 50 Hz)
        c, s = math.cos(self.yaw_offset), math.sin(self.yaw_offset)
        dx, dy = ox - self.start_ox, oy - self.start_oy
        self.raw_x = c * dx - s * dy
        self.raw_y = s * dx + c * dy
        self.raw_yaw = oyaw + self.yaw_offset

        # + fina korekcija iz relokalizacije
        self.current_x = self.raw_x + self.offset_x
        self.current_y = self.raw_y + self.offset_y
        self.current_yaw = self.raw_yaw + self.offset_yaw

    def command_callback(self, msg):
        if self.state != "IDLE":
            return
        cmd = msg.data
        if cmd == "FORWARD":
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.target_distance = 0.180
            self.target_yaw = self.snap_cardinal(
                self.current_yaw
            )  # drzi tocan kardinalni smjer
            self.prev_error_linear = 0.0
            self.state = "FORWARD"
        elif cmd == "TURN_LEFT":
            self.target_yaw = self.snap_cardinal(self.current_yaw + math.pi / 2)
            self.prev_error_angular = 0.0
            self.state = "TURN"
        elif cmd == "TURN_RIGHT":
            self.target_yaw = self.snap_cardinal(self.current_yaw - math.pi / 2)
            self.prev_error_angular = 0.0
            self.state = "TURN"
        elif cmd == "WIGGLE":
            self.wiggle_ticks = 0
            self.state = "WIGGLE"

    def publish_status(self):
        m = String()
        m.data = self.state
        self.status_pub.publish(m)

    def centering_correction(self):
        """Kutna ispravka da robot ostane na sredini koridora (po bocnim senzorima)."""
        L, R = self.ir_left, self.ir_right
        thr = self.side_wall_thr
        if L < thr and R < thr:
            return self.kp_center * (L - R)  # oba zida -> izjednaci
        elif L < thr:
            return self.kp_center * (L - self.target_side)  # samo lijevi
        elif R < thr:
            return self.kp_center * (self.target_side - R)  # samo desni
        else:
            err_head = self.ang_diff(self.target_yaw, self.current_yaw)
            return self.kp_heading * err_head  # nema zidova -> drzi kurs

    # ================= GLAVNA PETLJA =================
    def control_loop(self):
        cmd = Twist()

        if self.state == "IDLE":
            self.cmd_pub.publish(cmd)
            return

        elif self.state == "FORWARD":
            dist = math.hypot(
                self.current_x - self.start_x, self.current_y - self.start_y
            )
            err_lin = self.target_distance - dist
            d_err_lin = err_lin - self.prev_error_linear
            self.prev_error_linear = err_lin

            front_wall = self.ir_front < self.front_wall_thr

            # ZAUSTAVLJANJE (vise nezavisnih uvjeta -> nikad se ne zaglavi):
            stop_by_wall = front_wall and (self.ir_front <= self.front_target)
            stop_by_dist = dist >= self.target_distance  # presao 18 cm
            stop_too_close = self.ir_front <= 0.035  # sigurnost: preblizu zidu
            if stop_by_wall or stop_by_dist or stop_too_close:
                self.state = "IDLE"
                self.cmd_pub.publish(Twist())
                return

            # brzina naprijed
            v = (self.kp_linear * err_lin) + (self.kd_linear * d_err_lin)
            if front_wall:
                v = self.kp_linear * (
                    self.ir_front - self.front_target
                )  # glatko usporavanje uz zid
            cmd.linear.x = max(
                min(v, 0.15), 0.02
            )  # min 0.02 -> uvijek se mice, ne zapne

            # centriranje po senzorima
            cmd.angular.z = max(min(self.centering_correction(), 1.5), -1.5)

        elif self.state == "TURN":
            err_ang = self.ang_diff(self.target_yaw, self.current_yaw)
            d_err_ang = err_ang - self.prev_error_angular
            self.prev_error_angular = err_ang

            if abs(err_ang) < 0.01:
                self.state = "IDLE"
                self.cmd_pub.publish(Twist())
                return
            w = (self.kp_angular * err_ang) + (self.kd_angular * d_err_ang)
            cmd.angular.z = max(min(w, 1.2), -1.2)  # sporiji okret = cisci pivot

        elif self.state == "WIGGLE":
            # kratko poravnanje na mjestu: rotiraj da izjednacis bocne senzore, pa IDLE
            self.wiggle_ticks += 1
            corr = self.centering_correction()
            cmd.angular.z = max(min(corr, 0.8), -0.8)
            if self.wiggle_ticks > 25 or abs(corr) < 0.02:  # ~0.5 s ili poravnat
                self.state = "IDLE"
                self.cmd_pub.publish(Twist())
                return

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MotionController())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
