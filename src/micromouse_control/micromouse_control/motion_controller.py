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
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.command_sub = self.create_subscription(
            String, "/maze_commands", self.command_callback, 10
        )

        self.ir_left = float("inf")
        self.ir_right = float("inf")
        self.ir_front = float("inf")
        self.ir_left_diag = float("inf")
        self.ir_right_diag = float("inf")

        self.create_subscription(
            LaserScan, "/ir_left", lambda m: self._ir("ir_left", m), 10
        )
        self.create_subscription(
            LaserScan, "/ir_right", lambda m: self._ir("ir_right", m), 10
        )
        self.create_subscription(
            LaserScan, "/ir_front", lambda m: self._ir("ir_front", m), 10
        )
        self.create_subscription(
            LaserScan, "/ir_left_diag", lambda m: self._ir("ir_left_diag", m), 10
        )
        self.create_subscription(
            LaserScan, "/ir_right_diag", lambda m: self._ir("ir_right_diag", m), 10
        )

        self.timer = self.create_timer(0.02, self.control_loop)
        self.status_timer = self.create_timer(0.1, self.publish_status)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.state = "IDLE"
        self.start_x = 0.0
        self.start_y = 0.0
        self.target_distance = 0.0
        self.target_yaw = 0.0

        self.kp_linear, self.kd_linear = 2.0, 0.35
        self.kp_heading = 1.5
        self.kp_angular, self.kd_angular = 2.0, 0.3
        self.prev_error_linear = 0.0
        self.prev_error_angular = 0.0
        self.prev_front_wall = False

        self.kp_center, self.kd_center = 4.0, 0.8
        self.center_deadband = (
            0.003  # [m] mrtva zona - sum na IR ne smije izazvati korekciju
        )
        self.lat_filter_alpha = (
            0.4  # niskopropusni filter lateralne greske (0..1, manje = glace)
        )
        self.lat_error_filtered = 0.0
        self.prev_lat_error = 0.0
        self.target_side = 0.048
        self.side_wall_thr = 0.12
        self.front_target = 0.02
        self.front_wall_thr = 0.12
        self.brake_min_v = -0.06  # aktivno kocenje: dopusteni max "unatrag" pogon

    def _ir(self, name, msg):
        setattr(self, name, msg.ranges[0] if msg.ranges else float("inf"))

    @staticmethod
    def ang_diff(a, b):
        return math.atan2(math.sin(a - b), math.cos(a - b))

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.current_yaw = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])

    def align_with_sensors(self):
        ideal_x = round(self.current_x / 0.180) * 0.180
        ideal_y = round(self.current_y / 0.180) * 0.180
        heading = round(self.current_yaw / (math.pi / 2)) % 4

        lateral_offset = 0.0
        L, R = self.ir_left, self.ir_right
        if L < self.side_wall_thr and R < self.side_wall_thr:
            lateral_offset = (R - L) / 2.0
        elif L < self.side_wall_thr:
            lateral_offset = self.target_side - L
        elif R < self.side_wall_thr:
            lateral_offset = R - self.target_side

        frontal_offset = 0.0
        if self.ir_front < self.front_wall_thr:
            frontal_offset = self.ir_front - self.front_target

        if heading == 1:
            corrected_y = ideal_y - frontal_offset
            corrected_x = ideal_x + lateral_offset
        elif heading == 3:
            corrected_y = ideal_y + frontal_offset
            corrected_x = ideal_x - lateral_offset
        elif heading == 0:
            corrected_x = ideal_x - frontal_offset
            corrected_y = ideal_y - lateral_offset
        elif heading == 2:
            corrected_x = ideal_x + frontal_offset
            corrected_y = ideal_y + lateral_offset

        self.start_x = corrected_x
        self.start_y = corrected_y

    def command_callback(self, msg):
        if self.state != "IDLE":
            return

        cmd = msg.data
        if cmd == "WIGGLE":
            self.prev_error_angular = 0.0
            self.state = "WIGGLE"
            self.get_logger().info("WIGGLE: Zapocinjem poravnanje!")
            return

        self.align_with_sensors()

        if cmd == "FORWARD":
            self.target_distance = 0.180
            self.target_yaw = round(self.current_yaw / (math.pi / 2)) * (math.pi / 2)
            self.prev_error_linear = 0.0
            self.state = "FORWARD"
        elif cmd == "TURN_LEFT":
            self.target_yaw = math.atan2(
                math.sin(self.current_yaw + math.pi / 2),
                math.cos(self.current_yaw + math.pi / 2),
            )
            self.prev_error_angular = 0.0
            self.state = "TURN"
        elif cmd == "TURN_RIGHT":
            self.target_yaw = math.atan2(
                math.sin(self.current_yaw - math.pi / 2),
                math.cos(self.current_yaw - math.pi / 2),
            )
            self.prev_error_angular = 0.0
            self.state = "TURN"

    def publish_status(self):
        msg = String()
        msg.data = self.state
        self.status_pub.publish(msg)

    def centering_correction(self):
        L, R = self.ir_left, self.ir_right
        thr = self.side_wall_thr

        if L < thr and R < thr:
            raw_err = L - R
        elif L < thr:
            raw_err = L - self.target_side
        elif R < thr:
            raw_err = self.target_side - R
        else:
            self.lat_error_filtered = 0.0
            self.prev_lat_error = 0.0
            err_head = math.atan2(
                math.sin(self.target_yaw - self.current_yaw),
                math.cos(self.target_yaw - self.current_yaw),
            )
            return self.kp_heading * err_head

        # niskopropusni filter: sirovi IR sum je glavni uzrok krivudanja
        a = self.lat_filter_alpha
        self.lat_error_filtered = a * raw_err + (1 - a) * self.lat_error_filtered

        if abs(self.lat_error_filtered) < self.center_deadband:
            self.prev_lat_error = self.lat_error_filtered
            return 0.0

        d_err = self.lat_error_filtered - self.prev_lat_error
        self.prev_lat_error = self.lat_error_filtered
        return (self.kp_center * self.lat_error_filtered) + (self.kd_center * d_err)

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

            front_wall = self.ir_front < self.front_wall_thr
            if front_wall:
                err_lin = self.ir_front - self.front_target

            if front_wall != self.prev_front_wall:
                # izbjegni skok u D-clanu pri prijelazu izmedju odometrije i IR mjerenja
                self.prev_error_linear = err_lin
            self.prev_front_wall = front_wall

            d_err_lin = err_lin - self.prev_error_linear
            self.prev_error_linear = err_lin

            stop = err_lin <= (0.0 if front_wall else 0.005)
            if stop:
                self.state = "IDLE"
                self.cmd_pub.publish(Twist())
                return

            bias = 0.02 if front_wall else 0.0
            v = (self.kp_linear * (err_lin + bias)) + (self.kd_linear * d_err_lin)
            # aktivno kocenje: dopusti mali "unatrag" pogon (prije se v rezao na 0,
            # pa je robot kod preleta mete samo koeljao umjesto da se aktivno vrati)
            cmd.linear.x = max(min(v, 0.15), self.brake_min_v)

            corr = self.centering_correction()
            cmd.angular.z = max(min(corr, 1.0), -1.0)

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
            rotational_error = 0.0

            # Savršeni matematički omjer kad je robot apsolutno paralelan sa zidom
            K = 0.5457

            # Zastavice koje nam govore s kojim zidovima sigurno raspolažemo
            has_front = (
                self.ir_front < 0.05
                and self.ir_left_diag < 0.18
                and self.ir_right_diag < 0.18
            )
            has_left = (
                self.ir_left < self.side_wall_thr and 0.08 < self.ir_left_diag < 0.12
            )
            has_right = (
                self.ir_right < self.side_wall_thr and 0.08 < self.ir_right_diag < 0.12
            )

            # 1. Poravnanje po PREDNJEM zidu (ako ga ima, najpouzdaniji je)
            if has_front:
                rotational_error = self.ir_right_diag - self.ir_left_diag

            # 2. Poravnanje ako ima OBJE BOČNE strane
            elif has_left and has_right:
                err_L = (self.ir_left_diag * K) - self.ir_left
                err_R = self.ir_right - (self.ir_right_diag * K)
                rotational_error = (err_L + err_R) / 2.0

            # 3. Poravnanje po SAMO LIJEVOM zidu
            elif has_left:
                rotational_error = (self.ir_left_diag * K) - self.ir_left

            # 4. Poravnanje po SAMO DESNOM zidu
            elif has_right:
                rotational_error = self.ir_right - (self.ir_right_diag * K)

            else:
                self.get_logger().info(
                    "WIGGLE nemoguć (nema referentnih zidova). Prekidam."
                )
                self.state = "IDLE"
                self.cmd_pub.publish(Twist())
                return

            d_err_rot = rotational_error - self.prev_error_angular
            self.prev_error_angular = rotational_error

            if abs(rotational_error) < 0.002:
                self.state = "IDLE"
                self.cmd_pub.publish(Twist())
                self.get_logger().info("WIGGLE: Završeno! Robot je savršeno poravnat.")

                snapped_yaw = round(self.current_yaw / (math.pi / 2)) * (math.pi / 2)
                self.current_yaw = snapped_yaw
                return
            else:
                w = (3.0 * rotational_error) + (1.5 * d_err_rot)
                cmd.angular.z = max(min(w, 0.4), -0.4)

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MotionController())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
