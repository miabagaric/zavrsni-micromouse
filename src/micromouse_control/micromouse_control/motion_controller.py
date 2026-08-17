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

        # senzori (samo front nam treba za Blok A, ostale učitavamo za kasnije)
        self.ir_front = float("inf")
        self.create_subscription(
            LaserScan, "/ir_front", lambda m: self._ir("ir_front", m), 10
        )

        # --- stanje odometrije (SIROVA, relativna — NE treba nam svjetski okvir) ---
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.have_odom = False

        # --- parametri Bloka A ---
        self.CELL = 0.180  # duljina ćelije [m]
        self.FRONT_TARGET = 0.0078  # izmjereno: ir_front kad je base_footprint u centru
        self.FRONT_WALL_THR = 0.12  # ispod ovoga smatramo da zid POSTOJI ispred
        self.V_MAX = 0.15  # gornji limit brzine [m/s]
        self.V_BRAKE = -0.06  # dopušteni pogon unatrag (aktivno kočenje kod preleta)
        self.STOP_ODOM = 0.003  # tolerancija zaustavljanja kad NEMA zida [m]
        self.STOP_WALL = 0.0005  # tolerancija kad se sidrimo na zid [m]

        self.kp_lin = 3.0
        self.kd_lin = 0.35

        # --- FSM ---
        self.state = "IDLE"
        self.anchor_x = 0.0  # odom pozicija na početku poteza
        self.anchor_y = 0.0
        self.prev_err = 0.0
        self.prev_front_wall = False

        self.create_timer(0.02, self.control_loop)  # 50 Hz kontrola
        self.create_timer(0.10, self.publish_status)  # 10 Hz status

    def _ir(self, name, msg):
        setattr(self, name, msg.ranges[0] if msg.ranges else float("inf"))

    def odom_cb(self, msg):
        # čitamo SAMO poziciju, i to relativno. Yaw nas u Bloku A ne zanima.
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.have_odom = True

    def publish_status(self):
        m = String()
        m.data = self.state
        self.status_pub.publish(m)

    def command_cb(self, msg):
        if self.state != "IDLE":
            return  # handshake: primamo naredbu samo kad smo slobodni
        if msg.data == "FORWARD":
            if not self.have_odom:
                return
            # sidro = TRENUTNA odom pozicija. Sve mjerimo relativno od nje,
            # pa drift između poteza ne ulazi u ovaj potez.
            self.anchor_x = self.odom_x
            self.anchor_y = self.odom_y
            self.prev_err = 0.0
            self.prev_front_wall = self.ir_front < self.FRONT_WALL_THR
            self.state = "FORWARD"

    def control_loop(self):
        cmd = Twist()

        if self.state == "IDLE":
            self.cmd_pub.publish(cmd)
            return

        if self.state == "FORWARD":
            # koliko smo stvarno prešli od sidra (euklidski pomak u ravnini)
            travelled = math.hypot(
                self.odom_x - self.anchor_x, self.odom_y - self.anchor_y
            )

            front_wall = self.ir_front < self.FRONT_WALL_THR

            # DVA NAČINA MJERENJA GREŠKE, isti cilj (base_footprint u centru):
            if front_wall:
                # zid ispred = precizno ravnalo. Greška = koliko još do mete.
                # ir_front > FRONT_TARGET znači "još sam predaleko, vozi naprijed".
                err = self.ir_front - self.FRONT_TARGET
                stop_tol = self.STOP_WALL
            else:
                # nema zida = oslanjamo se na odometriju (18 cm od sidra).
                err = self.CELL - travelled
                stop_tol = self.STOP_ODOM

            # prijelaz odometrija<->zid: resetiraj D-član da ne skoči derivacija
            if front_wall != self.prev_front_wall:
                self.prev_err = err
            self.prev_front_wall = front_wall

            # uvjet zaustavljanja (uvijek postoji izlaz iz stanja -> robusnost)
            if err <= stop_tol:
                self.state = "IDLE"
                self.cmd_pub.publish(Twist())
                return

            # PD regulator brzine
            d_err = err - self.prev_err
            self.prev_err = err
            v = self.kp_lin * err + self.kd_lin * d_err

            # limit + aktivno kočenje unatrag ako smo preletjeli metu
            cmd.linear.x = max(min(v, self.V_MAX), self.V_BRAKE)
            # angular.z = 0 za sada (centriranje dolazi u Bloku B)

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MotionController())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
