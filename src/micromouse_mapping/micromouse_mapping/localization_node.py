#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


# --- JEDAN izvor istine za yaw<->kvaternion (rotacija samo oko z) ---
def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def yaw_to_quat_zw(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# kardinalni smjer -> yaw u 'map' okviru. N=+y=+90°, E=+x=0, S=-90°, W=180°.
HEADING_YAW = {0: math.pi / 2, 1: 0.0, 2: -math.pi / 2, 3: math.pi}


class LocalizationNode(Node):
    """Jedini izvor poze. Cita sirovu /odom, kalibrira u 'map' okvir,
    i korigira SAMO POZICIJU sidrenjem na prednji zid. YAW se NE dira -
    odometrijski kut je dokazano tocan. Neovisno o planeru."""

    def __init__(self):
        super().__init__("localization_node")

        # geometrija
        self.CELL = 0.18
        self.WALL_FACE = 0.084  # centar celije -> lice zida (0.09 - 0.006)
        self.SENSOR_X = 0.076  # ir_front pozicija na x od base_footprinta
        self.ANCHOR_THR = 0.05  # ir_front < ovo => stvarno smo uz prednji zid
        self.ALIGN_TOL = math.radians(8)  # koliko blizu kardinala da korigiramo
        self.CENTER_TOL = (
            0.05  # koliko blizu centra celije (po osi gledanja!) da korigiramo
        )
        self.MAX_CORR = 0.05  # veci skok = kriva celija -> odbaci

        # jednokratna kalibracija
        self.yaw_offset = None
        self.start_ox = 0.0
        self.start_oy = 0.0

        # kalibrirana (jos driftajuca) odom poza u 'map'
        self.cal_x = 0.0
        self.cal_y = 0.0
        self.cal_yaw = math.pi / 2

        # akumulirani ISPRAVCI POZICIJE (ne yaw!). izlaz = cal + corr.
        self.corr_x = 0.0
        self.corr_y = 0.0

        self.ir_front = float("inf")
        self.prev_status = "IDLE"

        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(LaserScan, "/ir_front", self._ir, 10)
        self.create_subscription(String, "/robot_status", self.status_cb, 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/robot_pose", 10)

    def _ir(self, msg):
        self.ir_front = msg.ranges[0] if msg.ranges else float("inf")

    def odom_cb(self, msg):
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        oyaw = quat_to_yaw(msg.pose.pose.orientation)

        if self.yaw_offset is None:
            # spawn: robot gleda sjever (+y=+90°), odom yaw je ~0 -> offset=+90°
            self.yaw_offset = math.pi / 2 - oyaw
            self.start_ox = ox
            self.start_oy = oy

        c, s = math.cos(self.yaw_offset), math.sin(self.yaw_offset)
        dx, dy = ox - self.start_ox, oy - self.start_oy
        self.cal_x = c * dx - s * dy
        self.cal_y = s * dx + c * dy
        self.cal_yaw = wrap(oyaw + self.yaw_offset)

        self.publish_pose()

    def out_pose(self):
        # POZICIJA korigirana, YAW netaknut (odometrija je tocna)
        return self.cal_x + self.corr_x, self.cal_y + self.corr_y, self.cal_yaw

    def status_cb(self, msg):
        if msg.data == "IDLE" and self.prev_status != "IDLE":
            self.correct()
        self.prev_status = msg.data

    def correct(self):
        px, py, pyaw = self.out_pose()

        # 1) moramo biti poravnati s kardinalom (inace ne znamo koji zid gledamo)
        best_h, best_e = 0, math.pi
        for h, cy_yaw in HEADING_YAW.items():
            e = wrap(pyaw - cy_yaw)
            if abs(e) < abs(best_e):
                best_h, best_e = h, e
        if abs(best_e) > self.ALIGN_TOL:
            self.get_logger().info(
                f"preskacem korekciju: nije poravnat ({math.degrees(best_e):.1f}°)"
            )
            return
        heading = best_h

        # 2) treba postojati prednji zid da bismo se sidrili
        if self.ir_front >= self.ANCHOR_THR:
            self.get_logger().info(
                f"preskacem korekciju: nema prednjeg zida (ir={self.ir_front:.3f})"
            )
            return

        # 3) gruba poza daje IDENTITET celije (drift << pola celije)
        cx = round(px / self.CELL)
        cy = round(py / self.CELL)
        if not (0 <= cx < 16 and 0 <= cy < 16):
            return

        # lice zida ispred = SENSOR_X + ir_front od base_footprinta
        d = self.SENSOR_X + self.ir_front

        if heading == 0:  # N -> korigira y
            true_v = (cy * self.CELL + self.WALL_FACE) - d
            axis, cal_v, old_corr = "y", self.cal_y, self.corr_y
        elif heading == 2:  # S -> y
            true_v = (cy * self.CELL - self.WALL_FACE) + d
            axis, cal_v, old_corr = "y", self.cal_y, self.corr_y
        elif heading == 1:  # E -> x
            true_v = (cx * self.CELL + self.WALL_FACE) - d
            axis, cal_v, old_corr = "x", self.cal_x, self.corr_x
        else:  # W -> x
            true_v = (cx * self.CELL - self.WALL_FACE) + d
            axis, cal_v, old_corr = "x", self.cal_x, self.corr_x

        new_corr = true_v - cal_v

        # 4) sigurnosna brana: prevelik skok = kriva celija
        if abs(new_corr - old_corr) > self.MAX_CORR:
            self.get_logger().warn(
                f"korekcija {axis} {new_corr - old_corr:+.3f} m prevelika -> kriva celija, odbacujem"
            )
            return

        if axis == "y":
            self.corr_y = new_corr
        else:
            self.corr_x = new_corr
        self.get_logger().info(
            f"SIDRENJE ({cx},{cy}) smjer {heading}: {axis} ispravak {new_corr - old_corr:+.4f} m "
            f"(ukupni corr_x={self.corr_x:+.4f}, corr_y={self.corr_y:+.4f})"
        )

    def publish_pose(self):
        px, py, pyaw = self.out_pose()
        z, w = yaw_to_quat_zw(pyaw)
        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = px
        p.pose.position.y = py
        p.pose.orientation.z = z
        p.pose.orientation.w = w
        self.pose_pub.publish(p)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LocalizationNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
