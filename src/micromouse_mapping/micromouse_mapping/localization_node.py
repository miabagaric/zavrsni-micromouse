#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


# smjerovi: N, E, S, W = 0, 1, 2, 3  (konvencija projekta)
N, E, S, W = 0, 1, 2, 3

# pomak ćelije po smjeru: +x=istok, +y=sjever
DELTA = {N: (0, 1), E: (1, 0), S: (0, -1), W: (-1, 0)}

# rotacija smjera pri okretu
LEFT_OF = {N: W, W: S, S: E, E: N}
RIGHT_OF = {N: E, E: S, S: W, W: N}

# smjer -> yaw u 'map' okviru: N=+y=+90°, E=+x=0°, S=-90°, W=180°
HEADING_YAW = {N: math.pi / 2, E: 0.0, S: -math.pi / 2, W: math.pi}

CELL = 0.18


class LocalizationNode(Node):
    """Diskretna lokalizacija: prati (cx, cy, heading) brojeci izvrsene komande.
    NE cita odometriju — metri zive samo u motion controlleru. Objavljuje
    savrsenu pozu (centar celije + kardinalni kut) na /robot_pose."""

    def __init__(self):
        super().__init__("localization_node")

        # diskretno stanje — izvor istine o tome gdje je robot
        self.cx = 0
        self.cy = 0
        self.heading = N

        # pracenje poteza: koju komandu izvrsavamo i jesmo li ju vec obradili
        self.pending_cmd = None  # komanda koju motion trenutno izvrsava
        self.prev_status = "IDLE"

        self.create_subscription(String, "/maze_commands", self.command_cb, 10)
        self.create_subscription(String, "/robot_status", self.status_cb, 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/robot_pose", 10)

        # objavi pocetno stanje odmah (0,0,N) da svi znaju gdje krecemo
        self.create_timer(0.1, self.publish_pose)

    def command_cb(self, msg):
        # zapamti komandu koja je upravo poslana motionu.
        # (motion ju prihvaca samo kad je IDLE, pa je ovo komanda koja krece.)
        if msg.data in ("FORWARD", "TURN_LEFT", "TURN_RIGHT"):
            self.pending_cmd = msg.data

    def status_cb(self, msg):
        status = msg.data

        # okidac: motion je ZAVRSIO potez = presao iz gibanja natrag u IDLE
        finished = status == "IDLE" and self.prev_status != "IDLE"
        self.prev_status = status

        if finished and self.pending_cmd is not None:
            self.apply(self.pending_cmd)
            self.pending_cmd = None

    def apply(self, cmd):
        """Azuriraj diskretno stanje na temelju izvrsene komande."""
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
        # diskretno stanje -> savrsena metricka poza (centar celije + kardinalni kut)
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
