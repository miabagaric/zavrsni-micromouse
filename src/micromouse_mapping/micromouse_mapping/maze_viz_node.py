import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class MazeVizNode(Node):
    """PRIKAZ: pretplacuje se na mapu (OccupancyGrid) i pozu robota,
    crta ciste zidove kao linije + robota kao trokut. Nema logike mapiranja."""

    def __init__(self):
        super().__init__("maze_viz_node")
        self.sub = 3
        self.cell = 0.18
        self.size = 16

        self.create_subscription(OccupancyGrid, "/maze_map", self.grid_cb, 10)
        self.create_subscription(PoseStamped, "/robot_pose", self.pose_cb, 10)

        self.wall_pub = self.create_publisher(MarkerArray, "/maze_markers", 10)
        self.robot_pub = self.create_publisher(Marker, "/robot_marker", 10)

    # ---------- zidovi kao ciste linije ----------
    def grid_cb(self, msg):
        w = msg.info.width
        data = msg.data

        def pix(px, py):
            return data[py * w + px]

        arr = MarkerArray()
        mid = self.sub // 2
        mid_i = 0
        for x in range(self.size):
            for y in range(self.size):
                bx, by = x * self.sub, y * self.sub
                edges = {
                    "N": (bx + mid, by + self.sub - 1),
                    "S": (bx + mid, by + 0),
                    "E": (bx + self.sub - 1, by + mid),
                    "W": (bx + 0, by + mid),
                }
                for d, (px, py) in edges.items():
                    if pix(px, py) != 100:
                        continue
                    arr.markers.append(self.wall_marker(x, y, d, mid_i))
                    mid_i += 1
        # ako nema zidova, posalji brisac da se stari ocisti
        if not arr.markers:
            clear = Marker()
            clear.header.frame_id = "map"
            clear.action = Marker.DELETEALL
            arr.markers.append(clear)
        self.wall_pub.publish(arr)

    def wall_marker(self, x, y, d, mid_i):
        wx, wy = x * self.cell, y * self.cell
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "walls"
        m.id = mid_i
        m.type = Marker.CUBE
        m.action = Marker.ADD
        thick, length, height = 0.012, self.cell, 0.05
        if d == "N":
            m.pose.position.x, m.pose.position.y = wx, wy + self.cell / 2
            m.scale.x, m.scale.y = length, thick
        elif d == "S":
            m.pose.position.x, m.pose.position.y = wx, wy - self.cell / 2
            m.scale.x, m.scale.y = length, thick
        elif d == "E":
            m.pose.position.x, m.pose.position.y = wx + self.cell / 2, wy
            m.scale.x, m.scale.y = thick, length
        else:  # W
            m.pose.position.x, m.pose.position.y = wx - self.cell / 2, wy
            m.scale.x, m.scale.y = thick, length
        m.pose.position.z = height / 2
        m.scale.z = height
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.4, 1.0, 1.0
        return m

    # ---------- robot kao trokut ----------
    def pose_cb(self, msg):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "robot"
        m.id = 0
        m.type = Marker.TRIANGLE_LIST
        m.action = Marker.ADD
        # trokut pokazuje +x (naprijed); orijentaciju preuzima iz poze
        m.pose = msg.pose
        m.pose.position.z = 0.03
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.points = [
            Point(x=0.09, y=0.0, z=0.0),
            Point(x=-0.05, y=0.05, z=0.0),
            Point(x=-0.05, y=-0.05, z=0.0),
        ]
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.1, 0.1, 1.0
        self.robot_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = MazeVizNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
