from micromouse_common.maze_map import MazeMap, WALL, FREE, N, E, S, W, yaw_to_heading
import math

m = MazeMap(16)

# robot u celiji (0,0), gleda sjever (yaw=90°=pi/2)
# simuliraj: prednji vidi zid (0.06), lijevi vidi zid (0.05), desni slobodno (inf)
ir = {
    "ir_front": 0.06,
    "ir_left": 0.05,
    "ir_right": float("inf"),
    "ir_left_diag": float("inf"),
    "ir_right_diag": float("inf"),
}
m.update_from_sensors(0.0, 0.0, math.pi / 2, ir)

print("heading (yaw=90):", yaw_to_heading(math.pi / 2), "(ocekivano", N, "= N)")
print(
    "sjever od (0,0):", m.get_wall(0, 0, N), "(prednji vidi zid -> ocekivano", WALL, ")"
)
print(
    "zapad od (0,0):", m.get_wall(0, 0, W), "(lijevi vidi zid -> ocekivano", WALL, ")"
)
print("istok od (0,0):", m.get_wall(0, 0, E), "(desni slobodno -> ocekivano", FREE, ")")
