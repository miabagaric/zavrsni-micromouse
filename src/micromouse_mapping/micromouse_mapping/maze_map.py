import math

# stanja zida
UNKNOWN = 0
WALL = 1
FREE = 2

# smjerovi (indeksi u polju zidova svake celije)
N, E, S, W = 0, 1, 2, 3

# pomak (dx, dy) za svaki smjer  -- +x=istok, +y=sjever
DELTA = {N: (0, 1), E: (1, 0), S: (0, -1), W: (-1, 0)}
OPPOSITE = {N: S, E: W, S: N, W: E}

# rotacija smjera
LEFT_OF = {N: W, W: S, S: E, E: N}
RIGHT_OF = {N: E, E: S, S: W, W: N}

# tolerancija
tolerance_deg = 10
center_tolerance = 0.35  # tolerancija pozicije u celiji (postotak)


def yaw_to_heading_strict(yaw, tolerance_deg=10):
    """
    Kontinuirani kut (rad) -> najblizi od 4 smjera N/E/S/W, ali SAMO
    ako je unutar zadane tolerancije (npr. +- 10 stupnjeva).
    +y=sjever=90°, +x=istok=0°.
    """
    deg = math.degrees(yaw) % 360

    # Tolerancija oko 90 (N)
    if abs(deg - 90) <= tolerance_deg:
        return N
    # Tolerancija oko 180 (W)
    if abs(deg - 180) <= tolerance_deg:
        return W
    # Tolerancija oko 270 (S)
    if abs(deg - 270) <= tolerance_deg:
        return S
    # Tolerancija oko 0/360 (E)
    if deg <= tolerance_deg or deg >= 360 - tolerance_deg:
        return E

    return None  # Robot nije dobro poravnat


class MazeMap:
    def __init__(self, size=16):
        self.size = size
        self.walls = [
            [[UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN] for _ in range(size)]
            for _ in range(size)
        ]
        self.visited = set()  # celije u kojima je robot bio
        self.last_conflict = None  # zadnje neslaganje (cx,cy,dir,known,sensed)
        self._init_borders()

    def in_bounds(self, x, y):
        return 0 <= x < self.size and 0 <= y < self.size

    def _init_borders(self):
        n = self.size
        for i in range(n):
            self.set_wall(i, n - 1, N, WALL)
            self.set_wall(i, 0, S, WALL)
            self.set_wall(0, i, W, WALL)
            self.set_wall(n - 1, i, E, WALL)

    def set_wall(self, x, y, direction, state):
        self.walls[x][y][direction] = state
        dx, dy = DELTA[direction]
        nx, ny = x + dx, y + dy
        if self.in_bounds(nx, ny):
            self.walls[nx][ny][OPPOSITE[direction]] = state

    def get_wall(self, x, y, direction):
        return self.walls[x][y][direction]

    def world_to_cell(self, wx, wy, cell=0.18):
        return round(wx / cell), round(wy / cell)

    def update_from_sensors(
        self, robot_x, robot_y, yaw, ir_ranges, threshold=0.12, cell=0.18
    ):
        cx, cy = self.world_to_cell(robot_x, robot_y, cell)
        if not self.in_bounds(cx, cy):
            return

        # Uvijek biljezimo da je robot bio u celiji radi iscrtavanja puta
        self.visited.add((cx, cy))

        # --- 1. PROVJERA POZICIJE (nalazi li se u centru ćelije +- 20%) ---
        center_x = cx * cell
        center_y = cy * cell
        pos_tolerance = cell * center_tolerance

        if (
            abs(robot_x - center_x) > pos_tolerance
            or abs(robot_y - center_y) > pos_tolerance
        ):
            return  # Robot nije u centru, ignoriraj senzore

        heading = yaw_to_heading_strict(yaw, tolerance_deg)

        if heading is None:
            return  # Robot je ukoso, ignoriraj senzore

        # --- ZAPISIVANJE ZIDOVA (samo ako je centriran i poravnat) ---
        sensor_dirs = {
            "ir_front": heading,
            "ir_left": LEFT_OF[heading],
            "ir_right": RIGHT_OF[heading],
        }
        conflict = False
        for name, direction in sensor_dirs.items():
            r = ir_ranges[name]
            sensed = WALL if r < threshold else FREE
            known = self.get_wall(cx, cy, direction)

            if known != UNKNOWN and known != sensed and (cx != 0 and cy != 0):
                # NESLAGANJE: senzor se ne slaze s vec poznatim zidom te ivice.
                # Vjerojatno robot NIJE tamo gdje misli -> ne diraj mapu (cuvaj poznato).
                conflict = True
                self.last_conflict = (cx, cy, direction, known, sensed)
            else:
                self.set_wall(cx, cy, direction, sensed)

        return conflict

    def to_grid(self, sub=3):
        """Rešetka (sub x sub piksela/celiji): -1=nepoznato, 0=slobodno, 100=zid.
        Slobodnu sredinu crtamo SAMO za posjecene celije (bez sahovskog uzorka)."""
        n = self.size * sub
        grid = [[-1 for _ in range(n)] for _ in range(n)]
        for x in range(self.size):
            for y in range(self.size):
                if (x, y) in self.visited:
                    grid[y * sub + sub // 2][x * sub + sub // 2] = 0
                for direction in (N, E, S, W):
                    state = self.get_wall(x, y, direction)
                    if state == UNKNOWN:
                        continue
                    val = 100 if state == WALL else 0
                    if direction == N:
                        px, py = x * sub + sub // 2, y * sub + (sub - 1)
                    elif direction == S:
                        px, py = x * sub + sub // 2, y * sub + 0
                    elif direction == E:
                        px, py = x * sub + (sub - 1), y * sub + sub // 2
                    else:  # W
                        px, py = x * sub + 0, y * sub + sub // 2
                    grid[py][px] = val
        return grid
