#!/usr/bin/env python3
"""Dijagonalni solver po Harrisonovom modelu.
Stanje = (x, y, dir8). Cijena po TIPU poteza (ne po celiji), u vremenu.
Dijkstra ekspanzija -> dijagonala je punopravni potez u pretrazi, pa se
bira sama kad je vremenski jeftinija (rjesava L-shape problem)."""

import heapq
from micromouse_common.maze_map import WALL

# 8 smjerova
N, NE, E, SE, S, SW, W, NW = range(8)
DIAG = {NE, SE, SW, NW}
ORTHO = {N, E, S, W}

# jedinicni pomak po smjeru (dijagonalni pomjeraju obje osi)
STEP = {
    N: (0, 1),
    NE: (1, 1),
    E: (1, 0),
    SE: (1, -1),
    S: (0, -1),
    SW: (-1, -1),
    W: (-1, 0),
    NW: (-1, 1),
}

# ortogonalni zid-smjerovi za provjeru (indeksi u walls[x][y] su N=0,E=1,S=2,W=3)
WN, WE, WS, WW = 0, 1, 2, 3

# --- cijene poteza (placeholderi; zamijeniti mjerenjem) ---
T_STRAIGHT = 1.0  # ravni korak (ortogonalno, drzi brzinu)
T_DIAG = 0.71  # dijagonalni korak (~1/sqrt2 po duljini; glatko)
COST_SD45 = 0.7  # ulaz ravno->dijagonala
COST_DS45 = 0.7  # izlaz dijagonala->ravno
COST_DD90 = 0.5  # dijagonalni skretaj (dijag->dijag, 90 u dir8)
COST_SS90 = 1.5  # ravni glatki 90 zavoj (bez potpunog stajanja)
COST_IP180 = 2.5  # okret nazad


def _ortho_free(walls, x, y, wall_dir):
    """je li ortogonalni prijelaz slobodan (nije WALL)."""
    return walls[x][y][wall_dir] != WALL


def _in(x, y, size):
    return 0 <= x < size and 0 <= y < size


def _diag_passable(walls, x, y, ddir, size):
    """moze li se iz (x,y) kliziti u dijagonalu ddir (mora postojati stepenica)."""
    dx, dy = STEP[ddir]
    nx, ny = x + dx, y + dy
    if not _in(nx, ny, size):
        return False
    # dva 'L' puta oko kuta; barem jedan mora biti otvoren za prolaz,
    # ali za sigurnu stepenicu trazimo da postoji put koji ne rezi zid.
    if ddir == NE:
        p1 = _ortho_free(walls, x, y, WN) and _ortho_free(walls, x, y + 1, WE)
        p2 = _ortho_free(walls, x, y, WE) and _ortho_free(walls, x + 1, y, WN)
    elif ddir == NW:
        p1 = _ortho_free(walls, x, y, WN) and _ortho_free(walls, x, y + 1, WW)
        p2 = _ortho_free(walls, x, y, WW) and _ortho_free(walls, x - 1, y, WN)
    elif ddir == SE:
        p1 = _ortho_free(walls, x, y, WS) and _ortho_free(walls, x, y - 1, WE)
        p2 = _ortho_free(walls, x, y, WE) and _ortho_free(walls, x + 1, y, WS)
    else:  # SW
        p1 = _ortho_free(walls, x, y, WS) and _ortho_free(walls, x, y - 1, WW)
        p2 = _ortho_free(walls, x, y, WW) and _ortho_free(walls, x - 1, y, WS)
    return p1 or p2


def _ortho_dir_to_wall(d):
    return {N: WN, E: WE, S: WS, W: WW}[d]


def _turn_cost(from_dir, to_dir):
    """cijena promjene smjera u dir8 prostoru, po tipu."""
    if from_dir == to_dir:
        return 0.0
    diff = (to_dir - from_dir) % 8
    from_ortho = from_dir in ORTHO
    to_ortho = to_dir in ORTHO
    if from_ortho and not to_ortho:
        return COST_SD45  # ravni -> dijagonala (45)
    if not from_ortho and to_ortho:
        return COST_DS45  # dijagonala -> ravni (45)
    if not from_ortho and not to_ortho:
        return COST_DD90  # dijag -> dijag (90 u dir8)
    # ortho -> ortho
    if diff == 4:
        return COST_IP180
    return COST_SS90  # 90 ravni zavoj


def _step_cost(to_dir):
    return T_DIAG if to_dir in DIAG else T_STRAIGHT


def _neighbors(walls, state, size):
    """legalni prijelazi iz stanja -> lista (novo_stanje, cijena)."""
    x, y, d = state
    out = []
    for nd in range(8):
        # dozvoli samo: nastavak, +-45 (ortho<->diag ili diag skret), 180
        diff = (nd - d) % 8
        if diff not in (0, 1, 2, 6, 7, 4):
            continue
        dx, dy = STEP[nd]
        nx, ny = x + dx, y + dy
        if not _in(nx, ny, size):
            continue
        # prohodnost prema tipu ODREDISNOG smjera
        if nd in ORTHO:
            if not _ortho_free(walls, x, y, _ortho_dir_to_wall(nd)):
                continue
        else:
            if not _diag_passable(walls, x, y, nd, size):
                continue
        cost = _turn_cost(d, nd) + _step_cost(nd)
        out.append(((nx, ny, nd), cost))
    return out


class DiagonalSolver:
    def __init__(self, size=16, start_heading=N):
        self.size = size
        self.goal_cells = [(7, 7), (7, 8), (8, 7), (8, 8)]
        self.start_heading = start_heading

    def set_goal(self, goal_cells):
        self.goal_cells = goal_cells

    def find_path(self, walls, start, start_heading=None):
        sh = self.start_heading if start_heading is None else start_heading
        sx, sy = start
        start_state = (sx, sy, sh)
        dist = {start_state: 0.0}
        came = {}
        heap = [(0.0, sx, sy, sh)]
        expanded = 0
        goal_state = None
        while heap:
            g, x, y, d = heapq.heappop(heap)
            expanded += 1
            if g > dist.get((x, y, d), float("inf")):
                continue
            if (x, y) in self.goal_cells:
                goal_state = (x, y, d)
                break
            for nstate, c in _neighbors(walls, (x, y, d), self.size):
                ng = g + c
                if ng < dist.get(nstate, float("inf")):
                    dist[nstate] = ng
                    came[nstate] = (x, y, d)
                    heapq.heappush(heap, (ng, nstate[0], nstate[1], nstate[2]))
        if goal_state is None:
            return None, float("inf"), 0, expanded
        # rekonstrukcija
        states = [goal_state]
        while states[-1] in came:
            states.append(came[states[-1]])
        states.reverse()
        path = [(s[0], s[1]) for s in states]
        cost = dist[goal_state]
        n_diag = sum(1 for s in states if s[2] in DIAG)
        return path, cost, n_diag, expanded
