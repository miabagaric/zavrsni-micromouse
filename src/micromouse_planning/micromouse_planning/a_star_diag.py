#!/usr/bin/env python3
"""Klasicni 8-smjerni A* (udzbenicki). Dijagonala = punopravni potez cijene
sqrt(2), ortogonala = 1. Oktilna heuristika (dopustiva s dijagonalama).
Preferira dijagonale jer su jeftinije po duljini. Za usporedbu s
ortogonalnim A*/Dijkstra."""

import heapq
import math
from micromouse_common.maze_map import N, E, S, W, WALL, DELTA

SQRT2 = math.sqrt(2)
# 8 smjerova kao (dx, dy)
DIRS8 = [
    (0, 1),
    (1, 0),
    (0, -1),
    (-1, 0),  # N E S W
    (1, 1),
    (1, -1),
    (-1, -1),
    (-1, 1),
]  # NE SE SW NW


class AStarDiag:
    def __init__(self, size=16):
        self.size = size
        self.goal_cells = [(7, 7), (7, 8), (8, 7), (8, 8)]

    def set_goal(self, goal_cells):
        self.goal_cells = goal_cells

    def _in(self, x, y):
        return 0 <= x < self.size and 0 <= y < self.size

    def _ortho_free(self, walls, x, y, d):
        """je li ortogonalni brid iz (x,y) u smjeru d slobodan (nije WALL)."""
        return walls[x][y][d] != WALL

    def _passable(self, walls, x, y, dx, dy):
        """moze li se iz (x,y) u (x+dx, y+dy)."""
        nx, ny = x + dx, y + dy
        if not self._in(nx, ny):
            return False
        if dx == 0 or dy == 0:
            # ortogonalno: nadji smjer i provjeri brid
            d = {(0, 1): N, (1, 0): E, (0, -1): S, (-1, 0): W}[(dx, dy)]
            return self._ortho_free(walls, x, y, d)
        # dijagonalno: oba susjedna ortogonalna brida moraju biti slobodna
        # (robot ne rezi kut kroz zid) -> "L" preko obje medjucelije
        dh = E if dx > 0 else W  # horizontalni smjer
        dv = N if dy > 0 else S  # vertikalni smjer
        # put 1: horizontalno pa vertikalno; put 2: vertikalno pa horizontalno
        l1 = self._ortho_free(walls, x, y, dh) and self._ortho_free(walls, nx, y, dv)
        l2 = self._ortho_free(walls, x, y, dv) and self._ortho_free(walls, x, ny, dh)
        return l1 or l2

    def _heuristic(self, x, y):
        """oktilna udaljenost do najblizeg cilja (dopustiva s dijagonalama)."""
        best = float("inf")
        for gx, gy in self.goal_cells:
            dx, dy = abs(x - gx), abs(y - gy)
            h = (dx + dy) + (SQRT2 - 2) * min(dx, dy)
            best = min(best, h)
        return best

    def find_path(self, walls, start, start_heading=None):
        sx, sy = start
        open_heap = [(self._heuristic(sx, sy), 0.0, sx, sy)]
        best_g = {(sx, sy): 0.0}
        came_from = {}
        expanded = 0
        while open_heap:
            f, g, x, y = heapq.heappop(open_heap)
            expanded += 1
            if (x, y) in self.goal_cells:
                return self._reconstruct(came_from, (x, y), start, expanded)
            if g > best_g.get((x, y), float("inf")):
                continue
            for dx, dy in DIRS8:
                if not self._passable(walls, x, y, dx, dy):
                    continue
                nx, ny = x + dx, y + dy
                step = SQRT2 if (dx != 0 and dy != 0) else 1.0
                ng = g + step
                if ng < best_g.get((nx, ny), float("inf")):
                    best_g[(nx, ny)] = ng
                    came_from[(nx, ny)] = (x, y)
                    nf = ng + self._heuristic(nx, ny)
                    heapq.heappush(open_heap, (nf, ng, nx, ny))
        return None, float("inf"), 0, expanded

    def _reconstruct(self, came_from, goal, start, expanded):
        path = [goal]
        while path[-1] in came_from:
            path.append(came_from[path[-1]])
        path.reverse()
        cost = 0.0
        diag = 0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i - 1][0]
            dy = path[i][1] - path[i - 1][1]
            if dx != 0 and dy != 0:
                cost += SQRT2
                diag += 1
            else:
                cost += 1.0
        return path, cost, diag, expanded

    def expand(self, path, walls):
        """Sazeti put (sa dijagonalnim skokovima) -> puni celija-po-celija put,
        umecuci veznu celiju za svaki dijagonalni korak. Vraca (full, is_diag)."""
        full = [path[0]]
        is_diag = [False]
        for i in range(1, len(path)):
            px, py = path[i - 1]
            cx, cy = path[i]
            dx, dy = cx - px, cy - py
            if dx != 0 and dy != 0:
                # dijagonala: umetni veznu celiju koja je prohodna
                dh = E if dx > 0 else W
                dv = N if dy > 0 else S
                # opcija A: (px,cy) preko vertikalnog pa horizontalnog
                a_ok = self._ortho_free(walls, px, py, dv) and self._ortho_free(
                    walls, px, cy, dh
                )
                mid = (px, cy) if a_ok else (cx, py)
                full.append(mid)
                is_diag.append(True)
                full.append((cx, cy))
                is_diag.append(True)
            else:
                full.append((cx, cy))
                is_diag.append(False)
        return full, is_diag
