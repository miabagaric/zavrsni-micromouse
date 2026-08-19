#!/usr/bin/env python3
"""A* ortogonalni (4 smjera) s kaznom za zavoje. Stanje = (x, y, heading)."""

import heapq
from micromouse_common.maze_map import N, E, S, W, WALL, DELTA


class AStar:
    def __init__(self, size=16, turn_cost=2.0):
        self.size = size
        self.goal_cells = [(7, 7), (7, 8), (8, 7), (8, 8)]
        self.turn_cost = turn_cost

    def set_goal(self, goal_cells):
        self.goal_cells = goal_cells

    def _passable(self, walls, x, y, direction):
        if walls[x][y][direction] == WALL:
            return False
        dx, dy = DELTA[direction]
        nx, ny = x + dx, y + dy
        return 0 <= nx < self.size and 0 <= ny < self.size

    def _heuristic(self, x, y):
        return min(abs(x - gx) + abs(y - gy) for gx, gy in self.goal_cells)

    def find_path(self, walls, start, start_heading):
        sx, sy = start
        open_heap = [(self._heuristic(sx, sy), 0.0, sx, sy, start_heading)]
        best_g = {(sx, sy, start_heading): 0.0}
        came_from = {}
        expanded = 0
        while open_heap:
            f, g, x, y, h = heapq.heappop(open_heap)
            expanded += 1
            if (x, y) in self.goal_cells:
                return self._reconstruct(came_from, (x, y, h), start, expanded)
            if g > best_g.get((x, y, h), float("inf")):
                continue
            for nd in (N, E, S, W):
                if not self._passable(walls, x, y, nd):
                    continue
                dx, dy = DELTA[nd]
                nx, ny = x + dx, y + dy
                step = 1.0
                if nd != h:
                    step += self.turn_cost
                ng = g + step
                nstate = (nx, ny, nd)
                if ng < best_g.get(nstate, float("inf")):
                    best_g[nstate] = ng
                    came_from[nstate] = (x, y, h)
                    nf = ng + self._heuristic(nx, ny)
                    heapq.heappush(open_heap, (nf, ng, nx, ny, nd))
        return None, float("inf"), 0, expanded

    def _reconstruct(self, came_from, goal_state, start, expanded):
        states = [goal_state]
        while states[-1] in came_from:
            states.append(came_from[states[-1]])
        states.reverse()
        path = [(s[0], s[1]) for s in states]
        turns = sum(
            1 for i in range(1, len(states)) if states[i][2] != states[i - 1][2]
        )
        moves = len(path) - 1
        cost = moves + self.turn_cost * turns
        return path, cost, turns, expanded
