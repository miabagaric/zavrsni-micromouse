#!/usr/bin/env python3
"""Wall follower (modificirano pravilo lijeve ruke) — BASELINE."""

from micromouse_common.maze_map import (
    N,
    E,
    S,
    W,
    WALL,
    FREE,
    DELTA,
    LEFT_OF,
    RIGHT_OF,
    OPPOSITE,
)


class WallFollower:
    def __init__(self, size=16):
        self.size = size
        self.goal_cells = [(7, 7), (7, 8), (8, 7), (8, 8)]
        self.seen = set()
        self.last_state = None
        self.max_steps = 4 * size * size
        self.steps = 0
        self.go_forward = False
        self.finished = False
        self.reached_goal = False
        self.gave_up = False

    def set_goal(self, goal_cells):
        self.goal_cells = goal_cells

    def _passable(self, walls, cx, cy, direction):
        # prolazno = DOKAZANO slobodno (FREE). UNKNOWN NE racunamo (inace vrtenje).
        if walls[cx][cy][direction] != FREE:
            return False
        dx, dy = DELTA[direction]
        nx, ny = cx + dx, cy + dy
        return 0 <= nx < self.size and 0 <= ny < self.size

    def get_best_move(self, cx, cy, heading, walls):
        if self.finished:
            return None
        state = (cx, cy, heading)
        if state != self.last_state:
            if (cx, cy) in self.goal_cells:
                self.reached_goal = True
                self.finished = True
                return None
            if state in self.seen:
                self.gave_up = True
                self.finished = True
                return None
            self.seen.add(state)
            self.last_state = state
            self.steps += 1
            if self.steps > self.max_steps:
                self.gave_up = True
                self.finished = True
                return None

        # ako smo upravo skrenuli: idi ravno AKO je naprijed slobodno
        if self.go_forward:
            self.go_forward = False
            if self._passable(walls, cx, cy, heading):
                return heading

        # pravilo LIJEVE ruke: lijevo -> naprijed -> desno -> natrag
        for direction in (
            LEFT_OF[heading],
            heading,
            RIGHT_OF[heading],
            OPPOSITE[heading],
        ):
            if self._passable(walls, cx, cy, direction):
                if direction == LEFT_OF[heading] or direction == RIGHT_OF[heading]:
                    self.go_forward = True
                return direction

        self.gave_up = True
        self.finished = True
        return None
