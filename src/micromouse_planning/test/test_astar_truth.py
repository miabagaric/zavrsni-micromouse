#!/usr/bin/env python3
"""Offline test A* na maze_truth.json (poznata puna mapa)."""

import json
from micromouse_common.maze_map import N, E, S, W, WALL, FREE
from micromouse_planning.a_star import AStar
from micromouse_planning.dijkstra import Dijkstra

TRUTH = "/home/mia/micromouse_ws/src/micromouse_bringup/scripts/maze_truth.json"


def load_walls(path):
    d = json.load(open(path))
    size = d["N"]
    east, north = d["east"], d["north"]
    walls = [[[FREE, FREE, FREE, FREE] for _ in range(size)] for _ in range(size)]
    for x in range(size):
        for y in range(size):
            walls[x][y][E] = WALL if east[x][y] else FREE
            walls[x][y][N] = WALL if north[x][y] else FREE
            walls[x][y][W] = (
                WALL if (x > 0 and east[x - 1][y]) else (WALL if x == 0 else FREE)
            )
            walls[x][y][S] = (
                WALL if (y > 0 and north[x][y - 1]) else (WALL if y == 0 else FREE)
            )
    return walls, size, tuple(d["start"]), [tuple(g) for g in d["goal"]]


def main():
    walls, size, start, goal = load_walls(TRUTH)
    astar = AStar(size=size)
    astar.set_goal(goal)
    path, cost, turns, expanded = astar.find_path(walls, start, N)  # start gleda sjever
    if path is None:
        print("NEMA PUTA — nesto je krivo")
        return
    print(f"path duljina (celije): {len(path)}")
    print(f"moves: {len(path) - 1}, turns: {turns}, cost: {cost}")
    print(f"start: {path[0]}  ->  goal: {path[-1]}")
    print(f"put: {path}")
    # provjere zdravog razuma:
    assert path[0] == start, "put ne krece iz starta"
    assert path[-1] in goal, "put ne zavrsava u cilju"
    for i in range(1, len(path)):
        (ax, ay), (bx, by) = path[i - 1], path[i]
        assert abs(ax - bx) + abs(ay - by) == 1, (
            f"nesusjedni skok {path[i - 1]}->{path[i]}"
        )
    print("SVE PROVJERE OK")

    # optimalnost: turn_cost=0 mora dati istu DULJINU kao cisti najkraci put
    a0 = AStar(size=size, turn_cost=0.0)
    a0.set_goal(goal)
    p0, c0, t0, _ = a0.find_path(walls, start, N)
    print(f"\nturn_cost=0 -> moves: {len(p0) - 1}, turns: {t0}, cost: {c0}")
    print(f"turn_cost=2 -> moves: {len(path) - 1}, turns: {turns}")
    print(
        f"kazna za zavoje smanjila zavoje: {t0} -> {turns}"
        if turns <= t0
        else f"UPOZORENJE: turn_cost povecao zavoje ({t0} -> {turns})?!"
    )

    print("\n--- A* vs Dijkstra (isti put, razlicit broj prosirenih cvorova) ---")
    for name, cls in (("A*", AStar), ("Dijkstra", Dijkstra)):
        alg = cls(size=size, turn_cost=2.0)
        alg.set_goal(goal)
        p, c, t, exp = alg.find_path(walls, start, N)
        print(f"{name:>9}: moves={len(p) - 1} turns={t} cost={c} prosireno={exp}")


if __name__ == "__main__":
    main()
