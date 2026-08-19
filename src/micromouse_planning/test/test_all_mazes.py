#!/usr/bin/env python3
"""A* vs Dijkstra na vise labirinata: isti optimalni put, razlika u broju
prosirenih cvorova (efikasnost pretrage). Ispisuje tablicu + prosjek."""

import glob
import json
import os
from micromouse_common.maze_map import N, E, S, W, WALL, FREE
from micromouse_planning.a_star import AStar
from micromouse_planning.dijkstra import Dijkstra

MAZE_DIR = os.path.expanduser("~/micromouse_ws/src/micromouse_bringup/generated_mazes")
TURN_COST = 2.0


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


def run_one(path):
    walls, size, start, goal = load_walls(path)
    res = {}
    for name, cls in (("astar", AStar), ("dijkstra", Dijkstra)):
        alg = cls(size=size, turn_cost=TURN_COST)
        alg.set_goal(goal)
        p, c, t, exp = alg.find_path(walls, start, N)
        if p is None:
            return None  # nerjesivo
        res[name] = (len(p) - 1, t, c, exp)
    return res


def main():
    files = sorted(glob.glob(os.path.join(MAZE_DIR, "truth_s*.json")))
    if not files:
        print(f"nema fileova u {MAZE_DIR} (ocekivano truth_s*.json)")
        return

    print(
        f"{'mapa':<14}{'moves':>6}{'turns':>6}{'cost':>7}"
        f"{'A* exp':>8}{'Dij exp':>9}{'omjer':>7}"
    )
    print("-" * 57)

    a_sum, d_sum, ok, mismatch = 0, 0, 0, 0
    for path in files:
        name = os.path.basename(path).replace("truth_", "").replace(".json", "")
        try:
            res = run_one(path)
        except Exception as e:
            print(f"{name:<14} GRESKA: {e}")
            continue
        if res is None:
            print(f"{name:<14} NERJESIV (nema puta)")
            continue

        am, at, ac, aexp = res["astar"]
        dm, dt, dc, dexp = res["dijkstra"]
        # sanity: isti optimum? (put smije biti drugaciji, ali cijena MORA biti ista)
        flag = "" if ac == dc else "  <-- CIJENE SE RAZLIKUJU!"
        if ac != dc:
            mismatch += 1
        ratio = dexp / aexp if aexp else 0.0
        print(
            f"{name:<14}{am:>6}{at:>6}{ac:>7.0f}{aexp:>8}{dexp:>9}{ratio:>7.2f}{flag}"
        )
        a_sum += aexp
        d_sum += dexp
        ok += 1

    if ok:
        print("-" * 57)
        print(
            f"{'PROSJEK':<14}{'':>6}{'':>6}{'':>7}"
            f"{a_sum / ok:>8.0f}{d_sum / ok:>9.0f}{d_sum / a_sum:>7.2f}"
        )
        print(f"\nrjesivih: {ok}/{len(files)}")
        if mismatch:
            print(
                f"UPOZORENJE: {mismatch} mapa gdje se cijene A*/Dijkstra razlikuju "
                f"-> BUG, moraju biti jednake"
            )
        else:
            print("OK: A* i Dijkstra svugdje daju istu cijenu (isti optimum)")


if __name__ == "__main__":
    main()
