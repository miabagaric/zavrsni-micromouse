#!/usr/bin/env python3
"""
Generator micromouse labirinta 16x16 za Gazebo Harmonic.
Pravila (standardni micromouse):
- recursive backtracker nad ne-ciljnim celijama (perfektni labirint -> nema samaca stupica)
- cilj 2x2 u centru s TOCNO JEDNIM ulazom
- petlje se dodaju samo ako NE ostave nijedan stupic bez zida
- svi stupici postoje OSIM centralnog (8,8), koji je jedini bez zidova
- pocetna celija (0,0) ima izlaz iskljucivo prema gore (+y)
Konvencija: +x = istok, +y = sjever. Start = (0,0).
"""
import argparse, json, random

N = 16
CELL = 0.180
WALL_T = 0.012
WALL_H = 0.050
POST = 0.012
SEG = CELL

GOAL = [(7, 7), (8, 7), (7, 8), (8, 8)]
GOALSET = set(GOAL)
PERIM = [('E', 6, 7), ('E', 6, 8), ('E', 8, 7), ('E', 8, 8),
         ('N', 7, 6), ('N', 8, 6), ('N', 7, 8), ('N', 8, 8)]
PERIM_SET = set(PERIM)
CENTER_POST = (8, 8)   # jedini stupic koji se izostavlja


def edge_posts(kind, x, y):
    """Uglovi (stupici) koje dotice zid."""
    if kind == 'E':   # vertikalni
        return (x + 1, y), (x + 1, y + 1)
    else:             # 'N' horizontalni
        return (x, y + 1), (x + 1, y + 1)


def post_wall_counts(east, north):
    cnt = {}
    def bump(c): cnt[c] = cnt.get(c, 0) + 1
    for x in range(N):
        for y in range(N):
            if east[x][y]:
                a, b = edge_posts('E', x, y); bump(a); bump(b)
            if north[x][y]:
                a, b = edge_posts('N', x, y); bump(a); bump(b)
    for y in range(N):   # zapadni rub
        bump((0, y)); bump((0, y + 1))
    for x in range(N):   # juzni rub
        bump((x, 0)); bump((x + 1, 0))
    return cnt


def carve(seed, loops):
    random.seed(seed)
    east = [[True] * N for _ in range(N)]
    north = [[True] * N for _ in range(N)]
    visited = [[False] * N for _ in range(N)]
    
    for gx, gy in GOAL:
        visited[gx][gy] = True
        
    # PRAVILO: Startna celija (0,0) ima izlaz samo prema sjeveru (+y)
    visited[0][0] = True
    visited[0][1] = True
    north[0][0] = False
    stack = [(0, 1)]  # Krecemo od (0,1) kako bismo osigurali da start ostane zatvoren s istoka
    
    while stack:
        x, y = stack[-1]
        nbrs = []
        for d, nx, ny in (('E', x + 1, y), ('W', x - 1, y), ('N', x, y + 1), ('S', x, y - 1)):
            if 0 <= nx < N and 0 <= ny < N and not visited[nx][ny]:
                nbrs.append((d, nx, ny))
        if not nbrs:
            stack.pop(); continue
        d, nx, ny = random.choice(nbrs)
        if d == 'E':   east[x][y] = False
        elif d == 'W': east[nx][ny] = False
        elif d == 'N': north[x][y] = False
        elif d == 'S': north[nx][ny] = False
        visited[nx][ny] = True; stack.append((nx, ny))

    # otvori cilj (2x2) + tocno jedan ulaz
    east[7][7] = False; east[7][8] = False
    north[7][7] = False; north[8][7] = False
    d, x, y = random.choice(PERIM)
    if d == 'E': east[x][y] = False
    else:        north[x][y] = False

    # petlje: brisi zid samo ako oba njegova stupica ostanu s >=1 zida
    cnt = post_wall_counts(east, north)
    added, tries = 0, 0
    while added < loops and tries < 4000:
        tries += 1
        if random.random() < 0.5:
            x, y = random.randint(0, N - 2), random.randint(0, N - 1)
            kind = 'E'; open_now = not east[x][y]
        else:
            x, y = random.randint(0, N - 1), random.randint(0, N - 2)
            kind = 'N'; open_now = not north[x][y]
            
        if (kind, x, y) in PERIM_SET or open_now:
            continue
            
        # Ne dopusti uklanjanje istocnog zida od startne celije (0,0)
        if kind == 'E' and x == 0 and y == 0:
            continue
            
        if (x, y) in GOALSET and ((kind == 'E' and (x + 1, y) in GOALSET) or
                                  (kind == 'N' and (x, y + 1) in GOALSET)):
            continue
        pa, pb = edge_posts(kind, x, y)
        # ne dopusti da bilo koji stupic (osim centra) padne na 0
        if (pa != CENTER_POST and cnt.get(pa, 0) - 1 < 1) or \
           (pb != CENTER_POST and cnt.get(pb, 0) - 1 < 1):
            continue
        if kind == 'E': east[x][y] = False
        else:           north[x][y] = False
        cnt[pa] -= 1; cnt[pb] -= 1
        added += 1
    return east, north


def segments(east, north):
    segs = []
    for x in range(N):
        for y in range(N):
            if east[x][y]:
                segs.append(((x + 0.5) * CELL, y * CELL, WALL_T, SEG))
            if north[x][y]:
                segs.append((x * CELL, (y + 0.5) * CELL, SEG, WALL_T))
    for y in range(N):
        segs.append((-0.5 * CELL, y * CELL, WALL_T, SEG))
    for x in range(N):
        segs.append((x * CELL, -0.5 * CELL, SEG, WALL_T))
    # svi stupici osim centra
    posts = [(i, j) for i in range(N + 1) for j in range(N + 1) if (i, j) != CENTER_POST]
    return segs, posts


def to_sdf(segs, posts):
    p = ['<?xml version="1.0"?>', '<sdf version="1.9">',
         '  <model name="maze">', '    <static>true</static>', '    <link name="walls">']
    n = 0
    for (cx, cy, sx, sy) in segs:
        p.append(f'''      <collision name="c{n}"><pose>{cx:.4f} {cy:.4f} {WALL_H/2:.4f} 0 0 0</pose>
        <geometry><box><size>{sx:.4f} {sy:.4f} {WALL_H:.4f}</size></box></geometry></collision>
      <visual name="v{n}"><pose>{cx:.4f} {cy:.4f} {WALL_H/2:.4f} 0 0 0</pose>
        <geometry><box><size>{sx:.4f} {sy:.4f} {WALL_H:.4f}</size></box></geometry>
        <material><ambient>0.8 0.1 0.1 1</ambient><diffuse>0.8 0.1 0.1 1</diffuse></material></visual>''')
        n += 1
    for (i, j) in posts:
        px, py = (i - 0.5) * CELL, (j - 0.5) * CELL
        p.append(f'''      <collision name="c{n}"><pose>{px:.4f} {py:.4f} {WALL_H/2:.4f} 0 0 0</pose>
        <geometry><box><size>{POST:.4f} {POST:.4f} {WALL_H:.4f}</size></box></geometry></collision>
      <visual name="v{n}"><pose>{px:.4f} {py:.4f} {WALL_H/2:.4f} 0 0 0</pose>
        <geometry><box><size>{POST:.4f} {POST:.4f} {WALL_H:.4f}</size></box></geometry>
        <material><ambient>0.9 0.9 0.9 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material></visual>''')
        n += 1
    p += ['    </link>', '  </model>', '</sdf>']
    return '\n'.join(p), n


def count_goal_entrances(east, north):
    return sum(1 for d, x, y in PERIM
               if (d == 'E' and not east[x][y]) or (d == 'N' and not north[x][y]))


def lone_posts(east, north):
    cnt = post_wall_counts(east, north)
    return [(i, j) for i in range(N + 1) for j in range(N + 1) if cnt.get((i, j), 0) == 0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--loops', type=int, default=12)
    ap.add_argument('--out', default='maze.sdf')
    ap.add_argument('--json', default='maze_truth.json')
    a = ap.parse_args()
    east, north = carve(a.seed, a.loops)
    segs, posts = segments(east, north)
    sdf, count = to_sdf(segs, posts)
    open(a.out, 'w').write(sdf)
    json.dump({'N': N, 'cell': CELL, 'seed': a.seed, 'east': east, 'north': north,
               'start': [0, 0], 'goal': GOAL}, open(a.json, 'w'))
    lones = lone_posts(east, north)
    print(f'seed={a.seed} | segmenata={len(segs)} | stupica={len(posts)} '
          f'| ulaza u cilj={count_goal_entrances(east, north)}')
    print(f'samci stupici (ocekivano samo centar): {lones}')


if __name__ == '__main__':
    main()
