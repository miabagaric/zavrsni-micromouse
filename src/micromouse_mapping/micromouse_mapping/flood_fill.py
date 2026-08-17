from collections import deque

from micromouse_mapping.maze_map import N, E, S, W, WALL, FREE, DELTA


class FloodFill:
    def __init__(self, size=16):
        self.size = size
        # 255 predstavlja "beskonačnu" udaljenost (nedostupnu ćeliju)
        self.distances = [[255 for _ in range(size)] for _ in range(size)]

        # Cilj u centru labirinta (4 ćelije za 16x16 standard)
        self.goal_cells = [(7, 7), (7, 8), (8, 7), (8, 8)]

    def set_goal(self, goal_cells):
        """Postavlja novi cilj flood filla (npr. start celija za povratak)."""
        self.goal_cells = goal_cells

    def update_distances(self, maze_walls):
        """
        Preračunava udaljenosti od cilja do svake ćelije koristeći BFS.
        maze_walls je 16x16x4 matrica iz klase MazeMap.
        """
        # 1. Resetiraj sve na 255
        for x in range(self.size):
            for y in range(self.size):
                self.distances[x][y] = 255

        queue = deque()

        # 2. Postavi ciljeve na udaljenost 0 i stavi ih u red čekanja
        for gx, gy in self.goal_cells:
            self.distances[gx][gy] = 0
            queue.append((gx, gy))

        # 3. BFS (Breadth-First Search) - Širenje "vode"
        while queue:
            cx, cy = queue.popleft()
            current_dist = self.distances[cx][cy]

            # Provjeri sve 4 strane
            for direction in (N, E, S, W):
                wall_state = maze_walls[cx][cy][direction]

                # Ključno: Pretpostavljamo da nema zida ako je FREE ili UNKNOWN.
                # To potiče robota da istražuje nepoznata područja!
                if wall_state != WALL:
                    dx, dy = DELTA[direction]
                    nx, ny = cx + dx, cy + dy

                    if 0 <= nx < self.size and 0 <= ny < self.size:  # noqa: SIM102
                        # Ako susjedna ćelija još nije dobila svoju udaljenost
                        if self.distances[nx][ny] == 255:
                            self.distances[nx][ny] = current_dist + 1
                            queue.append((nx, ny))

    def get_best_move(self, cx, cy, maze_walls):
        """
        Na temelju pozicije vraća apsolutni smjer (N, E, S, W) u kojem treba ići.
        """
        best_direction = None
        min_dist = 255

        for direction in (N, E, S, W):
            wall_state = maze_walls[cx][cy][direction]

            # Smijemo ići samo tamo gdje je dokazano ili pretpostavljeno slobodno
            if wall_state != WALL:
                dx, dy = DELTA[direction]
                nx, ny = cx + dx, cy + dy

                if 0 <= nx < self.size and 0 <= ny < self.size:
                    neighbor_dist = self.distances[nx][ny]

                    # Tražimo susjeda s najmanjom udaljenošću
                    if neighbor_dist < min_dist:
                        min_dist = neighbor_dist
                        best_direction = direction

        return best_direction
