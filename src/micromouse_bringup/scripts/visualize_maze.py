#!/usr/bin/env python3
import json
import argparse
import sys

def main():
    parser = argparse.ArgumentParser(description="Visualize maze JSON as text.")
    parser.add_argument('--json', default='maze_truth.json', help='Path to the maze JSON file')
    args = parser.parse_args()

    try:
        with open(args.json, 'r') as f:
            data = json.load(f)
    except FileNotFoundError:
        print(f"Error: Could not find {args.json}")
        sys.exit(1)

    N = data['N']
    east = data['east']
    north = data['north']
    start_cell = tuple(data['start'])
    
    # Convert goals to a set of tuples for fast lookup
    goals = set(tuple(g) for g in data['goal'])

    print(f"--- Maze Visualization ({N}x{N}) ---")
    
    # Since +y is up (North) and +x is right (East), we print from top (N-1) down to bottom (0)
    for y in range(N - 1, -1, -1):
        # 1. Draw the North walls (and the corner posts) for the current row
        north_line = "+"
        for x in range(N):
            if north[x][y]:
                north_line += "---+"
            else:
                north_line += "   +"
        print(north_line)

        # 2. Draw the West outer boundary, cell contents, and East walls
        # The leftmost wall (West boundary) is always solid in a standard maze
        cell_line = "|"
        for x in range(N):
            # Determine cell content
            if (x, y) == start_cell:
                cell_char = " S "  # Start
            elif (x, y) in goals:
                cell_char = " G "  # Goal
            else:
                cell_char = "   "  # Empty space
                
            # Determine East wall
            if east[x][y]:
                wall_char = "|"
            else:
                wall_char = " "
                
            cell_line += cell_char + wall_char
            
        print(cell_line)

    # 3. Draw the bottom South boundary (y=0 has a solid perimeter at the bottom)
    bottom_line = "+" + "---+" * N
    print(bottom_line)

if __name__ == '__main__':
    main()
