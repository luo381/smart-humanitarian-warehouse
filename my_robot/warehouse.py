import csv

# Cell codes from layout_legend.csv
WALL, RACK, PILLAR, RESTRICTED = 1, 2, 3, 4
BLOCKED = {WALL, RACK, PILLAR}

# Cost of entering a cell: floor 1, restricted zone 2, staging area 3
COST = {0: 1.0, 4: 2.0, 5: 1.0, 6: 1.0, 7: 1.0, 8: 3.0}

# The robot can move right, left, down, up (no diagonal moves).
MOVES = [(1, 0), (-1, 0), (0, 1), (0, -1)]


def load_grid():
    grid = []
    with open("data/layout_grid.csv") as f:
        for row in csv.reader(f):
            grid.append([int(v) for v in row])
    return grid


def passable(grid, cell, allow_restricted=False):
    """Can the robot stand on this cell?"""
    x, y = cell
    if x < 0 or y < 0 or y >= len(grid) or x >= len(grid[0]):
        return False                      # outside the map
    code = grid[y][x]
    if code in BLOCKED:
        return False                      # wall, rack or pillar
    if code == RESTRICTED and not allow_restricted:
        return False                      # cold chain / hazardous zone
    return True


def neighbours(grid, cell, allow_restricted=False):
    """All cells the robot can reach in one move."""
    x, y = cell
    result = []
    for dx, dy in MOVES:
        nxt = (x + dx, y + dy)
        if passable(grid, nxt, allow_restricted):
            result.append(nxt)
    return result


def move_cost(grid, cell):
    x, y = cell
    return COST[grid[y][x]]