import heapq

from warehouse import neighbours, move_cost


def manhattan(a, b):
    """Distance if there were no obstacles: steps sideways + steps up/down."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def find_path(grid, start, goal, algorithm="astar", allow_restricted=False, blocked=(), use_costs=True):
    """Find the cheapest path from start to goal with "astar" or "dijkstra".

    allow_restricted  True if the robot may enter the cold chain / hazardous zone
    blocked           cells that are blocked by a dynamic obstacle right now
    use_costs         False = every move costs 1 (pure step count, used for validation)
    Returns (path, cost, expanded) or None if there is no path.
    """
    # The open list holds (priority, cost so far, cell).
    open_list = [(0.0, 0.0, start)]
    best_cost = {start: 0.0}     # cheapest known cost to reach each cell
    parent = {start: None}       # the cell we came from, to rebuild the path
    closed = set()               # cells that are finished
    expanded = 0                 # how many cells we explored

    while open_list:
        _, cost, cell = heapq.heappop(open_list)
        if cell in closed:
            continue
        closed.add(cell)
        expanded += 1

        if cell == goal:
            # Walk backwards from the goal to the start.
            path = []
            while cell is not None:
                path.append(cell)
                cell = parent[cell]
            path.reverse()
            return path, cost, expanded

        for nxt in neighbours(grid, cell, allow_restricted):
            if nxt in blocked:
                continue                  # a dynamic obstacle is on this cell
            step = move_cost(grid, nxt) if use_costs else 1.0
            new_cost = cost + step
            if nxt not in best_cost or new_cost < best_cost[nxt]:
                best_cost[nxt] = new_cost
                parent[nxt] = cell
                # THE ONLY DIFFERENCE: A* adds an estimate of the distance still to go.
                if algorithm == "astar":
                    estimate = manhattan(nxt, goal)
                else:
                    estimate = 0
                heapq.heappush(open_list, (new_cost + estimate, new_cost, nxt))

    return None