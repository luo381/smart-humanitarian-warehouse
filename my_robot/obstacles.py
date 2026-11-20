"""Create obstacles for the demos and experiments."""
import random

from warehouse import passable


def fixed_blockage(grid, route, lasts=600):
    """One fixed blockage halfway along a route, across the aisle, for `lasts` seconds."""
    t, (x, y) = route[len(route) // 2]
    cells = {(cx, y) for cx in [x - 1, x, x + 1] if passable(grid, (cx, y), True)}
    return {"cells": cells, "start": t - 10, "end": t - 10 + lasts}


def permanent_block(cell):
    """A blockage that never clears. Used to test what the robot does when a goal is unreachable."""
    return {"cells": {cell}, "start": 0, "end": float("inf")}


def patrols(route, seed, count=2, length=8, step_seconds=2):
    """Moving obstacles (forklift or person) that walk up and down a piece of the robot's route.

    Each one patrols `length` cells of the route for the whole order, one cell every step_seconds.
    """
    rng = random.Random(seed)
    cells = [cell for t, cell in route]
    result = []
    for _ in range(count):
        begin = rng.randrange(5, max(6, len(cells) - length))
        track = []
        for cell in cells[begin:begin + length]:
            if cell in track:                         # the route turns back here: stop the track
                break
            track.append(cell)
        if len(track) >= 4:                       # a very short track would block one spot forever
            result.append({"track": track, "step_seconds": step_seconds, "start": 0, "end": 10 ** 9})
    return result