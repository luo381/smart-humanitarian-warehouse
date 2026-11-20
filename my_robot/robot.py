from pathfinding import find_path
from warehouse import move_cost

CAPACITY_KG = 150      # the robot must unload at the dock before it gets heavier than this
PICK_SECONDS = 20      # time to stop at a bin
SECONDS_PER_UNIT = 3   # extra time for every unit picked
UNLOAD_SECONDS = 30    # time to unload at the dock


SENSOR_RANGE = 3       # the robot notices moving obstacles within this many cells
LOOK_AHEAD = 2         # it reacts to a moving obstacle only on the next cells of its route
RETRY_SECONDS = 5      # how long to wait before trying again when a moving obstacle is in the way
DETOUR_LIMIT = 10      # a moving obstacle is worth driving around only if the detour costs less than this
PATIENCE = 3           # after this many waits in a row the robot takes the long detour anyway
MAX_WAITS = 120        # give up (FAILED) after this many waits in a row without moving


def is_active(obstacle, time):
    return obstacle["start"] <= time < obstacle["end"]


def moving_cells(obstacle, time):
    """A moving obstacle patrols up and down its track, one cell every step_seconds.

    It blocks the cell it is on and the cell it will enter next.
    """
    track = obstacle["track"]
    last = len(track) - 1
    step = int((time - obstacle["start"]) / obstacle["step_seconds"])
    cells = set()
    for s in [step, step + 1]:
        k = s % (2 * last)                       # position in the there-and-back cycle
        cells.add(track[k if k <= last else 2 * last - k])
    return cells


def occupied_cells(obstacles, time):
    """The cells an obstacle is physically standing on at this moment."""
    cells = set()
    for obstacle in obstacles:
        if is_active(obstacle, time):
            if "track" in obstacle:
                track = obstacle["track"]
                last = len(track) - 1
                k = int((time - obstacle["start"]) / obstacle["step_seconds"]) % (2 * last)
                cells.add(track[k if k <= last else 2 * last - k])
            else:
                cells.update(obstacle["cells"])
    return cells


def blocked_cells(obstacles, time):
    """All cells the robot must not enter now: where an obstacle is, and where a moving one goes next."""
    cells = set()
    for obstacle in obstacles:
        if is_active(obstacle, time):
            cells.update(moving_cells(obstacle, time) if "track" in obstacle else obstacle["cells"])
    return cells


def seen_cells(obstacles, time, position, include_moving=True):
    """The blocked cells the robot knows about.

    Fixed obstacles (spill, fallen box) are reported by the warehouse system, so all are known.
    Moving obstacles (forklift, person) are only seen by the robot's own sensor when they are close.
    """
    cells = set()
    for obstacle in obstacles:
        if not is_active(obstacle, time):
            continue
        if "track" not in obstacle:
            cells.update(obstacle["cells"])
        elif include_moving:
            for cell in moving_cells(obstacle, time):
                if abs(cell[0] - position[0]) + abs(cell[1] - position[1]) <= SENSOR_RANGE:
                    cells.add(cell)
    return cells


def run_order(grid, picks, start, dock, allow_restricted, obstacles=(), strategy="replan", show=True):
    """Carry out one order with a finite state machine. Returns the results.

    strategy  "replan"  find a new route when the route is blocked (our approach)
              "wait"    stop in front of the obstacle until it is gone (baseline)
              "ignore"  drive through obstacles (baseline, counts collisions)
    """
    position = start
    time = 0.0
    load = 0.0
    todo = list(picks)     # picks not collected yet
    target = None          # the pick we are driving to (None = driving to the dock)
    path = []
    distance = 0
    trips = 0
    replans = 0
    collisions = 0
    waited = 0.0
    route = []             # every (time, cell) the robot visited, used by the experiments
    waits_in_a_row = 0     # how many times in a row the robot has waited
    in_collision = False   # True while the robot is driving through an obstacle
    bumped = 0             # how many times an obstacle moved onto the cell the robot was standing on
    has_moving = any("track" in o for o in obstacles)
    events = []            # (time, what happened), so the viewer can count while it plays

    def stay(seconds):
        """The robot stays on its cell for some seconds. Count obstacles that move onto it."""
        nonlocal time, bumped
        end = time + seconds
        # Moments to check: every second if something moves, plus the start of each fixed obstacle.
        moments = [o["start"] for o in obstacles if "track" not in o and time < o["start"] < end]
        if has_moving and seconds < 100000:
            moments += range(int(time) + 1, int(end) + 1)
        under = position in occupied_cells(obstacles, time)
        for moment in sorted(m for m in moments if m <= end):
            now = position in occupied_cells(obstacles, moment)
            if now and not under:
                bumped += 1
                events.append((moment, "bumped"))
            under = now
        time = end

    def log(text):
        if show:
            print(f"t={time:6.0f}s  {text}")

    state = "PLAN"
    reason = ""            # why the order failed (empty if it did not fail)
    loops = 0
    while state not in ("DONE", "FAILED"):
        loops += 1
        if loops > 200000:                     # safety net: never loop forever
            state, reason = "FAILED", "The robot is stuck"
            continue

        if state == "PLAN":
            # Go to the dock if nothing is left or the next pick is too heavy.
            if not todo or load + todo[0]["weight"] > CAPACITY_KG:
                target = None
                goal = dock
            else:
                target = todo[0]
                goal = target["cell"]
            # Only the replanning robot plans around the obstacles it knows about.
            avoid = seen_cells(obstacles, time, position) if strategy == "replan" else set()
            result = find_path(grid, position, goal, "astar", allow_restricted, avoid)
            # DECISION: drive around a moving obstacle, or wait for it to move away?
            fixed_only = seen_cells(obstacles, time, position, False) if strategy == "replan" else set()
            if avoid != fixed_only:            # a moving obstacle is in sight
                direct = find_path(grid, position, goal, "astar", allow_restricted, fixed_only)
                too_long = result is not None and direct is not None and result[1] > direct[1] + DETOUR_LIMIT
                if too_long and waits_in_a_row < PATIENCE:
                    result = None              # the detour is too long: waiting is cheaper
            if result is None:
                state = "WAIT"                 # no good route right now
            else:
                waits_in_a_row = 0
                path = result[0][1:]           # drop the cell we are standing on
                log(f"PLAN      from {position} to {goal}: {len(path)} steps")
                state = "NAVIGATE"

        elif state == "NAVIGATE":
            blocked = blocked_cells(obstacles, time)
            if not path:                       # arrived
                state = "DELIVER" if target is None else "PICK"
            elif strategy == "replan" and (
                    seen_cells(obstacles, time, position, False).intersection(path)
                    or seen_cells(obstacles, time, position).intersection(path[:LOOK_AHEAD])):
                # A fixed obstacle anywhere on the route, or a moving one right in front of us.
                state = "REPLAN"
            elif strategy == "wait" and path[0] in blocked:
                state = "WAIT"                 # the next cell is blocked
            else:
                entering_obstacle = path[0] in blocked
                if entering_obstacle and not in_collision:
                    collisions += 1            # the robot drives into an obstacle ("ignore" only)
                    events.append((time, "collision"))
                in_collision = entering_obstacle
                waits_in_a_row = 0
                position = path.pop(0)         # move one cell
                stay(move_cost(grid, position))
                distance += 1
                route.append((time, position))

        elif state == "REPLAN":
            replans += 1
            events.append((time, "replan"))
            log(f"REPLAN    route blocked, robot is at {position}")
            state = "PLAN"

        elif state == "WAIT":
            active = [o for o in obstacles if is_active(o, time)]
            # Fixed obstacle: jump to the moment it disappears. Moving obstacle: try again soon.
            until = min([o["end"] for o in active], default=float("inf"))
            if any("track" in o for o in active):
                until = min(until, time + RETRY_SECONDS)
            # Give up instead of waiting forever: nothing will ever clear, or we waited too long.
            if until == float("inf"):
                state, reason = "FAILED", "The goal cannot be reached"
                log("FAILED    " + reason)
                continue
            if waits_in_a_row >= MAX_WAITS:
                state, reason = "FAILED", "Waited too long for a moving obstacle"
                log("FAILED    " + reason)
                continue
            log(f"WAIT      {until - time:.0f} s")
            waited += until - time
            stay(until - time)
            waits_in_a_row += 1
            state = "PLAN" if strategy == "replan" else "NAVIGATE"

        elif state == "PICK":
            stay(PICK_SECONDS + SECONDS_PER_UNIT * target["qty"])
            load += target["weight"]
            todo.remove(target)
            log(f"PICK      {target['qty']} x {target['sku']}  load = {load:.0f} kg")
            state = "PLAN"

        elif state == "DELIVER":
            stay(UNLOAD_SECONDS)
            trips += 1
            log(f"DELIVER   unloaded {load:.0f} kg at the dock")
            load = 0.0
            state = "PLAN" if todo else "DONE"

    if state == "DONE":
        log("DONE")
    return {"completed": state == "DONE", "reason": reason,
            "time": time, "distance": distance, "trips": trips,
            "replans": replans, "collisions": collisions, "bumped": bumped,
            "waited": waited, "route": route, "events": events}
