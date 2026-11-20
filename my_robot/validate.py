"""Check that the dataset is complete and consistent before we trust any result.

Run with:  python3 validate.py
"""
from warehouse import load_grid, passable, COST, BLOCKED
from data import read_csv, load_docks


def validate():
    """Return a list of problems. An empty list means the data is fine."""
    problems = []
    grid = load_grid()
    height, width = len(grid), len(grid[0])

    # 1. The map is a rectangle and only uses known cell codes.
    if any(len(row) != width for row in grid):
        problems.append("Grid rows have different lengths")
    known_codes = set(COST) | BLOCKED
    for y, row in enumerate(grid):
        for x, code in enumerate(row):
            if code not in known_codes:
                problems.append(f"Unknown cell code {code} at ({x}, {y})")

    # 2. Start and docks are cells the robot can stand on.
    docks, start = load_docks()
    if start is None or not passable(grid, start):
        problems.append("Robot start cell is missing or blocked")
    for name, cell in docks.items():
        if not passable(grid, cell):
            problems.append(f"Dock {name} stop cell {cell} is blocked")

    # 3. Every bin has a reachable pick cell right next to its rack.
    bins = {}
    for r in read_csv("storage_locations.csv"):
        if r["bin_id"] in bins:
            problems.append(f"Duplicate bin {r['bin_id']}")
        bins[r["bin_id"]] = r
        rack = (int(r["rack_x"]), int(r["rack_y"]))
        access = (int(r["access_x"]), int(r["access_y"]))
        if grid[rack[1]][rack[0]] != 2:
            problems.append(f"Bin {r['bin_id']}: rack cell is not a rack")
        if not passable(grid, access, True):
            problems.append(f"Bin {r['bin_id']}: pick cell is blocked")
        if abs(rack[0] - access[0]) + abs(rack[1] - access[1]) != 1:
            problems.append(f"Bin {r['bin_id']}: pick cell is not next to the rack")

    # 4. Items have a positive weight.
    items = {r["sku"]: r for r in read_csv("items.csv")}
    for sku, r in items.items():
        if float(r["unit_weight_kg"]) <= 0:
            problems.append(f"Item {sku} has no weight")

    # 5. Inventory points to bins and items that exist, with stock that is not negative.
    for r in read_csv("inventory.csv"):
        if r["bin_id"] not in bins or r["sku"] not in items:
            problems.append(f"Lot {r['lot_number']} points to an unknown bin or item")
        if int(r["quantity"]) < 0:
            problems.append(f"Lot {r['lot_number']} has negative stock")

    # 6. Orders use known docks and have a positive deadline; every order has lines.
    orders = {r["order_id"]: r for r in read_csv("picking_orders.csv")}
    for order_id, r in orders.items():
        if r["dispatch_dock"] not in docks:
            problems.append(f"Order {order_id} uses unknown dock {r['dispatch_dock']}")
        if int(r["deadline_minutes"]) <= 0:
            problems.append(f"Order {order_id} has no deadline")
    with_lines = set()
    for r in read_csv("picking_order_lines.csv"):
        with_lines.add(r["order_id"])
        if r["order_id"] not in orders or r["sku"] not in items:
            problems.append(f"Order line points to unknown order or item: {r}")
        if int(r["quantity"]) <= 0:
            problems.append(f"Order {r['order_id']} has a line with no quantity")
    for order_id in orders:
        if order_id not in with_lines:
            problems.append(f"Order {order_id} has no lines")

    # 7. Obstacle events are inside the map and last a positive time.
    for r in read_csv("dynamic_obstacles.csv"):
        x, y = int(r["x"]), int(r["y"])
        if not (0 <= x < width and 0 <= y < height):
            problems.append(f"Obstacle {r['event_id']} is outside the map")
        if int(r["duration_minutes"]) <= 0:
            problems.append(f"Obstacle {r['event_id']} has no duration")

    summary = (f"{width}x{height} grid, {len(bins)} bins, {len(items)} items, "
               f"{len(orders)} orders, {len(docks)} docks")
    return problems, summary


if __name__ == "__main__":
    problems, summary = validate()
    if problems:
        print("DATA PROBLEMS FOUND:", len(problems))
        for p in problems[:20]:
            print(" -", p)
    else:
        print("DATA OK:", summary)