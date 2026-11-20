"""Demo of one order.

Usage:
    python3 main.py                      # ORD-0009, replan, with one blockage
    python3 main.py ORD-0144             # another order
    python3 main.py ORD-0009 wait        # strategy: replan, wait or ignore
    python3 main.py ORD-0009 replan no   # "no" = run without any obstacle
"""
import sys

from warehouse import load_grid, passable
from data import load_bins, load_items, load_inventory, load_orders, load_docks
from planner import allocate, sequence
from robot import run_order

order_id = sys.argv[1] if len(sys.argv) > 1 else "ORD-0009"
strategy = sys.argv[2] if len(sys.argv) > 2 else "replan"
with_obstacle = not (len(sys.argv) > 3 and sys.argv[3] == "no")

grid = load_grid()
bins = load_bins()
items = load_items()
inventory = load_inventory()
orders = load_orders()
docks, start = load_docks()

if order_id not in orders:
    sys.exit("Unknown order: " + order_id)
if strategy not in ["replan", "wait", "ignore"]:
    sys.exit("Strategy must be replan, wait or ignore")

order = orders[order_id]
dock = docks[order["dock"]]
picks, missing = allocate(order, inventory, bins, items)
picks = sequence(picks, start)
allow_restricted = any(items[sku]["restricted"] for sku, qty in order["lines"])

print("ORDER", order_id, "| priority:", order["priority"], "| dock:", order["dock"],
      "| deadline:", order["deadline_minutes"], "min")
print("PICK PLAN (earliest expiry first, nearest stop first, cold chain last)")
for number, p in enumerate(picks, 1):
    print(" ", number, "-", p["qty"], "x", items[p["sku"]]["name"], "| bin", p["bin_id"],
          "| cell", p["cell"], "|", round(p["weight"]), "kg")
for sku, qty in missing:
    print("  OUT OF STOCK:", qty, "x", items[sku]["name"])

# One blockage: halfway along the normal route, across the aisle,
# appearing 10 seconds before the robot would get there and lasting 10 minutes.
obstacles = []
if with_obstacle:
    normal = run_order(grid, picks, start, dock, allow_restricted, show=False)
    t, (x, y) = normal["route"][len(normal["route"]) // 2]
    cells = {(cx, y) for cx in [x - 1, x, x + 1] if passable(grid, (cx, y), True)}
    obstacles = [{"cells": cells, "start": t - 10, "end": t - 10 + 600}]
    print("OBSTACLE at", sorted(cells), "from", round(t - 10), "s to", round(t + 590), "s")

print("ROBOT LOG (strategy:", strategy + ")")
result = run_order(grid, picks, start, dock, allow_restricted, obstacles, strategy)

print("RESULT")
print("  time:", round(result["time"]), "s | distance:", result["distance"], "cells | trips:", result["trips"])
print("  replans:", result["replans"], "| collisions:", result["collisions"],
      "| bumped:", result["bumped"], "| waited:", round(result["waited"]), "s")
print("  on time:", result["time"] <= order["deadline_minutes"] * 60)
