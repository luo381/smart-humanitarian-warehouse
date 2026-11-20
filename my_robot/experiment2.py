"""Experiment 2: all 200 orders without obstacles. Nearest-first vs the order as listed."""
import csv
import os

from warehouse import load_grid
from data import load_bins, load_items, load_inventory, load_orders, load_docks
from planner import allocate, sequence
from robot import run_order

grid = load_grid()
bins = load_bins()
items = load_items()
inventory = load_inventory()
orders = load_orders()          # already in time order, so stock is used up realistically
docks, start = load_docks()
os.makedirs("results", exist_ok=True)


# Compare nearest-first pick sequencing against the original
# order-line sequence to evaluate warehouse travel efficiency.
# Track distance, execution time, missing stock and multi-trip orders.
rows = []
orders_with_missing = 0
units_wanted = 0
units_missing = 0

for order_id, order in orders.items():
    picks, missing = allocate(order, inventory, bins, items)
    allow_restricted = any(items[sku]["restricted"] for sku, qty in order["lines"])
    dock = docks[order["dock"]]

    units_wanted += sum(qty for sku, qty in order["lines"])
    units_missing += sum(qty for sku, qty in missing)
    if missing:
        orders_with_missing += 1

    # Baseline: visit the picks in the order of the order lines.
    listed = run_order(grid, picks, start, dock, allow_restricted, show=False)
    # Ours: nearest pick first, cold chain last.
    nearest = run_order(grid, sequence(picks, start), start, dock, allow_restricted, show=False)

    rows.append({
        "order_id": order_id, "priority": order["priority"], "stops": len(picks),
        "listed_distance": listed["distance"], "listed_time": listed["time"],
        "nearest_distance": nearest["distance"], "nearest_time": nearest["time"],
        "trips": nearest["trips"], "missing_units": sum(qty for sku, qty in missing),
    })

with open("results/experiment2.csv", "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    writer.writeheader()
    writer.writerows(rows)


def average(column):
    return sum(r[column] for r in rows) / len(rows)


print("Orders completed:", len(rows))
print("As listed      average distance:", round(average("listed_distance"), 1),
      "cells   average time:", round(average("listed_time"), 1), "s")
print("Nearest-first  average distance:", round(average("nearest_distance"), 1),
      "cells   average time:", round(average("nearest_time"), 1), "s")
saving = 1 - average("nearest_distance") / average("listed_distance")
print("Nearest-first saves", round(saving * 100, 1), "% distance")
print("Orders that needed more than one trip:", sum(1 for r in rows if r["trips"] > 1))
print("Orders with missing stock:", orders_with_missing, "of", len(rows))
print("Units delivered:", units_wanted - units_missing, "of", units_wanted,
      "(", round((1 - units_missing / units_wanted) * 100, 1), "% )")
print("Saved results/experiment2.csv")
