"""Experiment 3: dynamic obstacles. Replanning vs waiting vs ignoring."""
import csv
import os
import random
from datetime import datetime
from statistics import mean, stdev

from warehouse import load_grid, passable
from data import read_csv, load_bins, load_items, load_inventory, load_orders, load_docks
from planner import allocate, sequence
from robot import run_order

SEEDS = [1, 2, 3, 4, 5]        # the whole test is repeated with 5 different random seeds
BLOCKAGES_PER_ORDER = 2
STRATEGIES = ["replan", "wait", "ignore"]

grid = load_grid()
bins = load_bins()
items = load_items()
inventory = load_inventory()
orders = load_orders()
docks, start = load_docks()
os.makedirs("results", exist_ok=True)

# ---------- Prepare every order once (same picks for every strategy) ----------
jobs = []
for order_id, order in orders.items():
    picks, missing = allocate(order, inventory, bins, items)
    job = {
        "order_id": order_id,
        "priority": order["priority"],
        "deadline": order["deadline_minutes"] * 60,
        "picks": sequence(picks, start),
        "dock": docks[order["dock"]],
        "allow_restricted": any(items[sku]["restricted"] for sku, qty in order["lines"]),
    }
    # The route the robot takes when nothing is in the way.
    job["normal"] = run_order(grid, job["picks"], start, job["dock"], job["allow_restricted"], show=False)
    jobs.append(job)

# ---------- Part A: the 40 obstacle events recorded in the dataset ----------
fmt = "%Y-%m-%d %H:%M:%S"
created = {r["order_id"]: datetime.strptime(r["created_datetime"], fmt) for r in read_csv("picking_orders.csv")}
events = read_csv("dynamic_obstacles.csv")
durations = [int(e["duration_minutes"]) * 60 for e in events]

affected = 0
for job in jobs:
    hit = False
    for e in events:
        cell = (int(e["x"]), int(e["y"]))
        # Obstacle start and end in seconds after this order was created.
        begin = (datetime.strptime(e["start_datetime"], fmt) - created[job["order_id"]]).total_seconds()
        end = begin + int(e["duration_minutes"]) * 60
        for t, position in job["normal"]["route"]:
            if position == cell and begin <= t < end:
                hit = True
    if hit:
        affected += 1
print("PART A - obstacle events from the dataset")
print("  events:", len(events), " orders whose route meets an active obstacle:", affected, "of", len(jobs))


# ---------- Part B: stress test with random blockages on the robot's own route ----------
def make_blockages(job, seed):
    """Random blockages on the route of this order. Same seed = same blockages."""
    rng = random.Random(seed * 100000 + int(job["order_id"][-4:]))
    candidates = [(t, c) for t, c in job["normal"]["route"][5:] if c != job["dock"] and c != start]
    blockages = []
    for t, (x, y) in rng.sample(candidates, min(BLOCKAGES_PER_ORDER, len(candidates))):
        # Block the cell and its free left/right neighbours, so a 2-cell aisle is fully closed.
        cells = set()
        for cx in [x - 1, x, x + 1]:
            if passable(grid, (cx, y), True) and (cx, y) != job["dock"] and (cx, y) != start:
                cells.add((cx, y))
        begin = t - rng.uniform(5, 30)                 # appears shortly before the robot arrives
        blockages.append({"cells": cells, "start": begin, "end": begin + rng.choice(durations)})
    return blockages


rows = []
for seed in SEEDS:
    for job in jobs:
        obstacles = make_blockages(job, seed)
        for strategy in STRATEGIES:
            r = run_order(grid, job["picks"], start, job["dock"], job["allow_restricted"],
                          obstacles, strategy, show=False)
            on_time = r["time"] <= job["deadline"]
            rows.append({
                "seed": seed, "order_id": job["order_id"], "priority": job["priority"], "strategy": strategy,
                "time": r["time"], "distance": r["distance"], "waited": r["waited"],
                "replans": r["replans"], "collisions": r["collisions"],
                "on_time": on_time, "success": on_time and r["collisions"] == 0,
            })
    print("  seed", seed, "finished")

with open("results/experiment3.csv", "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    writer.writeheader()
    writer.writerows(rows)

print("PART B - stress test:", len(jobs), "orders x", len(SEEDS), "seeds,",
      BLOCKAGES_PER_ORDER, "blockages per order")
print("  no obstacles: average time", round(mean(j["normal"]["time"] for j in jobs) / 60, 2), "min")
for strategy in STRATEGIES:
    mine = [r for r in rows if r["strategy"] == strategy]
    # One average per seed, then mean and standard deviation across the 5 seeds.
    minutes = [mean(r["time"] for r in mine if r["seed"] == s) / 60 for s in SEEDS]
    success = [mean(r["success"] for r in mine if r["seed"] == s) * 100 for s in SEEDS]
    urgent = mean(r["success"] for r in mine if r["priority"] == "Urgent") * 100
    print(" ", strategy.ljust(7),
          "time", round(mean(minutes), 2), "+/-", round(stdev(minutes), 2), "min |",
          "safe and on time", round(mean(success), 1), "+/-", round(stdev(success), 1), "% |",
          "urgent orders", round(urgent, 1), "% |",
          "collisions", sum(r["collisions"] for r in mine), "|",
          "replans", sum(r["replans"] for r in mine))
print("Saved results/experiment3.csv")