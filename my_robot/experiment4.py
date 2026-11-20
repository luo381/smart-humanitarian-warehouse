"""Experiment 4: moving obstacles (forklifts and people walking in the aisles)."""
import csv
import os
from statistics import mean, stdev

from warehouse import load_grid
from data import load_bins, load_items, load_inventory, load_orders, load_docks
from planner import allocate, sequence
from robot import run_order
from obstacles import patrols

SEEDS = [1, 2, 3, 4, 5]
PATROLS_PER_ORDER = 2
STRATEGIES = ["replan", "wait", "ignore"]

grid = load_grid()
bins = load_bins()
items = load_items()
inventory = load_inventory()
orders = load_orders()
docks, start = load_docks()
os.makedirs("results", exist_ok=True)

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
    job["normal"] = run_order(grid, job["picks"], start, job["dock"], job["allow_restricted"], show=False)
    jobs.append(job)

rows = []
for seed in SEEDS:
    for job in jobs:
        # Same seed and order = the same moving obstacles for every strategy.
        obstacles = patrols(job["normal"]["route"], seed * 100000 + int(job["order_id"][-4:]), PATROLS_PER_ORDER)
        for strategy in STRATEGIES:
            r = run_order(grid, job["picks"], start, job["dock"], job["allow_restricted"],
                          obstacles, strategy, show=False)
            on_time = r["time"] <= job["deadline"]
            rows.append({
                "seed": seed, "order_id": job["order_id"], "priority": job["priority"], "strategy": strategy,
                "time": r["time"], "distance": r["distance"], "waited": r["waited"],
                "replans": r["replans"], "collisions": r["collisions"], "bumped": r["bumped"],
                "on_time": on_time, "success": on_time and r["collisions"] == 0,
            })
    print("  seed", seed, "finished")

with open("results/experiment4.csv", "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    writer.writeheader()
    writer.writerows(rows)

normal_time = mean(j["normal"]["time"] for j in jobs)
print("MOVING OBSTACLES:", len(jobs), "orders x", len(SEEDS), "seeds,", PATROLS_PER_ORDER, "patrols per order")
print("  no obstacles: average time", round(normal_time, 1), "s")
for strategy in STRATEGIES:
    mine = [r for r in rows if r["strategy"] == strategy]
    seconds = [mean(r["time"] for r in mine if r["seed"] == s) for s in SEEDS]
    safe = [mean(r["collisions"] == 0 for r in mine if r["seed"] == s) * 100 for s in SEEDS]
    print(" ", strategy.ljust(7),
          "time", round(mean(seconds), 1), "+/-", round(stdev(seconds), 1), "s |",
          "extra", round(mean(seconds) - normal_time, 1), "s |",
          "orders without collision", round(mean(safe), 1), "+/-", round(stdev(safe), 1), "% |",
          "collisions", sum(r["collisions"] for r in mine), "|",
          "bumped", sum(r["bumped"] for r in mine), "|",
          "replans", sum(r["replans"] for r in mine), "|",
          "waited", round(mean(r["waited"] for r in mine), 1), "s")
print("Saved results/experiment4.csv")
