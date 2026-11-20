"""Experiment 1: check our paths against the dataset, then compare A* with Dijkstra."""
import csv
import os
import time

from warehouse import load_grid
from data import read_csv, load_docks
from pathfinding import find_path

grid = load_grid()
docks, start = load_docks()
os.makedirs("results", exist_ok=True)

# Every pick location (many bins share one location) with the path lengths given in the dataset.
locations = {}
for r in read_csv("storage_locations.csv"):
    cell = (int(r["access_x"]), int(r["access_y"]))
    locations[cell] = {dock: int(r["path_len_from_" + dock + "_m"]) for dock in docks}
print("Pick locations:", len(locations), " Docks:", len(docks))

# ---------- Part A: validation ----------
# The dataset counts steps, so here every move costs 1 (use_costs=False).
checked = 0
correct = 0
for cell, expected in locations.items():
    for dock, dock_cell in docks.items():
        path, cost, expanded = find_path(grid, dock_cell, cell, "astar", True, (), False)
        checked += 1
        if len(path) - 1 == expected[dock]:
            correct += 1
print("PART A - validation")
print("  routes checked:", checked, " equal to the dataset:", correct)

# ---------- Part B: A* vs Dijkstra ----------
rows = []
for cell in locations:
    for dock, dock_cell in docks.items():
        row = {"dock": dock, "x": cell[0], "y": cell[1]}
        for algorithm in ["astar", "dijkstra"]:
            begin = time.perf_counter()
            path, cost, expanded = find_path(grid, dock_cell, cell, algorithm, True)
            row[algorithm + "_cost"] = cost
            row[algorithm + "_explored"] = expanded
            row[algorithm + "_ms"] = round((time.perf_counter() - begin) * 1000, 4)
        rows.append(row)

with open("results/experiment1.csv", "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    writer.writeheader()
    writer.writerows(rows)


def average(column):
    return sum(r[column] for r in rows) / len(rows)


same_cost = sum(1 for r in rows if r["astar_cost"] == r["dijkstra_cost"])
print("PART B - A* vs Dijkstra")
print("  queries:", len(rows), " same path cost:", same_cost)
print("  A*        average cells explored:", round(average("astar_explored"), 1))
print("  Dijkstra  average cells explored:", round(average("dijkstra_explored"), 1))
saving = 1 - average("astar_explored") / average("dijkstra_explored")
print("  A* explores", round(saving * 100, 1), "% fewer cells")
print("  A*        average time:", round(average("astar_ms"), 3), "ms")
print("  Dijkstra  average time:", round(average("dijkstra_ms"), 3), "ms")
print("Saved results/experiment1.csv")