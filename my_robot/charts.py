"""Draw the charts for the report from the CSV files in results/."""
import csv
from statistics import mean, stdev

import matplotlib
matplotlib.use("Agg")                    # draw to files, no window needed
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

from warehouse import load_grid
from data import load_bins, load_items, load_inventory, load_orders, load_docks
from planner import allocate, sequence
from robot import run_order

BLUE, GREY, ORANGE, RED, INK = "#2a78d6", "#c3c2b7", "#eb6834", "#d03b3b", "#0b0b0b"


def read_results(name):
    with open("results/" + name) as f:
        return list(csv.DictReader(f))


def bar_chart(filename, title, ylabel, labels, values, errors=None, decimals=1):
    """One bar per label. The first bar (our approach) is blue, the others are grey."""
    fig, ax = plt.subplots(figsize=(6, 3.8))
    colours = [BLUE] + [GREY] * (len(labels) - 1)
    bars = ax.bar(labels, values, width=0.5, color=colours, yerr=errors, capsize=3, ecolor="#898781")
    for i, bar in enumerate(bars):
        top = values[i] + (errors[i] if errors else 0)
        ax.text(bar.get_x() + bar.get_width() / 2, top, f" {values[i]:.{decimals}f}", ha="center", va="bottom")
    ax.set_title(title, loc="left", fontweight="bold", fontsize=11)
    ax.set_ylabel(ylabel)
    ax.set_ylim(0, max(values) * 1.2)
    ax.yaxis.grid(True, color="#e1e0d9")
    ax.set_axisbelow(True)
    for side in ["top", "right", "left"]:
        ax.spines[side].set_visible(False)
    ax.tick_params(length=0)
    fig.savefig("results/" + filename, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("Saved results/" + filename)


# ---------- Chart 1: A* vs Dijkstra ----------
rows = read_results("experiment1.csv")
bar_chart("chart1_cells_explored.png", "A* explores fewer cells for the same path cost",
          "Average cells explored per query", ["A*", "Dijkstra"],
          [mean(float(r["astar_explored"]) for r in rows), mean(float(r["dijkstra_explored"]) for r in rows)])

# ---------- Chart 2: pick sequencing ----------
rows = read_results("experiment2.csv")
bar_chart("chart2_sequencing.png", "Nearest-first sequencing shortens the route",
          "Average distance per order (cells)", ["Nearest-first", "As listed"],
          [mean(float(r["nearest_distance"]) for r in rows), mean(float(r["listed_distance"]) for r in rows)])

# ---------- Charts 3 and 4: stress test ----------
rows = read_results("experiment3.csv")
seeds = sorted(set(r["seed"] for r in rows))
strategies = ["replan", "wait", "ignore"]
labels = ["Replan (ours)", "Wait", "Ignore (collides)"]
time_mean, time_std, ok_mean, ok_std = [], [], [], []
for strategy in strategies:
    mine = [r for r in rows if r["strategy"] == strategy]
    minutes = [mean(float(r["time"]) for r in mine if r["seed"] == s) / 60 for s in seeds]
    success = [mean(r["success"] == "True" for r in mine if r["seed"] == s) * 100 for s in seeds]
    time_mean.append(mean(minutes))
    time_std.append(stdev(minutes))
    ok_mean.append(mean(success))
    ok_std.append(stdev(success))
bar_chart("chart3_completion_time.png", "Replanning keeps orders fast when aisles are blocked",
          "Average completion time (minutes)", labels, time_mean, time_std)
bar_chart("chart4_safe_on_time.png", "Only replanning is both safe and on time",
          "Orders on time without collision (%)", labels, ok_mean, ok_std)

# ---------- Map: one order with a blocked aisle ----------
grid = load_grid()
bins, items, inventory, orders = load_bins(), load_items(), load_inventory(), load_orders()
docks, start = load_docks()
order = orders["ORD-0009"]
picks, missing = allocate(order, inventory, bins, items)
picks = sequence(picks, start)
allow_restricted = any(items[sku]["restricted"] for sku, qty in order["lines"])
obstacles = [{"cells": {(27, 7), (28, 7)}, "start": 10, "end": 610}]
result = run_order(grid, picks, start, docks[order["dock"]], allow_restricted, obstacles, "replan", show=False)

# One colour for each cell code 0-8.
cell_colours = ["#fcfcfb", "#52514e", "#c3c2b7", "#898781", "#cde2fb", "#9ec5f4", "#f6d9a3", "#eeede8", "#f3f2ed"]
fig, ax = plt.subplots(figsize=(11, 7.4))
ax.imshow(grid, cmap=ListedColormap(cell_colours), vmin=0, vmax=8)
xs = [start[0]] + [cell[0] for t, cell in result["route"]]
ys = [start[1]] + [cell[1] for t, cell in result["route"]]
ax.plot(xs, ys, color=BLUE, linewidth=2, label="Robot route")
ax.scatter([p["cell"][0] for p in picks], [p["cell"][1] for p in picks], s=60, color=ORANGE,
           edgecolor="white", zorder=3, label="Pick stop")
ax.scatter([c[0] for c in obstacles[0]["cells"]], [c[1] for c in obstacles[0]["cells"]], s=80, marker="X",
           color=RED, edgecolor="white", zorder=4, label="Blocked aisle")
ax.scatter([start[0]], [start[1]], s=70, marker="s", color=INK, zorder=3, label="Start")
for name, (x, y) in docks.items():
    ax.text(x, y + 1, name, ha="center", va="center", fontsize=8)
ax.set_title(f"ORD-0009 with one blocked aisle: {result['replans']} replan, "
             f"{result['distance']} cells, {result['time']:.0f} s", loc="left", fontweight="bold", fontsize=11)
ax.set_xticks([])
ax.set_yticks([])
for side in ax.spines.values():
    side.set_visible(False)
ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.02), ncol=4, frameon=False)
fig.savefig("results/map_route.png", dpi=150, bbox_inches="tight")
plt.close(fig)
print("Saved results/map_route.png")