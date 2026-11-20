"""Show what happens when a pick location can never be reached."""
from warehouse import load_grid
from data import load_bins, load_items, load_inventory, load_orders, load_docks
from planner import allocate, sequence
from robot import run_order
from obstacles import permanent_block

grid = load_grid()
bins, items, inventory, orders = load_bins(), load_items(), load_inventory(), load_orders()
docks, start = load_docks()

order = orders["ORD-0009"]
picks, missing = allocate(order, inventory, bins, items)
picks = sequence(picks, start)

# Block the second pick location forever.
obstacles = [permanent_block(picks[1]["cell"])]
print("Permanently blocked cell:", picks[1]["cell"])

for strategy in ["replan", "wait", "ignore"]:
    print()
    print("=== strategy:", strategy, "===")
    result = run_order(grid, picks, start, docks[order["dock"]], True, obstacles, strategy)
    print("completed:", result["completed"], "| reason:", result["reason"] or "-",
          "| collisions:", result["collisions"])