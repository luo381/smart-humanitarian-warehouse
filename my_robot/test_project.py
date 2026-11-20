"""Automatic checks. Run with:  python3 test_project.py"""
from warehouse import load_grid, passable
from data import read_csv, load_bins, load_items, load_inventory, load_orders, load_docks
from pathfinding import find_path
from planner import allocate, sequence
from robot import run_order

grid = load_grid()
bins, items, orders = load_bins(), load_items(), load_orders()
docks, start = load_docks()


def test_data_is_loaded():
    assert len(grid) == 40 and len(grid[0]) == 60
    assert len(bins) == 1014 and len(items) == 20 and len(orders) == 200


def test_walls_and_racks_are_blocked():
    assert not passable(grid, (0, 0))          # wall
    assert not passable(grid, (5, 6))          # rack
    assert passable(grid, (3, 8))              # floor


def test_restricted_zone_needs_permission():
    assert find_path(grid, start, (52, 12)) is None
    assert find_path(grid, start, (52, 12), "astar", True) is not None


def test_path_lengths_match_the_dataset():
    for r in read_csv("storage_locations.csv")[::25]:
        cell = (int(r["access_x"]), int(r["access_y"]))
        path, cost, expanded = find_path(grid, docks["D1"], cell, "astar", True, (), False)
        assert len(path) - 1 == int(r["path_len_from_D1_m"])


def test_astar_is_as_good_as_dijkstra_and_explores_less():
    for goal in [(10, 37), (48, 37), (47, 6), (28, 10)]:
        a = find_path(grid, start, goal, "astar")
        d = find_path(grid, start, goal, "dijkstra")
        assert a[1] == d[1]                    # same cost
        assert a[2] <= d[2]                    # fewer or equal cells explored


def test_path_avoids_blocked_cells():
    blocked = {(27, 7), (28, 7)}
    path, cost, expanded = find_path(grid, start, (28, 10), "astar", False, blocked)
    assert not blocked.intersection(path)


def test_fefo_takes_the_earliest_expiry_first():
    inventory = load_inventory()
    order = {"lines": [("SKU-105", 1)]}        # bottled water
    picks, missing = allocate(order, inventory, bins, items)
    assert picks[0]["expiry"] == "2026-12-04" and missing == []


def test_missing_stock_is_reported():
    inventory = load_inventory()
    order = {"lines": [("SKU-106", 1000)]}     # far more tents than exist
    picks, missing = allocate(order, inventory, bins, items)
    assert missing[0][0] == "SKU-106" and missing[0][1] > 0


def test_cold_chain_picks_come_last():
    inventory = load_inventory()
    picks, missing = allocate(orders["ORD-0009"], inventory, bins, items)
    flags = [p["cold"] for p in sequence(picks, start)]
    assert flags == sorted(flags)              # all False first, then all True


def test_robot_never_overloads_and_replan_never_collides():
    inventory = load_inventory()
    picks, missing = allocate(orders["ORD-0009"], inventory, bins, items)
    picks = sequence(picks, start)
    obstacles = [{"cells": {(27, 7), (28, 7)}, "start": 10, "end": 610}]
    replan = run_order(grid, picks, start, docks["D2"], True, obstacles, "replan", show=False)
    ignore = run_order(grid, picks, start, docks["D2"], True, obstacles, "ignore", show=False)
    wait = run_order(grid, picks, start, docks["D2"], True, obstacles, "wait", show=False)
    assert replan["collisions"] == 0 and replan["replans"] == 1
    assert ignore["collisions"] > 0
    assert wait["time"] > replan["time"]
    assert replan["trips"] == 2                # 271 kg in total, capacity 150 kg


def test_moving_obstacles_are_handled_without_collision():
    from obstacles import patrols
    inventory = load_inventory()
    picks, missing = allocate(orders["ORD-0009"], inventory, bins, items)
    picks = sequence(picks, start)
    normal = run_order(grid, picks, start, docks["D2"], True, show=False)
    obstacles = patrols(normal["route"], seed=1, count=3)
    replan = run_order(grid, picks, start, docks["D2"], True, obstacles, "replan", show=False)
    ignore = run_order(grid, picks, start, docks["D2"], True, obstacles, "ignore", show=False)
    assert replan["collisions"] == 0
    assert ignore["collisions"] > 0


def test_driving_through_a_moving_obstacle_is_counted():
    """Regression test: the "ignore" robot of ORD-0144 passes through a patrol and must be caught."""
    from obstacles import patrols
    inventory = load_inventory()
    order = orders["ORD-0144"]
    picks, missing = allocate(order, inventory, bins, items)
    picks = sequence(picks, start)
    allowed = any(items[sku]["restricted"] for sku, qty in order["lines"])
    normal = run_order(grid, picks, start, docks[order["dock"]], allowed, show=False)
    obstacles = patrols(normal["route"], seed=1, count=3)
    ignore = run_order(grid, picks, start, docks[order["dock"]], allowed, obstacles, "ignore", show=False)
    replan = run_order(grid, picks, start, docks[order["dock"]], allowed, obstacles, "replan", show=False)
    assert ignore["collisions"] > 0
    assert replan["collisions"] == 0


def test_unreachable_goal_ends_in_failed_state():
    from obstacles import permanent_block
    inventory = load_inventory()
    picks, missing = allocate(orders["ORD-0009"], inventory, bins, items)
    picks = sequence(picks, start)
    obstacles = [permanent_block(picks[1]["cell"])]
    replan = run_order(grid, picks, start, docks["D2"], True, obstacles, "replan", show=False)
    wait = run_order(grid, picks, start, docks["D2"], True, obstacles, "wait", show=False)
    assert not replan["completed"] and replan["reason"] == "The goal cannot be reached"
    assert not wait["completed"]
    assert replan["time"] <= wait["time"]      # replanning notices the problem earlier


def test_normal_order_is_completed():
    inventory = load_inventory()
    picks, missing = allocate(orders["ORD-0009"], inventory, bins, items)
    result = run_order(grid, sequence(picks, start), start, docks["D2"], True, show=False)
    assert result["completed"] and result["reason"] == ""


def test_dataset_passes_validation():
    from validate import validate
    problems, summary = validate()
    assert problems == []


if __name__ == "__main__":
    tests = [(name, f) for name, f in sorted(globals().items()) if name.startswith("test_")]
    for name, f in tests:
        f()
        print("PASS", name)
    print("All", len(tests), "tests passed")
