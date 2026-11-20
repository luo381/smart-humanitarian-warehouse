import csv


def read_csv(name):
    with open("data/" + name, encoding="utf-8") as f:
        return list(csv.DictReader(f))


def load_bins():
    """bin_id -> the cell where the robot stops to pick from that bin."""
    bins = {}
    for r in read_csv("storage_locations.csv"):
        bins[r["bin_id"]] = (int(r["access_x"]), int(r["access_y"]))
    return bins


def load_items():
    """sku -> name, weight and whether it is a cold chain item."""
    items = {}
    for r in read_csv("items.csv"):
        items[r["sku"]] = {
            "name": r["item_name"],
            "weight": float(r["unit_weight_kg"]),
            "cold": r["temperature_control"] != "Ambient",
            "restricted": "Authorised picker only" in r["handling_notes"],
        }
    return items


def load_inventory():
    """List of lots: where each batch of stock is and when it expires."""
    lots = []
    for r in read_csv("inventory.csv"):
        lots.append({
            "bin_id": r["bin_id"],
            "sku": r["sku"],
            "lot": r["lot_number"],
            "qty": int(r["quantity"]),
            "expiry": r["expiry_date"] or "9999-12-31",   # no expiry date = never expires
        })
    return lots


def load_orders():
    """order_id -> dock, priority, deadline and the list of (sku, quantity) lines."""
    orders = {}
    for r in read_csv("picking_orders.csv"):
        orders[r["order_id"]] = {
            "dock": r["dispatch_dock"],
            "priority": r["priority"],
            "deadline_minutes": int(r["deadline_minutes"]),
            "lines": [],
        }
    for r in read_csv("picking_order_lines.csv"):
        orders[r["order_id"]]["lines"].append((r["sku"], int(r["quantity"])))
    return orders


def load_docks():
    """dock id -> the cell where the robot stops, plus the robot start cell."""
    docks, start = {}, None
    for r in read_csv("key_points.csv"):
        stop = (int(r["robot_stop_x"]), int(r["robot_stop_y"]))
        if r["type"].startswith("Dispatch"):
            docks[r["point_id"]] = stop
        else:
            start = stop
    return docks, start