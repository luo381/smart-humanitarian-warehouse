from pathfinding import manhattan


def allocate(order, inventory, bins, items):
    """Decide which lots to pick for an order (First-Expired-First-Out).

    Returns (picks, missing). Each pick is one stop at one bin.
    """
    picks, missing = [], []
    for sku, wanted in order["lines"]:
        # All lots of this item that still have stock, earliest expiry first.
        lots = [lot for lot in inventory if lot["sku"] == sku and lot["qty"] > 0]
        lots.sort(key=lambda lot: lot["expiry"])

        left = wanted
        for lot in lots:
            if left == 0:
                break
            take = min(left, lot["qty"])
            lot["qty"] -= take          # the stock is now reserved
            left -= take
            picks.append({
                "sku": sku,
                "qty": take,
                "bin_id": lot["bin_id"],
                "cell": bins[lot["bin_id"]],
                "expiry": lot["expiry"],
                "weight": take * items[sku]["weight"],
                "cold": items[sku]["cold"],
            })
        if left > 0:
            missing.append((sku, left))  # not enough stock
    return picks, missing


def sequence(picks, start):
    """Decide the visiting order: always go to the nearest pick next.

    Cold chain picks go last so they spend less time outside the cold zone.
    """
    normal = [p for p in picks if not p["cold"]]
    cold = [p for p in picks if p["cold"]]

    ordered = []
    position = start
    for group in [normal, cold]:
        while group:
            nearest = min(group, key=lambda p: manhattan(position, p["cell"]))
            group.remove(nearest)
            ordered.append(nearest)
            position = nearest["cell"]
    return ordered
