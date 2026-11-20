"""Live demo window (pygame): watch the robot carry out one order.

Usage:
    python3 viewer.py                         # ORD-0009, replan, one fixed blockage
    python3 viewer.py ORD-0144                # another order
    python3 viewer.py ORD-0009 wait           # strategy: replan, wait or ignore
    python3 viewer.py ORD-0009 replan moving  # obstacles: fixed, moving, blocked or none

Keys:  SPACE pause/resume   UP/DOWN faster/slower   R restart   ESC quit
"""
import sys
from bisect import bisect_right

import pygame

from warehouse import load_grid
from data import load_bins, load_items, load_inventory, load_orders, load_docks
from planner import allocate, sequence
from robot import run_order, blocked_cells
from obstacles import fixed_blockage, patrols, permanent_block

CELL = 14                 # size of one grid cell in pixels
SIDEBAR = 300             # width of the text panel
COLOURS = {               # one colour per cell code (see layout_legend.csv)
    0: (250, 250, 248), 1: (70, 70, 68), 2: (190, 188, 176), 3: (130, 130, 125),
    4: (205, 226, 251), 5: (150, 195, 245), 6: (246, 217, 163), 7: (236, 235, 230), 8: (243, 240, 225),
}
BLUE, ORANGE, RED, BLACK, GREY, WHITE = (42, 120, 214), (235, 104, 52), (208, 59, 59), (20, 20, 20), (110, 110, 105), (255, 255, 255)

# ---------- Read the command line ----------
order_id = sys.argv[1] if len(sys.argv) > 1 else "ORD-0009"
strategy = sys.argv[2] if len(sys.argv) > 2 else "replan"
mode = sys.argv[3] if len(sys.argv) > 3 else "fixed"

# ---------- Run the simulation first; the window then replays it in real time ----------
grid = load_grid()
bins, items, inventory, orders = load_bins(), load_items(), load_inventory(), load_orders()
docks, start = load_docks()
if order_id not in orders:
    sys.exit("Unknown order: " + order_id)
if strategy not in ["replan", "wait", "ignore"] or mode not in ["fixed", "moving", "blocked", "none"]:
    sys.exit("Usage: python3 viewer.py ORDER [replan|wait|ignore] [fixed|moving|blocked|none]")

order = orders[order_id]
dock = docks[order["dock"]]
picks, missing = allocate(order, inventory, bins, items)
picks = sequence(picks, start)
allow_restricted = any(items[sku]["restricted"] for sku, qty in order["lines"])

normal = run_order(grid, picks, start, dock, allow_restricted, show=False)
if mode == "fixed":
    obstacles = [fixed_blockage(grid, normal["route"])]
elif mode == "moving":
    obstacles = patrols(normal["route"], seed=1, count=3)
elif mode == "blocked":
    obstacles = [permanent_block(picks[-1]["cell"])]      # the last pick can never be reached
else:
    obstacles = []

result = run_order(grid, picks, start, dock, allow_restricted, obstacles, strategy, show=False)
route = [(0.0, start)] + result["route"]
route_times = [t for t, cell in route]
end_time = result["time"]

# ---------- Open the window ----------
pygame.init()
width, height = len(grid[0]) * CELL, len(grid) * CELL
screen = pygame.display.set_mode((width + SIDEBAR, height))
pygame.display.set_caption("Smart Humanitarian Warehouse - Robot Navigation")
font = pygame.font.SysFont(None, 22)
small = pygame.font.SysFont(None, 18)
clock = pygame.time.Clock()


def centre(cell):
    return cell[0] * CELL + CELL // 2, cell[1] * CELL + CELL // 2


def write(text, x, y, colour=BLACK, use=None):
    screen.blit((use or font).render(str(text), True, colour), (x, y))


sim_time = 0.0            # simulation clock in seconds
speed = 20                # simulation seconds per real second
paused = False
running = True

while running:
    real_seconds = clock.tick(60) / 1000          # 60 frames per second

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False
            elif event.key == pygame.K_SPACE:
                paused = not paused
            elif event.key == pygame.K_UP:
                speed = min(speed * 2, 640)
            elif event.key == pygame.K_DOWN:
                speed = max(speed // 2, 5)
            elif event.key == pygame.K_r:
                sim_time, paused = 0.0, False

    if not paused:
        sim_time = min(sim_time + real_seconds * speed, end_time)

    # Where is the robot now? The last route point that is not in the future.
    done = bisect_right(route_times, sim_time)
    position = route[done - 1][1]
    blocked = blocked_cells(obstacles, sim_time)

    # ---------- Draw the map ----------
    for y, row in enumerate(grid):
        for x, code in enumerate(row):
            pygame.draw.rect(screen, COLOURS[code], (x * CELL, y * CELL, CELL, CELL))
    for name, cell in docks.items():
        write(name, cell[0] * CELL - 4, (cell[1] + 1) * CELL, BLACK, small)

    # Obstacles, route so far, pick stops, robot.
    for cell in blocked:
        pygame.draw.rect(screen, RED, (cell[0] * CELL + 1, cell[1] * CELL + 1, CELL - 2, CELL - 2))
    if done >= 2:
        pygame.draw.lines(screen, BLUE, False, [centre(cell) for t, cell in route[:done]], 3)
    for pick in picks:
        pygame.draw.circle(screen, ORANGE, centre(pick["cell"]), CELL // 2 - 1)
        pygame.draw.circle(screen, WHITE, centre(pick["cell"]), CELL // 2 - 1, 1)
    pygame.draw.circle(screen, BLACK, centre(position), CELL // 2 + 1)

    def count(kind):
        """How many events of this kind have happened up to now."""
        return sum(1 for t, what in result["events"] if what == kind and t <= sim_time)

    # ---------- Draw the text panel ----------
    pygame.draw.rect(screen, WHITE, (width, 0, SIDEBAR, height))
    x = width + 16
    if sim_time < end_time:
        status = "PAUSED" if paused else "RUNNING"
    else:
        status = "DONE" if result["completed"] else "FAILED"
    lines = [
        ("Order " + order_id, BLACK), ("Priority: " + order["priority"], GREY),
        ("Dock: " + order["dock"] + "   Deadline: " + str(order["deadline_minutes"]) + " min", GREY),
        ("", GREY),
        ("Strategy: " + strategy, BLACK), ("Obstacles: " + mode, BLACK),
        ("", GREY),
        ("Time: " + str(int(sim_time)) + " s of " + str(int(end_time)) + " s", BLACK),
        ("Status: " + status, RED if status == "FAILED" else BLUE),
        ("Speed: x" + str(speed), GREY),
        ("", GREY),
        ("So far", BLACK),
        ("Distance: " + str(done - 1) + " cells", GREY),
        ("Replans: " + str(count("replan")), GREY),
        ("Collisions: " + str(count("collision")), RED if count("collision") else GREY),
        ("Bumped: " + str(count("bumped")), GREY),
        ("", GREY),
        ("Whole order: " + str(result["distance"]) + " cells, " + str(result["trips"]) + " trips", GREY),
    ]
    if result["reason"]:
        lines.append((result["reason"], RED))
    for i, (text, colour) in enumerate(lines):
        write(text, x, 14 + i * 21, colour)

    legend = [(BLACK, "Robot"), (ORANGE, "Pick stop"), (RED, "Obstacle"), (BLUE, "Route so far")]
    for i, (colour, text) in enumerate(legend):
        pygame.draw.rect(screen, colour, (x, height - 130 + i * 20, 12, 12))
        write(text, x + 20, height - 131 + i * 20, GREY, small)
    write("SPACE pause   UP/DOWN speed", x, height - 42, GREY, small)
    write("R restart   ESC quit", x, height - 24, GREY, small)

    pygame.display.flip()

pygame.quit()
print("Finished:", "completed" if result["completed"] else "FAILED", "| time:", round(end_time), "s | replans:",
      result["replans"], "| collisions:", result["collisions"], "| waited:", round(result["waited"]), "s")
