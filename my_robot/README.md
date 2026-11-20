# Smart Humanitarian Warehouse: Warehouse Robot Navigation

CSCI218 group project (Topic 11, Dubai Humanitarian use case).
A simulated robot collects relief items from the warehouse racks and brings them to a
dispatch dock. It plans routes with A*, is controlled by a finite state machine, and
replans when an aisle becomes blocked.

## How to run

```
python3 -m pip install -r requirements.txt
python3 validate.py          # check the dataset
python3 test_project.py      # 15 automatic tests
python3 viewer.py            # live demo window (see below)
python3 main.py              # the same order as a text log
python3 experiment1.py       # validation + A* vs Dijkstra
python3 experiment2.py       # 200 orders, pick sequencing
python3 experiment3.py       # fixed obstacles, three strategies
python3 experiment4.py       # moving obstacles, three strategies
python3 charts.py            # charts and map in results/
```

Live demo options:

```
python3 viewer.py ORDER [replan|wait|ignore] [fixed|moving|blocked|none]
python3 viewer.py ORD-0009 replan moving
```

Keys: SPACE pause, UP/DOWN speed, R restart, ESC quit.

## Files

| File | What it does |
|---|---|
| `warehouse.py` | Loads the grid; which cells are passable; move costs |
| `data.py` | Loads bins, items, inventory, orders and docks |
| `validate.py` | Checks that the dataset is complete and consistent |
| `pathfinding.py` | A* and Dijkstra |
| `planner.py` | FEFO stock allocation and nearest-first sequencing |
| `robot.py` | Finite state machine that completes one order |
| `obstacles.py` | Fixed, moving and permanent obstacles |
| `viewer.py` | Live demo window (pygame) |
| `main.py` | Text demo of one order |
| `experiment1-4.py` | Experiments, results saved as CSV |
| `charts.py` | Charts for the report |
| `test_project.py` | Automatic tests |

## Method

- **Grid:** 60 x 40 cells. Walls, racks and pillars are blocked. Floor costs 1,
  restricted zone 2, staging area 3. The restricted zone is entered only if the
  order contains cold chain or hazardous items.
- **Pathfinding:** A* with the Manhattan heuristic. Dijkstra (heuristic 0) is the baseline.
- **Planning:** First-Expired-First-Out allocation, then nearest pick first with cold
  chain picks last. The robot unloads at the dock before exceeding 150 kg.
- **Robot states:** PLAN, NAVIGATE, REPLAN, WAIT, PICK, DELIVER, DONE, FAILED.
- **Strategies:** `replan` (ours), `wait` (baseline), `ignore` (baseline).
- **Obstacles:** fixed ones are known everywhere (reported by the warehouse system);
  moving ones are seen only within 3 cells. For a moving obstacle the robot takes a
  detour of up to 10 extra cost, otherwise it waits 5 s and tries again; after 3 waits
  it takes the long detour.
- **Failure:** if a goal can never be reached the robot stops in the FAILED state
  and reports the reason.

## Results

| Experiment | Result |
|---|---|
| Validation | 1,352 of 1,352 path lengths equal the dataset values |
| A* vs Dijkstra | Same cost in all 1,352 queries; A* explores 264.1 cells, Dijkstra 796.1 (66.8% fewer) |
| Sequencing | Nearest-first 142.0 cells per order, as listed 169.7 (16.4% saved) |
| Stock | 18 of 200 orders had missing stock; 98.1% of units delivered |
| Dataset obstacles | Only 1 of 200 orders meets one of the 40 recorded events |

Fixed obstacles (200 orders x 5 seeds, 2 blockages per order; mean +/- standard deviation across seeds):

| Strategy | Completion time (min) | On time without collision | Collisions |
|---|---|---|---|
| Replan (ours) | 7.28 +/- 0.78 | 99.8% +/- 0.4 | 0 |
| Wait | 39.93 +/- 0.78 | 92.3% +/- 1.5 | 0 |
| Ignore | 5.07 +/- 0.00 | 0% | 2,214 |

Moving obstacles (200 orders x 5 seeds, 2 patrols per order; no obstacles: 304.4 s):

| Strategy | Completion time (s) | Orders without collision | Collisions | Bumped |
|---|---|---|---|---|
| Replan (ours) | 312.3 +/- 0.4 | 100% | 0 | 1,035 |
| Wait | 324.4 +/- 0.3 | 100% | 0 | 3,420 |
| Ignore | 304.4 +/- 0.0 | 0.2% +/- 0.3 | 2,316 | 1,990 |

"Collisions" = the robot drove into a cell an obstacle was on or about to enter.
"Bumped" = a moving obstacle walked onto the cell where the robot was standing
(for example while it was picking). The patrols in our simulation never stop, so this
happens about once per order; a real person or forklift would stop.

## Assumptions and limitations

- One robot; 1 cell = 1 m at 1 m/s; pick time 20 s + 3 s per unit; unload 30 s; capacity 150 kg.
- Every order starts at the charging station; orders do not queue.
- Obstacles for the experiments are placed on the robot's own route on purpose (stress test).
- Moving obstacles follow a fixed patrol and do not react to the robot. We count
  separately how often they walk onto the robot ("bumped"); the robot cannot prevent this.
- If one pick cannot be reached the whole order fails; the robot does not skip it.
- Not modelled: shelf levels, battery, cold chain time limit, several robots, restocking.
