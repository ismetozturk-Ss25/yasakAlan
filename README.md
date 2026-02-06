# Forbidden-Zone Avoidance Command Shaper

Real-time path planner for turret/antenna systems that routes around axis-aligned rectangular forbidden zones in AZ-EL (Azimuth-Elevation) space. Designed for DSP deployment at 1 kHz (1 ms cycle time).

## Problem

A turret or antenna must move from its current position to a commanded target in AZ-EL space, but certain rectangular regions are **forbidden** (e.g., sun avoidance, mechanical limits, radar exclusion zones). The system must:

- Never enter a forbidden zone
- Stay inside the working envelope
- Find the shortest safe path in real-time (1 ms per cycle)
- Handle up to 16 simultaneous forbidden zones
- Support AZ wrap-around at the +/-180 degree boundary

```
 EL 60 ┌──────────┬────────┬──────────┬────────┐
       │          │ FORBID │          │ FORBID │
       │  START   │  ZONE  │          │  ZONE  │
       │    *     │████████│  (path)  │████████│   * TARGET
       │          │████████│  ------->│████████│
EL -30 └──────────┴────────┴──────────┴────────┘
     AZ -180                                  AZ +180
```

## Architecture: Two-Phase Design

The algorithm uses a **two-phase architecture** to achieve real-time performance on resource-constrained DSP hardware:

### Phase 1: PRE-OP Mode (Time Unconstrained)

Called **once** when forbidden zones are defined. No deadline.

```c
AvdGraph graph;
Avoidance_BuildGraph(&graph, forbidden, count, &envelope, profile, az_wrap);
```

What it does:
1. **Generate candidate nodes** at each forbidden zone's 4 corners + 4 edge midpoints (offset by `AVD_CORNER_EPS = 0.1 deg`)
2. **Filter** nodes that fall inside forbidden zones or outside the envelope
3. **Compute all-pairs visibility** using Liang-Barsky line-AABB intersection tests
4. **Store results** in a bit-packed adjacency matrix

Result: ~80 nodes, ~1.3 KB memory, all geometry precomputed.

```
Complexity: O(V^2 x N)  where V <= 128 nodes, N <= 16 zones
Typical time: 1-10 ms (no deadline, called once)
Memory: ~3 KB (128 nodes x 2 coords x 4 bytes + 128 x 16 adjacency bytes)
```

### Phase 2: OP Mode (1 ms Deadline)

Called **every 1 ms** during operation.

```c
AvdOutput out = Avoidance_Step(&input, &state, &graph);
// out.az_next, out.el_next  = next position to command
// out.status                = AVD_OK_DIRECT | AVD_OK_WAYPOINT | AVD_NO_PATH
```

The step function executes an 8-stage pipeline:

| Stage | Description | Cost |
|-------|-------------|------|
| 0 | **Escape** - If inside a forbidden zone, move to nearest edge | O(N) |
| 1 | **Clamp** - Clamp target to envelope | O(1) |
| 2 | **Project** - Push target out of forbidden zones | O(N) |
| 3 | **Direct** - Check if straight path to target is clear | O(N) |
| 4 | **Cached A\* path** - Follow previously computed path | O(1) per step |
| 5 | **Greedy committed** - Continue toward committed waypoint | O(N) |
| 6 | **Greedy search** - Find best waypoint among zone corners/midpoints | O(V x N) |
| 7 | **A\* fallback** - Run A\* on precomputed graph | O(V^2) one-time |
| 8 | **NO_PATH** - Hold position if no safe path exists | O(1) |

## Algorithm Details

### Greedy One-Step Planner (Stage 6)

For each forbidden zone, generates 8 candidate waypoints (4 corners + 4 edge midpoints). Evaluates each candidate with a through-score:

```
f(w) = g(w) + h(w)
     = manhattan(current, w) + manhattan(w, target)
```

Prefers **through-waypoints** (waypoints that can see the target directly) over **intermediate** waypoints. Uses Manhattan (L1) distance which is division-free and DSP-friendly.

**Anti-oscillation**: Maintains a history of the last 4 waypoints. Candidates that match a recent waypoint (within `AVD_OSCILLATION_THRESH = 1.0 deg`) are skipped.

**Greedy escalation**: If the greedy planner selects 8 waypoints (`AVD_GREEDY_LIMIT`) without achieving `AVD_OK_DIRECT`, it triggers the A\* fallback.

### A\* on Precomputed Graph (Stage 7)

When greedy fails (oscillation or exhaustion), the algorithm falls back to A\* search on the precomputed visibility graph:

1. **Snap to graph**: `nearest_node()` finds the closest graph node to current position and target by Manhattan distance. Cost: ~240 ops.
2. **A\* search**: Pure array lookups on the precomputed adjacency matrix. Zero geometry at runtime. Cost: ~26,000 ops.
3. **Cache result**: The path is cached and followed in subsequent calls (Stage 4) until the target changes or the path becomes invalid.

```
Runtime cost (one-time, then cached):
  Find 2 nearest nodes: 2 x V x 3 ops     = ~480 ops
  A* search:            V x V bit-lookups   = ~6,400 ops
  Manhattan costs:      V x V x 3 ops       = ~19,200 ops
  Total:                                     ~26,000 ops
  At 300 MHz DSP:                            ~0.09 ms
```

**Waypoint advance guard**: When following a cached A\* path, the algorithm only advances to the next waypoint when the path to it is clear from the current position. This prevents corner-clipping failures at zone boundaries.

### Mathematical Foundations

| Method | Purpose |
|--------|---------|
| **Liang-Barsky parametric clipping** | O(1) line-segment vs AABB intersection test |
| **Visibility graph theorem** | Shortest path through obstacle vertices lies on the visibility graph |
| **Manhattan (L1) norm** | Division-free distance metric, suitable for fixed-point DSP |
| **Greedy best-first heuristic** | Fast path selection with f = g + h scoring |
| **State-machine anti-oscillation** | Committed waypoint until physically reached |
| **Bit-packed adjacency matrix** | O(1) edge lookup: `adj[i][j/8] >> (j%8) & 1` |
| **Wrap-aware geometry** | AZ boundary crossing splits segments at +/-180 |

### Motion Profiles

The algorithm supports three motion profiles that model how the lower-level controller moves:

- `AVD_MOTION_LINEAR` - Simultaneous AZ-EL diagonal movement (checks diagonal + both tails)
- `AVD_MOTION_AZ_THEN_EL` - L-shape: AZ first, then EL
- `AVD_MOTION_EL_THEN_AZ` - L-shape: EL first, then AZ

## Project Structure

```
yasakAlan/
├── src/
│   ├── avoidance.h          # Public API, types, constants
│   └── avoidance.c          # Core algorithm (~850 lines)
├── test/
│   └── runner_main.c        # Test harness with simulation loop
├── forbidenZoneGenerate.py  # Random forbidden zone generator
├── plot_path.py             # Matplotlib path visualizer
├── run_100_tests.py         # Batch test runner (100 random configs)
├── build.bat                # Windows build script
├── Makefile                 # Unix/MinGW build
└── README.md
```

## API Reference

### Types

```c
typedef float avd_real;         // Numeric type (change to int32_t for fixed-point)

typedef struct {
    avd_real az_min, az_max;
    avd_real el_min, el_max;
} AvdRect;                      // Axis-aligned rectangle

typedef struct { ... } AvdGraph; // Precomputed visibility graph (~3 KB)
typedef struct { ... } AvdInput; // Current position + target + zones
typedef struct { ... } AvdOutput;// Next position + status
typedef struct { ... } AvdState; // Internal state (persists between calls)

typedef enum {
    AVD_OK_DIRECT   = 0,        // Direct path clear
    AVD_OK_WAYPOINT = 1,        // Following intermediate waypoint
    AVD_NO_PATH     = 2         // No safe path, holding position
} AvdStatus;
```

### Functions

```c
// PRE-OP: Build visibility graph (call once, no deadline)
void Avoidance_BuildGraph(AvdGraph *graph,
                          const AvdRect *forbidden, int forbidden_count,
                          const AvdRect *envelope,
                          AvdMotionProfile profile, int az_wrap);

// Initialize state (call once before first Step)
void Avoidance_Init(AvdState *state, AvdMotionProfile profile, int az_wrap);

// OP MODE: Execute one planning cycle (call every 1 ms)
AvdOutput Avoidance_Step(const AvdInput *in, AvdState *state,
                         const AvdGraph *graph);
```

### Usage Pattern

```c
// --- PRE-OP (once) ---
AvdGraph graph;
AvdState state;

Avoidance_BuildGraph(&graph, forbidden_zones, zone_count,
                     &envelope, AVD_MOTION_LINEAR, /*az_wrap=*/1);
Avoidance_Init(&state, AVD_MOTION_LINEAR, /*az_wrap=*/1);

// --- OP MODE (every 1 ms) ---
AvdInput input;
input.az_now = current_az;
input.el_now = current_el;
input.az_cmd = target_az;
input.el_cmd = target_el;
input.envelope = envelope;
memcpy(input.forbidden, forbidden_zones, sizeof(forbidden_zones));
input.forbidden_count = zone_count;

AvdOutput out = Avoidance_Step(&input, &state, &graph);
// Send out.az_next, out.el_next to servo controller
```

## Configuration Constants

| Constant | Default | Description |
|----------|---------|-------------|
| `AVD_MAX_FORBIDDEN` | 16 | Maximum number of forbidden rectangles |
| `AVD_CORNER_EPS` | 0.1 | Waypoint offset from rectangle corners (degrees) |
| `AVD_WP_REACH_THRESH` | 0.3 | Waypoint "reached" threshold, L1 distance (degrees) |
| `AVD_CANDIDATES_PER_RECT` | 8 | Waypoint candidates per rectangle (4 corners + 4 midpoints) |
| `AVD_HISTORY_SIZE` | 4 | Waypoint history slots for oscillation detection |
| `AVD_GREEDY_LIMIT` | 8 | Max greedy waypoints before A\* escalation |
| `AVD_OSCILLATION_THRESH` | 1.0 | Threshold to detect repeated waypoints (degrees) |
| `AVD_MAX_PATH_LEN` | 48 | Maximum waypoints in A\* planned path |
| `AVD_MAX_FIXED_NODES` | 128 | Maximum graph nodes (16 zones x 8 candidates) |

## Build

### Prerequisites

- GCC with C99 support
- Python 3 with matplotlib (for visualization only)

### Windows (build.bat)

```bat
build.bat
```

### Unix / MinGW (Makefile)

```bash
make          # build
make run      # build and run
make clean    # remove build artifacts
```

### Manual Build

```bash
gcc -O2 -Wall -Wextra -pedantic -std=c99 -Isrc \
    src/avoidance.c test/runner_main.c -o test_runner.exe
```

## Run Tests

### Single Test

```bash
# Build and run
./test_runner.exe

# Output goes to both console and forbidden_zones_c.txt
```

The test harness simulates servo movement at a configurable rate, checks safety invariants every step, and reports PASS/FAIL:

```
======================================
  T1: No forbidden zones, wrap (OK_DIRECT expected)
======================================
  Envelope: AZ[-179.9,180.0]  EL[-30.0,60.0]  (az_wrap=1)
  Forbidden zones: 16
  Precomputed graph: 80 nodes, 1280 bytes adjacency
  Start: (-30.00, 55.00)  Target: (160.00, 0.00)
--------------------------------------
  [  0] pos=( -30.00,  55.00) cmd=( 160.00,   0.00) -> next=(-150.10,  50.10) OK_WAYPOINT
  ...
  >> Target reached at step 305
--------------------------------------
  RESULT: PASS  (0 violations, 306 steps)
```

### Visualize Results

```bash
# Plot the last test output
python plot_path.py forbidden_zones_c.txt
```

Generates a plot showing the envelope, forbidden zones, trajectory with direction arrows, waypoint commands, start/target markers.

### Batch Testing (100 Random Configurations)

```bash
python run_100_tests.py
```

For each iteration:
1. Generates 16 random non-overlapping forbidden zones
2. Picks random start/target positions outside zones
3. Updates `runner_main.c`, builds, runs, and plots
4. Reports PASS/FAIL summary

### Generate Custom Forbidden Zones

```bash
python forbidenZoneGenerate.py
```

Generates 16 random non-overlapping forbidden zones with configurable:
- AZ/EL size ranges
- Edge-touch probability (zones snapping to EL envelope boundaries)
- Output as C initialization code

## Design Constraints

- **C99 compatible** - No C11/C++ features
- **No dynamic memory** - All arrays fixed-size, stack-allocated
- **No math.h dependency** - Inline abs/min/max/clamp helpers
- **Fixed-point convertible** - `avd_real` typedef can be changed to `int32_t` with Q-format
- **Deterministic** - No random behavior, no floating-point non-determinism concerns
- **Thread-safe** - No global state; all state in caller-provided structs

## Performance Summary

| Operation | Complexity | Typical Time (300 MHz DSP) |
|-----------|------------|---------------------------|
| `Avoidance_BuildGraph` | O(V^2 x N) | 1-10 ms (once, no deadline) |
| `Avoidance_Step` (direct) | O(N) | ~0.01 ms |
| `Avoidance_Step` (greedy) | O(V x N) | ~0.03 ms |
| `Avoidance_Step` (A\* fallback) | O(V^2) | ~0.09 ms (once, then cached) |
| `Avoidance_Step` (cached path) | O(N) | ~0.01 ms |

Where V = graph nodes (~80), N = forbidden zones (up to 16).
