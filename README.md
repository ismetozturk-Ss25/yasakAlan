# Forbidden-Zone Avoidance Command Shaper

Real-time path planner for turret/antenna systems that routes around axis-aligned rectangular forbidden zones in AZ-EL (Azimuth-Elevation) space. Designed for DSP deployment at 1 kHz (1 ms cycle time) and Simulink S-Function integration.

## Problem

A turret or antenna must move from its current position to a commanded target in AZ-EL space, but certain rectangular regions are **forbidden** (e.g., sun avoidance, mechanical limits, radar exclusion zones). The system must:

- Never enter a forbidden zone
- Stay inside the working envelope
- Find the shortest safe path in real-time (1 ms per cycle)
- Handle up to 32 simultaneous forbidden zones
- Support AZ wrap-around at the +/-180 degree boundary

```
 EL 60 +----------+--------+----------+--------+
       |          | FORBID |          | FORBID |
       |  START   |  ZONE  |          |  ZONE  |
       |    *     |########|  (path)  |########|   * TARGET
       |          |########|  ------->|########|
EL -30 +----------+--------+----------+--------+
     AZ -180                                  AZ +180
```

## Architecture: Two-Phase Design

The algorithm is split into two independent C files that share a single header (`avoidance_simulink.h`). Each file compiles independently with zero dependencies on the other.

```
avoidance_simulink.h    <- types, constants, 12 inline helpers
       |          |
       v          v
avoidance_preop.c    avoidance_opmode.c
 (Phase 1: graph)    (Phase 2: real-time step)
```

---

## Phase 1: PRE-OP Mode — `avoidance_preop.c`

**File:** `src/avoidance_preop.c` (~320 lines)
**Entry point:** `Avoidance_BuildGraph()`
**When called:** Once, when forbidden zones are defined. No real-time deadline.
**Purpose:** Build a visibility graph so that all expensive geometry (O(V^2 x N) intersection tests) is done up front, leaving only array lookups for the real-time step.

### How `Avoidance_BuildGraph()` works, step by step:

#### Step 0: Wrap-Around Normalization

When `az_wrap=1` and the envelope or any forbidden zone wraps around the +/-180 boundary (i.e., `az_min > az_max`), the function creates local copies of all zones and the envelope, then normalizes them:

1. **Wrap-around envelope** (e.g., `az_min=170, az_max=-170`):
   - The "gap" `[az_max, az_min]` = `[-170, 170]` is added as a new forbidden zone (it's the region outside the envelope)
   - The envelope is expanded to `[-180, +180]`

2. **Wrap-around forbidden zones** (e.g., `az_min=170, az_max=-160`):
   - Split into two non-wrapping zones: `[170, 180]` and `[-180, -160]`
   - This is necessary because the Liang-Barsky intersection test requires non-wrapping rectangles

After this step, all rectangles have `az_min <= az_max`, making all subsequent geometry straightforward.

#### Step 1: Generate Candidate Nodes

For each valid forbidden zone, 8 candidate waypoint positions are generated:

```
         az_mid
           |
   [3]---[5]---[2]      [0..3] = 4 corners (offset by CORNER_EPS)
    |      |      |      [4..7] = 4 edge midpoints (offset by CORNER_EPS)
   [6]   ZONE   [7]
    |      |      |      CORNER_EPS = 0.1 deg ensures nodes sit
   [0]---[4]---[1]      just outside the zone boundary.
```

- Corner nodes: `(az_min - eps, el_min - eps)`, `(az_hi + eps, el_lo - eps)`, etc.
- Edge midpoints: `(az_mid, el_lo - eps)`, `(az_mid, el_hi + eps)`, `(az_lo - eps, el_mid)`, `(az_hi + eps, el_mid)`

Each candidate is **filtered**:
- Nodes outside the envelope are **skipped** (not clamped, because clamping would put nodes on zone boundaries causing false adjacencies)
- Nodes inside any forbidden zone are skipped

#### Step 2: Add Sweep Nodes

To guarantee graph connectivity across long AZ spans where zone corners may be far apart, additional "sweep" nodes are placed at regular AZ intervals:

- AZ step = `envelope_az_span / 5`, clamped to `[5, 60]` degrees
- At each AZ step, 3 nodes are placed: bottom edge, top edge, and midpoint of the envelope's EL range
- Nodes inside any forbidden zone are skipped
- Duplicate suppression: nodes within 1.0 deg Manhattan distance of an existing node are skipped

#### Step 3: Compute All-Pairs Visibility (Adjacency Matrix)

For every pair of nodes `(i, j)`, a `path_is_clear()` check is performed:

```
For each pair (i, j) where i < j:
    if path_is_clear(node_i, node_j, forbidden_zones, motion_profile, az_wrap):
        adj[i][j] = 1
        adj[j][i] = 1    (symmetric)
```

`path_is_clear()` respects the motion profile:
- **LINEAR**: checks one diagonal segment
- **AZ_THEN_EL**: checks two segments (horizontal then vertical L-shape)
- **EL_THEN_AZ**: checks two segments (vertical then horizontal L-shape)

Each segment check uses **Liang-Barsky parametric clipping** against every forbidden zone. For AZ-wrap scenarios, segments crossing the +/-180 boundary are split at the boundary and each half is checked independently.

The adjacency matrix is **bit-packed**: `adj[i][j/8] & (1 << (j%8))`, requiring only 32 bytes per node (256 nodes / 8 bits). Total adjacency storage: 256 x 32 = 8 KB.

#### Output

```c
AvdGraph {
    int nc;                           // node count (typically 40-120)
    float naz[256];                   // node AZ coordinates
    float nel[256];                   // node EL coordinates
    unsigned char adj[256][32];       // bit-packed adjacency matrix
}
```

```
Complexity: O(V^2 x N)  where V <= 256 nodes, N <= 32 zones
Typical time: 1-10 ms (no deadline, called once)
Memory: ~10 KB (256 nodes x 2 coords x 4 bytes + 256 x 32 adjacency bytes)
```

---

## Phase 2: OP Mode — `avoidance_opmode.c`

**File:** `src/avoidance_opmode.c` (~570 lines)
**Entry points:** `Avoidance_Init()`, `Avoidance_Step()`
**When called:** `Init` once at startup, then `Step` every 1 ms during operation.
**Purpose:** Real-time path planning using the precomputed graph. A\* is the primary planner; direct-path is a shortcut when the way is clear.

### Internal Helper Functions

The opmode file contains 4 static helper functions:

| Function | Purpose |
|----------|---------|
| `project_target()` | Push a target position out of any forbidden zone to the nearest edge |
| `try_escape()` | Emergency: if current position is inside a forbidden zone, find the nearest safe exit |
| `nearest_reachable_node()` | Find the graph node closest to a given point that has a clear path from that point |
| `astar_on_graph()` | A\* search on the precomputed graph using only array lookups (zero geometry) |

### How `Avoidance_Step()` works, step by step:

Each call to `Avoidance_Step()` executes a 5-step pipeline. It returns as soon as any step produces an output.

#### Step 0: Wrap-Around Normalization (same as PreOp)

Identical logic to PreOp. When `az_wrap=1`, local copies of the envelope and forbidden zones are created with wrapped rectangles split at +/-180.

#### Step 1: Emergency Escape

```c
if (current_position is inside a forbidden zone):
    try 4 escape directions: left, right, bottom, top edge of the zone
    sort by distance (nearest first)
    for each candidate:
        clamp to envelope
        skip if outside envelope or inside another zone
        return first valid escape point -> status = OK_WAYPOINT
```

The `try_escape()` function handles the rare case where a position update or zone change leaves the current position inside a zone. It uses insertion sort on 4 elements to find the nearest valid exit.

#### Step 2: Clamp + Project Target

```c
target_az = clamp(commanded_az, envelope.az_min, envelope.az_max)
target_el = clamp(commanded_el, envelope.el_min, envelope.el_max)
if (az_wrap) normalize to [-180, 180]
project_target(&target_az, &target_el, forbidden_zones, envelope, az_wrap)
```

`project_target()` checks if the (clamped) target falls inside a forbidden zone. If so, it computes the distance to all 4 edges and projects the target to the nearest edge (offset by `CORNER_EPS`), then clamps back to the envelope.

#### Step 3: Direct Path Check

```c
if (astar_lock <= 0 AND path_is_clear(current, target)):
    output = target
    status = OK_DIRECT
    clear path cache
    return
```

The cheapest case: if there is a clear line of sight to the target (respecting the motion profile), go straight there. The `astar_lock` prevents this shortcut when anti-oscillation is active (see below).

#### Step 4: A\* Path Planning

This is the core planner. It has three sub-steps:

**4a. Cache Invalidation:**
```c
if (cached path exists AND target moved > AVD_TGT_MOVE_THRESH = 2.0 deg):
    invalidate cache
```

**4b. Compute New A\* Path:**
```c
if (no valid cached path AND graph available):
    src = nearest_reachable_node(current_position)   // O(V) with path_is_clear checks
    dst = nearest_reachable_node(target_position)     // O(V) with path_is_clear checks
    path = astar_on_graph(src, dst)                   // O(V^2) pure array lookups
    prepend src node to path (so servo first reaches the graph entry point)
    cache the path
```

`nearest_reachable_node()` is critical: it finds the closest graph node that has a **clear path** from the given position. This ensures the first waypoint is always reachable. It falls back to closest-by-distance if no reachable node exists.

`astar_on_graph()` performs standard A\* using:
- Manhattan distance as the heuristic (admissible for L1)
- Bit-packed adjacency lookups (single bit-shift + AND per edge query)
- All arrays on the stack (fixed size: `g_cost[256]`, `f_cost[256]`, `parent[256]`, `closed[256]`)
- Zero geometry at runtime: all visibility was precomputed

**4c. Follow Cached Path:**
```c
while (following cached path):
    if (current waypoint reached within WP_REACH_THRESH = 0.3 deg):
        advance to next waypoint
        if (path exhausted): invalidate, fall through

    // Shortcut: try direct to target at every waypoint
    if (astar_lock <= 0 AND path_is_clear(current, target)):
        return target -> OK_DIRECT

    // Verify waypoint still reachable (zones may have changed)
    if (path_is_clear(current, waypoint) AND waypoint in envelope AND waypoint not in forbidden):
        return waypoint -> OK_WAYPOINT
    else:
        invalidate path, recompute
```

Key design decisions:
- **Src prepend**: The A\* path excludes the source node, but the servo must first reach the graph entry point. So `src` is prepended to the path.
- **Shortcut attempts**: At every waypoint, the algorithm checks if a direct path to the target has opened up. This avoids following the full graph path when a shortcut becomes available.
- **Waypoint validation**: Each waypoint is re-verified before use. If a waypoint became unreachable (e.g., dynamic zone changes), the path is invalidated and recomputed.
- **Double A\* attempt**: If the cached path becomes invalid during step 4c, a second A\* computation is tried immediately rather than waiting for the next call.

#### Step 5: No Path Found

```c
output = current_position   // hold position
status = AVD_NO_PATH
```

Last resort: if no escape, no direct path, and no A\* path exists, the system holds its current position.

### Anti-Oscillation Mechanism

The `avd_update_osc()` function detects rapid `DIRECT <-> WAYPOINT` status flips that can occur when the direct path is borderline clear/blocked:

```
Every step:
    if (status flipped between DIRECT and WAYPOINT):
        osc_count++
        if (osc_count >= AVD_OSC_THRESH = 3):
            astar_lock = AVD_LOCK_DURATION = 50 steps (50 ms at 1 kHz)
            osc_count = 0
    else:
        osc_count = 0

When astar_lock > 0:
    - Direct path shortcut is DISABLED (Step 3 and 4c shortcut both skip)
    - Forces the system to commit to A* waypoints
    - Timer decrements each step until it reaches 0
```

This prevents the output from jittering between direct and waypoint modes.

---

## Shared Header — `avoidance_simulink.h`

**File:** `src/avoidance_simulink.h` (~380 lines)
**Purpose:** Single header containing everything needed by both `.c` files. No other headers are needed (no `<math.h>` dependency).

### Contents:

**Section 1: Configuration Constants**

| Constant | Value | Description |
|----------|-------|-------------|
| `AVD_MAX_FORBIDDEN` | 32 | Maximum forbidden rectangles |
| `AVD_CORNER_EPS` | 0.1 | Node offset from zone corners (degrees) |
| `AVD_WP_REACH_THRESH` | 0.3 | Waypoint "reached" threshold, L1 (degrees) |
| `AVD_CANDIDATES_PER_RECT` | 8 | Nodes per zone: 4 corners + 4 midpoints |
| `AVD_MAX_PATH_LEN` | 48 | Maximum waypoints in A\* path |
| `AVD_TGT_MOVE_THRESH` | 2.0 | Target movement to invalidate cache (degrees) |
| `AVD_OSC_THRESH` | 3 | DIRECT/WAYPOINT flips before A\* lock |
| `AVD_LOCK_DURATION` | 50 | A\* lock duration (steps) |
| `AVD_MAX_FIXED_NODES` | 256 | Maximum graph nodes (32 x 8) |
| `AVD_ADJ_BYTES` | 32 | Adjacency row width (256/8) |

**Section 2: 12 Static Inline Helper Functions**

Each `.c` file gets its own copy (no linker issues).

| Function | Description |
|----------|-------------|
| `avd_abs(v)` | Absolute value |
| `avd_min(a,b)` | Minimum |
| `avd_max(a,b)` | Maximum |
| `avd_clamp(v,lo,hi)` | Clamp to range |
| `avd_normalize_az(az)` | Normalize AZ to (-180, +180] |
| `avd_manhattan(ax,ay,bx,by,wrap)` | Manhattan distance, wrap-aware |
| `point_in_rect(az,el,r)` | Point inside rectangle (exclusive boundaries) |
| `point_in_any_forbidden(az,el,f)` | Point inside any valid forbidden zone |
| `point_in_envelope(az,el,env)` | Point inside envelope (inclusive boundaries) |
| `segment_hits_rect(p0,p1,r)` | Liang-Barsky segment-AABB intersection |
| `segment_is_clear(p0,p1,f,wrap)` | Segment avoids all zones (wrap-aware, splits at +/-180) |
| `path_is_clear(p0,p1,f,prof,wrap)` | Motion-profile-aware path clearance check |

**Section 3: Function Declarations**

```c
void Avoidance_BuildGraph(AvdGraph *graph, const AvdRect *forbidden,
                          const AvdRect *envelope,
                          AvdMotionProfile profile, int az_wrap);

void Avoidance_Init(AvdState *state, AvdMotionProfile profile, int az_wrap);

AvdOutput Avoidance_Step(const AvdInput *in, AvdState *state,
                         const AvdGraph *graph);
```

---

## Simulink Integration

The code is designed as two Simulink C S-Function blocks:

### Avoidance_PreOp Block (runs once)

```
INPUTS:                              OUTPUTS:
  Port 1: forbidden[32x5]             Port 1: graph_nc (int)
  Port 2: envelope[4]                 Port 2: graph_naz[256]
  Port 3: motion_profile (int 0/1/2)  Port 3: graph_nel[256]
  Port 4: az_wrap (int 0/1)           Port 4: graph_adj[256x32]
```

### Avoidance_OpMode Block (runs every 1 ms)

```
INPUTS:                              OUTPUTS:
  Port  1: az_now                      Port 1: az_next
  Port  2: el_now                      Port 2: el_next
  Port  3: az_cmd                      Port 3: status (0/1/2)
  Port  4: el_cmd
  Port  5: envelope[4]              PERSISTENT STATE (DWork):
  Port  6: forbidden[32x5]            AvdState (~600 bytes)
  Port  7: graph_nc                    init via Avoidance_Init()
  Port  8: graph_naz[256]
  Port  9: graph_nel[256]
  Port 10: graph_adj[256x32]
```

---

## Data Types

```c
typedef float avd_real;             // Change to int32_t + Q-format for fixed-point

typedef struct {
    int      valid;                 // 1=active, 0=unused
    avd_real az_min, el_min;        // lower-left corner
    avd_real az_max, el_max;        // upper-right corner
} AvdRect;

typedef struct {
    int       nc;                   // node count
    avd_real  naz[256], nel[256];   // node coordinates
    unsigned char adj[256][32];     // bit-packed adjacency
} AvdGraph;

typedef struct {
    avd_real az_now, el_now;        // current position
    avd_real az_cmd, el_cmd;        // commanded target
    AvdRect  envelope;              // working envelope
    AvdRect  forbidden[32];         // forbidden zones
} AvdInput;

typedef struct {
    avd_real  az_next, el_next;     // next position to command
    AvdStatus status;               // OK_DIRECT / OK_WAYPOINT / NO_PATH
} AvdOutput;

typedef enum {
    AVD_OK_DIRECT   = 0,            // Direct path clear
    AVD_OK_WAYPOINT = 1,            // Following A* waypoint
    AVD_NO_PATH     = 2             // No safe path, holding position
} AvdStatus;

typedef enum {
    AVD_MOTION_LINEAR     = 0,      // Diagonal movement
    AVD_MOTION_AZ_THEN_EL = 1,     // L-shape: AZ first, then EL
    AVD_MOTION_EL_THEN_AZ = 2      // L-shape: EL first, then AZ
} AvdMotionProfile;
```

### Usage Pattern

```c
// --- PRE-OP (once) ---
AvdGraph graph;
AvdState state;

Avoidance_BuildGraph(&graph, forbidden_zones, &envelope,
                     AVD_MOTION_LINEAR, /*az_wrap=*/1);
Avoidance_Init(&state, AVD_MOTION_LINEAR, /*az_wrap=*/1);

// --- OP MODE (every 1 ms) ---
AvdInput input;
input.az_now = current_az;
input.el_now = current_el;
input.az_cmd = target_az;
input.el_cmd = target_el;
input.envelope = envelope;
memcpy(input.forbidden, forbidden_zones, sizeof(forbidden_zones));

AvdOutput out = Avoidance_Step(&input, &state, &graph);
// Send out.az_next, out.el_next to servo controller
```

---

## Mathematical Foundations

| Method | Purpose |
|--------|---------|
| **Liang-Barsky parametric clipping** | O(1) line-segment vs AABB intersection test |
| **Visibility graph theorem** | Shortest obstacle-avoiding path lies on the visibility graph |
| **Manhattan (L1) norm** | Division-free distance metric, suitable for fixed-point DSP |
| **A\* search with Manhattan heuristic** | Optimal path on precomputed graph; heuristic is admissible for L1 |
| **Bit-packed adjacency matrix** | O(1) edge lookup: `adj[i][j/8] >> (j%8) & 1` |
| **Wrap-aware segment splitting** | AZ boundary crossing splits segments at +/-180 for correct Liang-Barsky |
| **Anti-oscillation lock** | DIRECT/WAYPOINT flip detection with timed A\* commitment |

### Motion Profiles

The algorithm supports three motion profiles that model how the lower-level controller moves between two points:

- `AVD_MOTION_LINEAR` (0) - Simultaneous AZ-EL diagonal movement. Checks one diagonal segment.
- `AVD_MOTION_AZ_THEN_EL` (1) - L-shape: AZ first, then EL. Checks horizontal segment, then vertical segment.
- `AVD_MOTION_EL_THEN_AZ` (2) - L-shape: EL first, then AZ. Checks vertical segment, then horizontal segment.

## Project Structure

```
yasakAlan/
├── src/
│   ├── avoidance_simulink.h    # Types, constants, 12 inline helpers, declarations
│   ├── avoidance_preop.c       # Phase 1: visibility graph builder
│   └── avoidance_opmode.c      # Phase 2: real-time A* step function
├── Ccode_yasakAlan/
│   ├── sfun_avoidance_preop.c  # Simulink S-Function wrapper for PreOp
│   └── sfun_avoidance_opmode.c # Simulink S-Function wrapper for OpMode
├── test/
│   └── runner_main.c           # Test harness with simulation loop
├── forbidenZoneGenerate.py     # Random forbidden zone generator
├── plot_path.py                # Matplotlib path visualizer
├── run_100_tests.py            # Batch test runner (100 random configs)
├── build.bat                   # Windows build script
├── Makefile                    # Unix/MinGW build
└── README.md
```

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
    src/avoidance_preop.c src/avoidance_opmode.c test/runner_main.c \
    -o test_runner.exe
```

Enable debug output (prints zones, nodes, adjacency to console and `preop_debug.txt`):
```bash
gcc -O2 -DAVD_DEBUG_PRINT -Isrc \
    src/avoidance_preop.c src/avoidance_opmode.c test/runner_main.c \
    -o test_runner.exe
```

## Design Constraints

- **C99 compatible** - No C11/C++ features
- **No dynamic memory** - All arrays fixed-size, stack-allocated
- **No math.h dependency** - Inline abs/min/max/clamp helpers
- **Fixed-point convertible** - `avd_real` typedef can be changed to `int32_t` with Q-format
- **Deterministic** - No random behavior, no floating-point non-determinism concerns
- **Thread-safe** - No global state; all state in caller-provided structs
- **Two independent compilation units** - `avoidance_preop.c` and `avoidance_opmode.c` have zero dependencies on each other

## Performance Summary

| Operation | Complexity | Typical Time (300 MHz DSP) |
|-----------|------------|---------------------------|
| `Avoidance_BuildGraph` | O(V^2 x N) | 1-10 ms (once, no deadline) |
| `Avoidance_Step` (direct path clear) | O(N) | ~0.01 ms |
| `Avoidance_Step` (A\* compute) | O(V^2) | ~0.09 ms (one-time, then cached) |
| `Avoidance_Step` (cached path follow) | O(N) | ~0.01 ms |
| `Avoidance_Step` (escape) | O(N) | ~0.01 ms |
| `Avoidance_Step` (no path) | O(1) | ~0.001 ms |

Where V = graph nodes (typically 40-120), N = forbidden zones (up to 32).
