/*
 * Forbidden-Area Avoidance Command Shaper
 * ========================================
 *
 * Real-time path planner for turret/antenna systems.
 * Routes around axis-aligned rectangular forbidden zones in AZ-EL space.
 *
 * Architecture (two-phase):
 *
 *   PRE-OP  (time unconstrained):
 *     Avoidance_BuildGraph()  — builds visibility graph from forbidden zones.
 *     Precomputes all node-to-node visibility → bit-packed adjacency matrix.
 *     Memory: ~3 KB.  Called once when forbidden zones are defined.
 *
 *   OP MODE (1 ms deadline):
 *     Avoidance_Step()  — called every 1 ms.
 *     1. Greedy one-step planner (fast, ~2 700 ops)
 *     2. If greedy oscillates → A* on precomputed graph
 *        - Connect start/goal to graph: 2 × V × N segment checks  (~0.4 ms)
 *        - A* search: array lookups only, no geometry              (~0.05 ms)
 *        - Result cached — subsequent calls just follow waypoints  (~0 ms)
 *
 * Mathematical foundations
 * -----------------------
 *  1. Liang-Barsky parametric clipping   — O(1) line-AABB intersection
 *  2. Visibility-graph theorem           — shortest path through obstacle vertices
 *  3. L1 (Manhattan) norm               — division-free path cost metric
 *  4. Greedy best-first heuristic        — f(w) = g(w) + h(w), no open-set
 *  5. State-machine anti-oscillation     — committed waypoint until reached
 *  6. Precomputed adjacency matrix       — O(1) edge lookup during A*
 *
 * Designed for DSP deployment at 1 kHz (1 ms period).
 * C99 compatible, no dynamic memory, fixed-point convertible.
 */

#ifndef AVOIDANCE_H
#define AVOIDANCE_H

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  CONFIGURATION CONSTANTS
 * ================================================================ */
#define AVD_MAX_FORBIDDEN      16      /* max forbidden rectangles          */
#define AVD_CORNER_EPS         0.1f    /* waypoint offset from rect corners */
#define AVD_WP_REACH_THRESH    0.3f    /* waypoint "reached" threshold (L1) */
#define AVD_CANDIDATES_PER_RECT 8      /* 4 corners + 4 edge midpoints      */
#define AVD_HISTORY_SIZE       4       /* waypoint history for oscillation detection */
#define AVD_GREEDY_LIMIT       8       /* max greedy waypoints before A* escalation  */
#define AVD_OSCILLATION_THRESH 1.0f    /* threshold to consider waypoints "same" */
#define AVD_MAX_PATH_LEN       48      /* max waypoints in planned path          */

/* Graph sizing */
#define AVD_MAX_FIXED_NODES    (AVD_MAX_FORBIDDEN * AVD_CANDIDATES_PER_RECT) /* 128 */
#define AVD_MAX_GRAPH_NODES    (AVD_MAX_FIXED_NODES + 2)  /* +start +goal = 130 */
#define AVD_ADJ_BYTES          ((AVD_MAX_FIXED_NODES + 7) / 8)              /* 16  */

/* ================================================================
 *  NUMERIC TYPE  (change to int32_t + Q-format for fixed-point)
 * ================================================================ */
typedef float avd_real;

/* ================================================================
 *  ENUMERATIONS
 * ================================================================ */

/* Output status */
typedef enum {
    AVD_OK_DIRECT   = 0,   /* direct path clear, output = target         */
    AVD_OK_WAYPOINT = 1,   /* following intermediate waypoint             */
    AVD_NO_PATH     = 2    /* no safe path; holding current position      */
} AvdStatus;

/* Assumed motion profile of the lower-level controller */
typedef enum {
    AVD_MOTION_LINEAR     = 0,  /* simultaneous AZ-EL diagonal          */
    AVD_MOTION_AZ_THEN_EL = 1,  /* L-shape: AZ first, then EL           */
    AVD_MOTION_EL_THEN_AZ = 2   /* L-shape: EL first, then AZ           */
} AvdMotionProfile;

/* ================================================================
 *  DATA STRUCTURES
 * ================================================================ */

/* Forbidden zone in AZ-EL space (matches MATLAB 16x5 matrix row).
 * Field order: valid, az_min, el_min, az_max, el_max */
typedef struct {
    int      valid;                /* 1 = active zone, 0 = unused row    */
    avd_real az_min, el_min;       /* lower-left corner                  */
    avd_real az_max, el_max;       /* upper-right corner                 */
} AvdRect;

/* Precomputed visibility graph — built once in pre-op mode.
 * Memory: ~3 KB (128 nodes × 2 coords × 4 bytes + 128×16 adjacency) */
typedef struct {
    int       nc;                                            /* number of fixed nodes (<=128) */
    avd_real  naz[AVD_MAX_FIXED_NODES];                     /* node AZ positions             */
    avd_real  nel[AVD_MAX_FIXED_NODES];                     /* node EL positions             */
    unsigned char adj[AVD_MAX_FIXED_NODES][AVD_ADJ_BYTES];  /* bit-packed adjacency matrix   */
} AvdGraph;

/* Algorithm input — provided every 1 ms by upper software */
typedef struct {
    avd_real  az_now, el_now;                       /* current position      */
    avd_real  az_cmd, el_cmd;                       /* commanded target      */
    AvdRect   envelope;                              /* working envelope      */
    AvdRect   forbidden[AVD_MAX_FORBIDDEN];          /* forbidden zones (16x5)*/
} AvdInput;

/* Algorithm output */
typedef struct {
    avd_real   az_next, el_next;   /* next position to command              */
    AvdStatus  status;             /* result code                           */
} AvdOutput;

/* Internal state — small, fixed-size, persists between calls */
typedef struct {
    int              active;        /* 0 = idle, 1 = following waypoint     */
    avd_real         wp_az, wp_el;  /* committed waypoint (when active = 1) */
    AvdMotionProfile profile;       /* motion profile assumption            */
    int              az_wrap;       /* 1 = AZ wraps at ±180                 */
    /* Oscillation detection history */
    avd_real         hist_az[AVD_HISTORY_SIZE];
    avd_real         hist_el[AVD_HISTORY_SIZE];
    int              hist_idx;
    int              hist_count;
    /* A* cached path (from precomputed-graph search) */
    avd_real         path_az[AVD_MAX_PATH_LEN];
    avd_real         path_el[AVD_MAX_PATH_LEN];
    int              path_len;
    int              path_idx;
    int              path_valid;
    avd_real         path_tgt_az;
    avd_real         path_tgt_el;
    /* Greedy escalation counter */
    int              greedy_wp_count;
} AvdState;

/* ================================================================
 *  PUBLIC API
 * ================================================================ */

/*
 * Avoidance_BuildGraph
 *   Precomputes the visibility graph from forbidden zones.
 *   Call ONCE in pre-op mode (time unconstrained).
 *   Forbidden zones must NOT change after this call.
 *
 *   Computes: ~V^2 × N segment checks (V<=128, N<=16).
 *   Typical time: 1-10 ms on DSP, no deadline.
 *   forbidden[16] — zones with valid flag, always 16 entries.
 */
void Avoidance_BuildGraph(AvdGraph *graph,
                          const AvdRect *forbidden,
                          const AvdRect *envelope,
                          AvdMotionProfile profile, int az_wrap);

/*
 * Avoidance_Init
 *   Resets internal state. Must be called once before the first Step.
 */
void Avoidance_Init(AvdState *state, AvdMotionProfile profile, int az_wrap);

/*
 * Avoidance_Step
 *   Executes one planning cycle (call every 1 ms).
 *
 *   graph — precomputed visibility graph from Avoidance_BuildGraph.
 *           If NULL, only greedy planner is used (no A* fallback).
 *           If provided, A* fallback uses the precomputed graph.
 */
AvdOutput Avoidance_Step(const AvdInput *in, AvdState *state,
                         const AvdGraph *graph);

#ifdef __cplusplus
}
#endif

#endif /* AVOIDANCE_H */
