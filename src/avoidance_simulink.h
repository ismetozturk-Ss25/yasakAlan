/*
 * avoidance_simulink.h — Single header for Simulink C S-Function import
 *
 * Contains everything needed by both avoidance_preop.c and avoidance_opmode.c:
 *   1. Configuration constants, typedefs, enums, structs
 *   2. Static inline helper functions (12 functions)
 *   3. Function declarations for BuildGraph, Init, Step
 *
 * Forbidden zone format matches MATLAB 16x5 matrix:
 *   Column 1: valid  (1=active, 0=unused)
 *   Column 2: az_min
 *   Column 3: el_min
 *   Column 4: az_max
 *   Column 5: el_max
 *
 * Simulink S-Function Blocks:
 *
 *   "Avoidance_PreOp" — runs ONCE (time unconstrained)
 *   ┌──────────────────────────────────────────────────────────────┐
 *   │  INPUTS:                                                     │
 *   │    Port 1: forbidden[16x5]  (valid,az_min,el_min,           │
 *   │                              az_max,el_max per row)         │
 *   │    Port 2: envelope[4]      (az_min,el_min,az_max,el_max)  │
 *   │    Port 3: motion_profile   (int, 0/1/2)                    │
 *   │    Port 4: az_wrap          (int, 0/1)                      │
 *   │  OUTPUTS:                                                    │
 *   │    Port 1: graph_nc         (int, node count)               │
 *   │    Port 2: graph_naz[128]   (node AZ coords)               │
 *   │    Port 3: graph_nel[128]   (node EL coords)               │
 *   │    Port 4: graph_adj[128x16](adjacency bytes)              │
 *   └──────────────────────────────────────────────────────────────┘
 *
 *   "Avoidance_OpMode" — runs every 1 ms sample step
 *   ┌──────────────────────────────────────────────────────────────┐
 *   │  INPUTS:                                                     │
 *   │    Port  1: az_now, Port 2: el_now                          │
 *   │    Port  3: az_cmd, Port 4: el_cmd                          │
 *   │    Port  5: envelope[4]                                     │
 *   │    Port  6: forbidden[16x5]                                 │
 *   │    Port  7: graph_nc                                        │
 *   │    Port  8: graph_naz[128]                                  │
 *   │    Port  9: graph_nel[128]                                  │
 *   │    Port 10: graph_adj[128x16]                               │
 *   │  OUTPUTS:                                                    │
 *   │    Port 1: az_next  (next AZ command)                       │
 *   │    Port 2: el_next  (next EL command)                       │
 *   │    Port 3: status   (0=DIRECT, 1=WAYPOINT, 2=NO_PATH)      │
 *   │  PERSISTENT STATE (DWork):                                   │
 *   │    AvdState (~600 bytes), init via Avoidance_Init() at t=0  │
 *   └──────────────────────────────────────────────────────────────┘
 *
 * C99 compatible, no dynamic memory, no <math.h>, fixed-point convertible.
 */

#ifndef AVOIDANCE_SIMULINK_H
#define AVOIDANCE_SIMULINK_H

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  SECTION 1: CONFIGURATION CONSTANTS
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
 * Memory: ~3 KB (128 nodes x 2 coords x 4 bytes + 128x16 adjacency) */
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
    int              az_wrap;       /* 1 = AZ wraps at +/-180                */
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
 *  SECTION 2: STATIC INLINE HELPER FUNCTIONS
 *
 *  12 functions used by both preop and opmode .c files.
 *  static inline = each .c gets its own copy, no linker issues.
 *
 *  Math:      avd_abs, avd_min, avd_max, avd_clamp
 *  AZ wrap:   avd_normalize_az, avd_manhattan
 *  Geometry:  point_in_rect, point_in_any_forbidden, point_in_envelope
 *  Segment:   segment_hits_rect, segment_is_clear, path_is_clear
 * ================================================================ */

/* ---------- Math helpers (no <math.h> dependency) ---------- */

static inline avd_real avd_abs(avd_real v)
{
    return v < 0.0f ? -v : v;
}

static inline avd_real avd_min(avd_real a, avd_real b)
{
    return a < b ? a : b;
}

static inline avd_real avd_max(avd_real a, avd_real b)
{
    return a > b ? a : b;
}

static inline avd_real avd_clamp(avd_real v, avd_real lo, avd_real hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ---------- AZ wrap-around helpers ---------- */

static inline avd_real avd_normalize_az(avd_real az)
{
    if (az > 180.0f)  return az - 360.0f;
    if (az <= -180.0f) return az + 360.0f;
    return az;
}

static inline avd_real avd_manhattan(avd_real ax, avd_real ay,
                                     avd_real bx, avd_real by,
                                     int az_wrap)
{
    avd_real daz = avd_abs(ax - bx);
    if (az_wrap && daz > 180.0f) daz = 360.0f - daz;
    return daz + avd_abs(ay - by);
}

/* ---------- Geometric predicates ---------- */

static inline int point_in_rect(avd_real az, avd_real el, const AvdRect *r)
{
    return (az > r->az_min && az < r->az_max &&
            el > r->el_min && el < r->el_max);
}

static inline int point_in_any_forbidden(avd_real az, avd_real el,
                                         const AvdRect *f)
{
    int i;
    for (i = 0; i < AVD_MAX_FORBIDDEN; i++) {
        if (!f[i].valid) continue;
        if (point_in_rect(az, el, &f[i])) return 1;
    }
    return 0;
}

static inline int point_in_envelope(avd_real az, avd_real el, const AvdRect *env)
{
    return (az >= env->az_min && az <= env->az_max &&
            el >= env->el_min && el <= env->el_max);
}

/* ---------- Liang-Barsky line-segment vs AABB intersection ---------- */

static inline int segment_hits_rect(avd_real p0x, avd_real p0y,
                                    avd_real p1x, avd_real p1y,
                                    const AvdRect *r)
{
    avd_real dx = p1x - p0x;
    avd_real dy = p1y - p0y;
    avd_real t_near = 0.0f;
    avd_real t_far  = 1.0f;
    avd_real t1, t2, inv, tmp;

    if (dx > -1e-9f && dx < 1e-9f) {
        if (p0x <= r->az_min || p0x >= r->az_max) return 0;
    } else {
        inv = 1.0f / dx;
        t1  = (r->az_min - p0x) * inv;
        t2  = (r->az_max - p0x) * inv;
        if (t1 > t2) { tmp = t1; t1 = t2; t2 = tmp; }
        t_near = avd_max(t_near, t1);
        t_far  = avd_min(t_far,  t2);
        if (t_near >= t_far) return 0;
    }

    if (dy > -1e-9f && dy < 1e-9f) {
        if (p0y <= r->el_min || p0y >= r->el_max) return 0;
    } else {
        inv = 1.0f / dy;
        t1  = (r->el_min - p0y) * inv;
        t2  = (r->el_max - p0y) * inv;
        if (t1 > t2) { tmp = t1; t1 = t2; t2 = tmp; }
        t_near = avd_max(t_near, t1);
        t_far  = avd_min(t_far,  t2);
        if (t_near >= t_far) return 0;
    }

    return 1;
}

/* ---------- Segment-clear check (wrap-aware) ---------- */

static inline int segment_is_clear(avd_real p0x, avd_real p0y,
                                   avd_real p1x, avd_real p1y,
                                   const AvdRect *f,
                                   int az_wrap)
{
    int i;
    avd_real daz = p1x - p0x;

    if (az_wrap && (daz > 180.0f || daz < -180.0f)) {
        avd_real eff_daz, bnd, t_split, el_split;

        if (daz < -180.0f) {
            eff_daz = daz + 360.0f;
            bnd     = 180.0f;
        } else {
            eff_daz = daz - 360.0f;
            bnd     = -180.0f;
        }

        if (avd_abs(eff_daz) < 1e-9f) return 1;

        t_split  = (bnd - p0x) / eff_daz;
        el_split = p0y + t_split * (p1y - p0y);

        for (i = 0; i < AVD_MAX_FORBIDDEN; i++) {
            if (!f[i].valid) continue;
            if (segment_hits_rect(p0x, p0y, bnd, el_split, &f[i]))
                return 0;
        }

        for (i = 0; i < AVD_MAX_FORBIDDEN; i++) {
            if (!f[i].valid) continue;
            if (segment_hits_rect(-bnd, el_split, p1x, p1y, &f[i]))
                return 0;
        }

        return 1;
    }

    for (i = 0; i < AVD_MAX_FORBIDDEN; i++) {
        if (!f[i].valid) continue;
        if (segment_hits_rect(p0x, p0y, p1x, p1y, &f[i]))
            return 0;
    }
    return 1;
}

/* ---------- Path-clear (respects motion profile + wrap) ---------- */

static inline int path_is_clear(avd_real p0az, avd_real p0el,
                                avd_real p1az, avd_real p1el,
                                const AvdRect *f,
                                AvdMotionProfile profile,
                                int az_wrap)
{
    switch (profile) {
    case AVD_MOTION_AZ_THEN_EL:
        if (!segment_is_clear(p0az, p0el, p1az, p0el, f, az_wrap)) return 0;
        return segment_is_clear(p1az, p0el, p1az, p1el, f, az_wrap);

    case AVD_MOTION_EL_THEN_AZ:
        if (!segment_is_clear(p0az, p0el, p0az, p1el, f, az_wrap)) return 0;
        return segment_is_clear(p0az, p1el, p1az, p1el, f, az_wrap);

    default: /* AVD_MOTION_LINEAR */
        if (!segment_is_clear(p0az, p0el, p1az, p1el, f, az_wrap)) return 0;
        if (!segment_is_clear(p0az, p1el, p1az, p1el, f, az_wrap)) return 0;
        if (!segment_is_clear(p1az, p0el, p1az, p1el, f, az_wrap)) return 0;
        return 1;
    }
}

/* ================================================================
 *  SECTION 3: FUNCTION DECLARATIONS
 * ================================================================ */

/*
 * Avoidance_BuildGraph  (PRE-OP)
 *   Precomputes the visibility graph from forbidden zones.
 *   Call ONCE in pre-op mode (time unconstrained).
 *   forbidden[16] — zones with valid flag, always 16 entries.
 */
void Avoidance_BuildGraph(AvdGraph *graph,
                          const AvdRect *forbidden,
                          const AvdRect *envelope,
                          AvdMotionProfile profile, int az_wrap);

/*
 * Avoidance_Init  (OP-MODE)
 *   Resets internal state. Must be called once before the first Step.
 */
void Avoidance_Init(AvdState *state, AvdMotionProfile profile, int az_wrap);

/*
 * Avoidance_Step  (OP-MODE)
 *   Executes one planning cycle (call every 1 ms).
 *   graph — precomputed visibility graph from Avoidance_BuildGraph.
 *           If NULL, only greedy planner is used (no A* fallback).
 */
AvdOutput Avoidance_Step(const AvdInput *in, AvdState *state,
                         const AvdGraph *graph);

#ifdef __cplusplus
}
#endif

#endif /* AVOIDANCE_SIMULINK_H */
