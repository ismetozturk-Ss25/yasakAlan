/*
 * avoidance_types.h — Shared type definitions for Avoidance Command Shaper
 *
 * Contains all constants, typedefs, enums, and structs used by both
 * the pre-op (BuildGraph) and op-mode (Init/Step) modules.
 *
 * Designed for Simulink C S-Function import:
 *   - No dynamic memory
 *   - Fixed-size structures
 *   - C99 compatible
 */

#ifndef AVOIDANCE_TYPES_H
#define AVOIDANCE_TYPES_H

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

/* Axis-aligned rectangle in AZ-EL space */
typedef struct {
    avd_real az_min, az_max;
    avd_real el_min, el_max;
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
    AvdRect   forbidden[AVD_MAX_FORBIDDEN];          /* forbidden zones       */
    int       forbidden_count;                       /* 0 .. 16              */
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

#ifdef __cplusplus
}
#endif

#endif /* AVOIDANCE_TYPES_H */
