/*
 * sfun_avoidance_preop.c -- Simulink S-Function for Avoidance_BuildGraph
 *
 * Self-contained: all types, helpers, and BuildGraph in one file.
 * No external .c or .h dependencies needed.
 *
 * INPUT PORTS:
 *   Port 0: forbidden[32x5]  (SS_SINGLE matrix, column-major)
 *           Col 0: valid, Col 1: az_min, Col 2: el_min,
 *           Col 3: az_max, Col 4: el_max
 *   Port 1: env_az_min       (SS_SINGLE scalar)
 *   Port 2: env_el_min       (SS_SINGLE scalar)
 *   Port 3: env_az_max       (SS_SINGLE scalar)
 *   Port 4: env_el_max       (SS_SINGLE scalar)
 *   Port 5: az_wrap          (SS_SINGLE scalar, 0 or 1)
 *   Port 6: motion_profile   (SS_SINGLE scalar, 0/1/2)
 *
 * OUTPUT PORTS:
 *   Port 0: graph_nc         (SS_INT16 scalar, node count)
 *   Port 1: graph_naz[256]   (SS_SINGLE 1x256, node AZ coords)
 *   Port 2: graph_nel[256]   (SS_SINGLE 1x256, node EL coords)
 *   Port 3: graph_adj[8192]  (SS_UINT8  1x8192, adjacency 256x32 row-major)
 *
 * Build in MATLAB:
 *   mex sfun_avoidance_preop.c
 */

#define S_FUNCTION_NAME  sfun_avoidance_preop
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#if defined(MATLAB_MEX_FILE)
    #include "tmwtypes.h"
    #include "simstruc_types.h"
#else
    #include "rtwtypes.h"
#endif

/* ================================================================
 *  CONFIGURATION CONSTANTS
 * ================================================================ */
#define AVD_MAX_FORBIDDEN       32
#define AVD_CORNER_EPS          0.1f
#define AVD_CANDIDATES_PER_RECT 8
#define AVD_MAX_FIXED_NODES     (AVD_MAX_FORBIDDEN * AVD_CANDIDATES_PER_RECT) /* 256 */
#define AVD_ADJ_BYTES           ((AVD_MAX_FIXED_NODES + 7) / 8)              /* 32  */

typedef float avd_real;

/* Motion profile enum */
typedef enum {
    AVD_MOTION_LINEAR     = 0,
    AVD_MOTION_AZ_THEN_EL = 1,
    AVD_MOTION_EL_THEN_AZ = 2
} AvdMotionProfile;

/* Forbidden zone / envelope rectangle */
typedef struct {
    int      valid;
    avd_real az_min, el_min;
    avd_real az_max, el_max;
} AvdRect;

/* Precomputed visibility graph */
typedef struct {
    int           nc;
    avd_real      naz[AVD_MAX_FIXED_NODES];
    avd_real      nel[AVD_MAX_FIXED_NODES];
    unsigned char adj[AVD_MAX_FIXED_NODES][AVD_ADJ_BYTES];
} AvdGraph;

/* ================================================================
 *  WRAPPED-ZONE PREPROCESSING
 *
 *  If az_min > az_max, the zone wraps around the +/-180 boundary.
 *  Split it into two non-wrapping zones so that segment_hits_rect
 *  (Liang-Barsky) handles them correctly.
 *
 *  Example: AZ[150,-150] EL[0,35] becomes:
 *    Zone A: AZ[150, env_az_max] EL[0,35]
 *    Zone B: AZ[env_az_min, -150] EL[0,35]
 * ================================================================ */
static void split_wrapped_zones(AvdRect *zones, int max_zones,
                                avd_real env_az_min, avd_real env_az_max)
{
    int k, s;
    for (k = 0; k < max_zones; k++) {
        if (!zones[k].valid) continue;
        if (zones[k].az_min <= zones[k].az_max) continue;

        /* This zone wraps: az_min > az_max */
        avd_real orig_az_max = zones[k].az_max;

        /* Zone A: [az_min, env_az_max] (right side of +180 boundary) */
        zones[k].az_max = env_az_max;

        /* Zone B: [env_az_min, orig_az_max] (left side of -180 boundary) */
        /* Find an empty slot for Zone B */
        for (s = 0; s < max_zones; s++) {
            if (!zones[s].valid) {
                zones[s].valid  = 1;
                zones[s].az_min = env_az_min;
                zones[s].el_min = zones[k].el_min;
                zones[s].az_max = orig_az_max;
                zones[s].el_max = zones[k].el_max;
                break;
            }
        }
    }
}

/* ================================================================
 *  MATH HELPERS  (no <math.h>)
 * ================================================================ */
static avd_real avd_abs(avd_real v)    { return v < 0.0f ? -v : v; }
static avd_real avd_min(avd_real a, avd_real b) { return a < b ? a : b; }
static avd_real avd_max(avd_real a, avd_real b) { return a > b ? a : b; }

static avd_real avd_clamp(avd_real v, avd_real lo, avd_real hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static avd_real avd_normalize_az(avd_real az)
{
    if (az > 180.0f)   return az - 360.0f;
    if (az <= -180.0f) return az + 360.0f;
    return az;
}

static avd_real avd_manhattan(avd_real ax, avd_real ay,
                              avd_real bx, avd_real by,
                              int az_wrap)
{
    avd_real daz = avd_abs(ax - bx);
    if (az_wrap && daz > 180.0f) daz = 360.0f - daz;
    return daz + avd_abs(ay - by);
}

/* ================================================================
 *  GEOMETRIC PREDICATES
 * ================================================================ */
static int point_in_rect(avd_real az, avd_real el, const AvdRect *r)
{
    return (az > r->az_min && az < r->az_max &&
            el > r->el_min && el < r->el_max);
}

static int point_in_any_forbidden(avd_real az, avd_real el, const AvdRect *f)
{
    int i;
    for (i = 0; i < AVD_MAX_FORBIDDEN; i++) {
        if (!f[i].valid) continue;
        if (point_in_rect(az, el, &f[i])) return 1;
    }
    return 0;
}

static int point_in_envelope(avd_real az, avd_real el, const AvdRect *env)
{
    return (az >= env->az_min && az <= env->az_max &&
            el >= env->el_min && el <= env->el_max);
}

/* ================================================================
 *  LIANG-BARSKY  SEGMENT vs AABB
 * ================================================================ */
static int segment_hits_rect(avd_real p0x, avd_real p0y,
                             avd_real p1x, avd_real p1y,
                             const AvdRect *r)
{
    avd_real dx = p1x - p0x;
    avd_real dy = p1y - p0y;
    avd_real t_near = 0.0f, t_far = 1.0f;
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

/* ================================================================
 *  SEGMENT-CLEAR  (wrap-aware)
 * ================================================================ */
static int segment_is_clear(avd_real p0x, avd_real p0y,
                            avd_real p1x, avd_real p1y,
                            const AvdRect *f, int az_wrap)
{
    int i;
    avd_real daz = p1x - p0x;

    if (az_wrap && (daz > 180.0f || daz < -180.0f)) {
        avd_real eff_daz, bnd, t_split, el_split;

        if (daz < -180.0f) {
            eff_daz = daz + 360.0f;  bnd = 180.0f;
        } else {
            eff_daz = daz - 360.0f;  bnd = -180.0f;
        }
        if (avd_abs(eff_daz) < 1e-9f) return 1;

        t_split  = (bnd - p0x) / eff_daz;
        el_split = p0y + t_split * (p1y - p0y);

        for (i = 0; i < AVD_MAX_FORBIDDEN; i++) {
            if (!f[i].valid) continue;
            if (segment_hits_rect(p0x, p0y, bnd, el_split, &f[i])) return 0;
        }
        for (i = 0; i < AVD_MAX_FORBIDDEN; i++) {
            if (!f[i].valid) continue;
            if (segment_hits_rect(-bnd, el_split, p1x, p1y, &f[i])) return 0;
        }
        return 1;
    }

    for (i = 0; i < AVD_MAX_FORBIDDEN; i++) {
        if (!f[i].valid) continue;
        if (segment_hits_rect(p0x, p0y, p1x, p1y, &f[i])) return 0;
    }
    return 1;
}

/* ================================================================
 *  PATH-CLEAR  (respects motion profile + wrap)
 * ================================================================ */
static int path_is_clear(avd_real p0az, avd_real p0el,
                         avd_real p1az, avd_real p1el,
                         const AvdRect *f,
                         AvdMotionProfile profile, int az_wrap)
{
    switch (profile) {
    case AVD_MOTION_AZ_THEN_EL:
        if (!segment_is_clear(p0az, p0el, p1az, p0el, f, az_wrap)) return 0;
        return segment_is_clear(p1az, p0el, p1az, p1el, f, az_wrap);
    case AVD_MOTION_EL_THEN_AZ:
        if (!segment_is_clear(p0az, p0el, p0az, p1el, f, az_wrap)) return 0;
        return segment_is_clear(p0az, p1el, p1az, p1el, f, az_wrap);
    default: /* AVD_MOTION_LINEAR — only check diagonal */
        return segment_is_clear(p0az, p0el, p1az, p1el, f, az_wrap);
    }
}

/* ================================================================
 *  ADJACENCY BIT HELPER
 * ================================================================ */
static void adj_set(unsigned char adj[][AVD_ADJ_BYTES], int i, int j)
{
    adj[i][j >> 3] |= (unsigned char)(1u << (j & 7));
}

/* ================================================================
 *  Avoidance_BuildGraph  —  FULL IMPLEMENTATION
 *
 *  Generates candidate nodes at zone corners + edge midpoints,
 *  then tests all-pairs visibility using Liang-Barsky.
 *  Result: bit-packed adjacency matrix (~2 KB for 128 nodes).
 * ================================================================ */
static void Avoidance_BuildGraph(AvdGraph *graph,
                                 const AvdRect *forbidden,
                                 const AvdRect *envelope,
                                 AvdMotionProfile profile, int az_wrap)
{
    int i, c, j;
    int nc = 0;

    /* If envelope is not valid, produce empty graph */
    if (!envelope->valid) {
        graph->nc = 0;
        return;
    }

    /* ---- generate candidate nodes ---- */
    for (i = 0; i < AVD_MAX_FORBIDDEN && nc < AVD_MAX_FIXED_NODES; i++) {
        avd_real az_lo, az_hi, el_lo, el_hi, az_mid, el_mid, eps;
        avd_real caz[8], cel[8];

        if (!forbidden[i].valid) continue;
        az_lo  = forbidden[i].az_min;
        az_hi  = forbidden[i].az_max;
        el_lo  = forbidden[i].el_min;
        el_hi  = forbidden[i].el_max;
        az_mid = (az_lo + az_hi) * 0.5f;
        el_mid = (el_lo + el_hi) * 0.5f;
        eps    = AVD_CORNER_EPS;

        caz[0] = az_lo - eps;  cel[0] = el_lo - eps;
        caz[1] = az_hi + eps;  cel[1] = el_lo - eps;
        caz[2] = az_hi + eps;  cel[2] = el_hi + eps;
        caz[3] = az_lo - eps;  cel[3] = el_hi + eps;
        caz[4] = az_mid;       cel[4] = el_lo - eps;
        caz[5] = az_mid;       cel[5] = el_hi + eps;
        caz[6] = az_lo - eps;  cel[6] = el_mid;
        caz[7] = az_hi + eps;  cel[7] = el_mid;

        for (c = 0; c < AVD_CANDIDATES_PER_RECT && nc < AVD_MAX_FIXED_NODES; c++) {
            avd_real waz = caz[c], wel = cel[c];
            if (az_wrap) waz = avd_normalize_az(waz);
            /* Skip nodes outside envelope (don't clamp — clamping
             * puts nodes ON zone boundaries when zone edge = envelope edge,
             * causing false adjacencies along the boundary) */
            if (!point_in_envelope(waz, wel, envelope)) continue;
            if (point_in_any_forbidden(waz, wel, forbidden)) continue;
            graph->naz[nc] = waz;
            graph->nel[nc] = wel;
            nc++;
        }
    }

    /* ---- add sweep nodes at regular AZ intervals ----
     * Ensures graph connectivity across long AZ spans where
     * zone corners may be far apart.  Nodes at bottom, top,
     * and mid EL of the envelope every 30 degrees AZ.
     */
    {
        avd_real sweep_az;
        avd_real sweep_el[3];
        int ne, ns;
        avd_real az_step = 30.0f;

        sweep_el[0] = envelope->el_min + AVD_CORNER_EPS;
        sweep_el[1] = envelope->el_max - AVD_CORNER_EPS;
        sweep_el[2] = (envelope->el_min + envelope->el_max) * 0.5f;

        for (sweep_az = envelope->az_min + az_step;
             sweep_az < envelope->az_max && nc < AVD_MAX_FIXED_NODES;
             sweep_az += az_step) {
            avd_real saz = sweep_az;
            if (az_wrap) saz = avd_normalize_az(saz);

            for (ne = 0; ne < 3 && nc < AVD_MAX_FIXED_NODES; ne++) {
                avd_real sel = sweep_el[ne];
                if (point_in_any_forbidden(saz, sel, forbidden)) continue;

                /* Avoid duplicates: skip if a node already exists nearby */
                {
                    int dup = 0;
                    for (ns = 0; ns < nc; ns++) {
                        if (avd_manhattan(saz, sel, graph->naz[ns],
                                          graph->nel[ns], az_wrap) < 1.0f) {
                            dup = 1;
                            break;
                        }
                    }
                    if (dup) continue;
                }
                graph->naz[nc] = saz;
                graph->nel[nc] = sel;
                nc++;
            }
        }
    }

    graph->nc = nc;

    /* ---- compute all-pairs visibility -> adjacency matrix ---- */
    for (i = 0; i < nc; i++) {
        for (j = 0; j < AVD_ADJ_BYTES; j++) {
            graph->adj[i][j] = 0;
        }
    }

    for (i = 0; i < nc; i++) {
        for (j = i + 1; j < nc; j++) {
            if (path_is_clear(graph->naz[i], graph->nel[i],
                              graph->naz[j], graph->nel[j],
                              forbidden, profile, az_wrap)) {
                adj_set(graph->adj, i, j);
                adj_set(graph->adj, j, i);
            }
        }
    }
}

/* ================================================================
 *  S-FUNCTION METHODS
 * ================================================================ */

#define NUM_INPUTS   7
#define NUM_OUTPUTS  4
#define FZONE_ROWS   32
#define FZONE_COLS   5

static void mdlInitializeSizes(SimStruct *S)
{
    int_T i;

    ssSetNumSFcnParams(S, 0);

    /* === Input Ports === */
    ssSetNumInputPorts(S, NUM_INPUTS);

    /* Port 0: forbidden zone matrix [32x5] single */
    ssSetInputPortMatrixDimensions(S, 0, FZONE_ROWS, FZONE_COLS);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDataType(S, 0, SS_SINGLE);
    ssSetInputPortRequiredContiguous(S, 0, 1);

    /* Ports 1-6: scalar singles */
    for (i = 1; i < NUM_INPUTS; i++) {
        ssSetInputPortWidth(S, i, 1);
        ssSetInputPortDirectFeedThrough(S, i, 1);
        ssSetInputPortDataType(S, i, SS_SINGLE);
        ssSetInputPortRequiredContiguous(S, i, 1);
    }

    /* === Output Ports === */
    ssSetNumOutputPorts(S, NUM_OUTPUTS);

    /* Port 0: graph_nc (int16 scalar) */
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortDataType(S, 0, SS_INT16);

    /* Port 1: graph_naz [256] single */
    ssSetOutputPortWidth(S, 1, AVD_MAX_FIXED_NODES);
    ssSetOutputPortDataType(S, 1, SS_SINGLE);

    /* Port 2: graph_nel [256] single */
    ssSetOutputPortWidth(S, 2, AVD_MAX_FIXED_NODES);
    ssSetOutputPortDataType(S, 2, SS_SINGLE);

    /* Port 3: graph_adj [8192] uint8 (256 rows x 32 bytes, row-major) */
    ssSetOutputPortWidth(S, 3, AVD_MAX_FIXED_NODES * AVD_ADJ_BYTES);
    ssSetOutputPortDataType(S, 3, SS_UINT8);

    ssSetNumSampleTimes(S, 1);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

/* Function: mdlOutputs */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    uint16_T i, j;
    AvdGraph graph;

    /* ============================================================
     *  READ INPUTS
     * ============================================================ */

    /* Port 0: forbidden zone matrix [32x5] column-major
     *   MATLAB column-major: element [row][col] = index row + col * numRows
     *   Col 0 = valid, Col 1 = az_min, Col 2 = el_min,
     *   Col 3 = az_max, Col 4 = el_max                          */
    const real32_T *p0 = (const real32_T *)ssGetInputPortSignal(S, 0);
    AvdRect forbidden[AVD_MAX_FORBIDDEN];

    for (i = 0; i < FZONE_ROWS; i++) {
        forbidden[i].valid  = (int)(p0[i + 0 * FZONE_ROWS]);
        forbidden[i].az_min =       p0[i + 1 * FZONE_ROWS];
        forbidden[i].el_min =       p0[i + 2 * FZONE_ROWS];
        forbidden[i].az_max =       p0[i + 3 * FZONE_ROWS];
        forbidden[i].el_max =       p0[i + 4 * FZONE_ROWS];
    }

    /* Ports 1-4: envelope scalars */
    const real32_T *p1 = (const real32_T *)ssGetInputPortSignal(S, 1);
    const real32_T *p2 = (const real32_T *)ssGetInputPortSignal(S, 2);
    const real32_T *p3 = (const real32_T *)ssGetInputPortSignal(S, 3);
    const real32_T *p4 = (const real32_T *)ssGetInputPortSignal(S, 4);

    AvdRect envelope;
    envelope.valid  = 1;
    envelope.az_min = p1[0];
    envelope.el_min = p2[0];
    envelope.az_max = p3[0];
    envelope.el_max = p4[0];

    /* Port 5: az_wrap */
    const real32_T *p5 = (const real32_T *)ssGetInputPortSignal(S, 5);
    int az_wrap = (int)(p5[0]);

    /* Port 6: motion_profile (0=LINEAR, 1=AZ_THEN_EL, 2=EL_THEN_AZ) */
    const real32_T *p6 = (const real32_T *)ssGetInputPortSignal(S, 6);
    AvdMotionProfile profile = (AvdMotionProfile)(int)(p6[0]);

    /* Split any wrapped zones (az_min > az_max) into two non-wrapping zones */
    split_wrapped_zones(forbidden, AVD_MAX_FORBIDDEN,
                        envelope.az_min, envelope.az_max);

    /* ============================================================
     *  CALL BuildGraph
     * ============================================================ */
    Avoidance_BuildGraph(&graph, forbidden, &envelope, profile, az_wrap);

    /* ============================================================
     *  WRITE OUTPUTS
     * ============================================================ */

    /* Port 0: graph_nc */
    int16_T *y_nc = (int16_T *)ssGetOutputPortSignal(S, 0);
    *y_nc = (int16_T)graph.nc;

    /* Port 1: graph_naz[128] */
    real32_T *y_naz = (real32_T *)ssGetOutputPortSignal(S, 1);
    for (i = 0; i < AVD_MAX_FIXED_NODES; i++) {
        y_naz[i] = (real32_T)graph.naz[i];
    }

    /* Port 2: graph_nel[128] */
    real32_T *y_nel = (real32_T *)ssGetOutputPortSignal(S, 2);
    for (i = 0; i < AVD_MAX_FIXED_NODES; i++) {
        y_nel[i] = (real32_T)graph.nel[i];
    }

    /* Port 3: graph_adj[128x16] flattened row-major to 2048 bytes */
    uint8_T *y_adj = (uint8_T *)ssGetOutputPortSignal(S, 3);
    for (i = 0; i < AVD_MAX_FIXED_NODES; i++) {
        for (j = 0; j < AVD_ADJ_BYTES; j++) {
            y_adj[i * AVD_ADJ_BYTES + j] = (uint8_T)graph.adj[i][j];
        }
    }
}

static void mdlTerminate(SimStruct *S)
{
    /* No actions needed */
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
