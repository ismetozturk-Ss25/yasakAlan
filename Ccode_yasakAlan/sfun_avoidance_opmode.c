/*
 * sfun_avoidance_opmode.c -- Simulink S-Function for Avoidance_Step
 *
 * Self-contained: all types, helpers, Init, and Step in one file.
 * No external .c or .h dependencies needed.
 *
 * INPUT PORTS:
 *   Port  0: az_now          (SS_SINGLE scalar, current AZ position)
 *   Port  1: el_now          (SS_SINGLE scalar, current EL position)
 *   Port  2: az_cmd          (SS_SINGLE scalar, commanded AZ target)
 *   Port  3: el_cmd          (SS_SINGLE scalar, commanded EL target)
 *   Port  4: env_az_min      (SS_SINGLE scalar)
 *   Port  5: env_el_min      (SS_SINGLE scalar)
 *   Port  6: env_az_max      (SS_SINGLE scalar)
 *   Port  7: env_el_max      (SS_SINGLE scalar)
 *   Port  8: forbidden[32x5] (SS_SINGLE matrix, column-major)
 *            Col 0: valid, Col 1: az_min, Col 2: el_min,
 *            Col 3: az_max, Col 4: el_max
 *   Port  9: az_wrap         (SS_SINGLE scalar, 0 or 1)
 *   Port 10: motion_profile  (SS_SINGLE scalar, 0/1/2)
 *   Port 11: graph_nc        (SS_INT16 scalar, from PreOp block)
 *   Port 12: graph_naz[256]  (SS_SINGLE 1x256, from PreOp block)
 *   Port 13: graph_nel[256]  (SS_SINGLE 1x256, from PreOp block)
 *   Port 14: graph_adj[8192] (SS_UINT8  1x8192, from PreOp block)
 *
 * OUTPUT PORTS:
 *   Port 0: az_next     (SS_SINGLE scalar, next AZ command)
 *   Port 1: el_next     (SS_SINGLE scalar, next EL command)
 *   Port 2: status      (SS_INT16  scalar, 0=DIRECT, 1=WAYPOINT, 2=NO_PATH)
 *   Port 3: debug_node  (SS_SINGLE scalar, target node index, -1=DIRECT, -2=NO_PATH)
 *
 * PERSISTENT STATE (DWork):
 *   AvdState struct, initialized via Avoidance_Init() at t=0
 *
 * Build in MATLAB:
 *   mex sfun_avoidance_opmode.c
 */

#define S_FUNCTION_NAME  sfun_avoidance_opmode
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
#define AVD_WP_REACH_THRESH     0.3f
#define AVD_CANDIDATES_PER_RECT 8
#define AVD_MAX_PATH_LEN        48
#define AVD_TGT_MOVE_THRESH     2.0f
#define AVD_MAX_FIXED_NODES     (AVD_MAX_FORBIDDEN * AVD_CANDIDATES_PER_RECT) /* 256 */
#define AVD_ADJ_BYTES           ((AVD_MAX_FIXED_NODES + 7) / 8)              /* 32  */

/* Output rate limit: max degrees (manhattan) the command can move from
 * the current turret position per call.  Tune to your turret's max
 * velocity × sample time.  Example: 30 deg/s turret, 1 ms step → 0.03.
 * Set to 0 to disable rate limiting.                                     */
#define AVD_MAX_STEP            0.0f

typedef float avd_real;

/* ================================================================
 *  ENUMERATIONS
 * ================================================================ */
typedef enum {
    AVD_OK_DIRECT   = 0,
    AVD_OK_WAYPOINT = 1,
    AVD_NO_PATH     = 2
} AvdStatus;

typedef enum {
    AVD_MOTION_LINEAR     = 0,
    AVD_MOTION_AZ_THEN_EL = 1,
    AVD_MOTION_EL_THEN_AZ = 2
} AvdMotionProfile;

/* ================================================================
 *  DATA STRUCTURES
 * ================================================================ */
typedef struct {
    int      valid;
    avd_real az_min, el_min;
    avd_real az_max, el_max;
} AvdRect;

typedef struct {
    int           nc;
    avd_real      naz[AVD_MAX_FIXED_NODES];
    avd_real      nel[AVD_MAX_FIXED_NODES];
    unsigned char adj[AVD_MAX_FIXED_NODES][AVD_ADJ_BYTES];
} AvdGraph;

typedef struct {
    avd_real  az_now, el_now;
    avd_real  az_cmd, el_cmd;
    AvdRect   envelope;
    AvdRect   forbidden[AVD_MAX_FORBIDDEN];
} AvdInput;

typedef struct {
    avd_real   az_next, el_next;
    AvdStatus  status;
} AvdOutput;

typedef struct {
    AvdMotionProfile profile;
    int              az_wrap;
    avd_real         path_az[AVD_MAX_PATH_LEN];
    avd_real         path_el[AVD_MAX_PATH_LEN];
    int              path_len;
    int              path_idx;
    int              path_valid;
    avd_real         path_tgt_az;
    avd_real         path_tgt_el;
    int              dbg_step;      /* call counter for debug prints  */
    /* Cached output for minor time steps (ode3 solver protection) */
    avd_real         cached_az_next;
    avd_real         cached_el_next;
    int              cached_status;
    int              cached_node;
    int              has_cached;
} AvdState;

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
 *  ADJACENCY BIT HELPER  (read-only)
 * ================================================================ */
static int adj_get(const unsigned char adj[][AVD_ADJ_BYTES], int i, int j)
{
    return (adj[i][j >> 3] >> (j & 7)) & 1;
}

/* ================================================================
 *  TARGET PROJECTION
 * ================================================================ */
static void project_target(avd_real *taz, avd_real *tel,
                           const AvdRect *f,
                           const AvdRect *env,
                           int az_wrap)
{
    int i;
    for (i = 0; i < AVD_MAX_FORBIDDEN; i++) {
        if (!f[i].valid) continue;
        if (!point_in_rect(*taz, *tel, &f[i])) continue;

        avd_real d[4];
        avd_real proj_az[4], proj_el[4];

        d[0] = avd_abs(*taz - f[i].az_min);
        proj_az[0] = f[i].az_min - AVD_CORNER_EPS;
        proj_el[0] = *tel;

        d[1] = avd_abs(*taz - f[i].az_max);
        proj_az[1] = f[i].az_max + AVD_CORNER_EPS;
        proj_el[1] = *tel;

        d[2] = avd_abs(*tel - f[i].el_min);
        proj_az[2] = *taz;
        proj_el[2] = f[i].el_min - AVD_CORNER_EPS;

        d[3] = avd_abs(*tel - f[i].el_max);
        proj_az[3] = *taz;
        proj_el[3] = f[i].el_max + AVD_CORNER_EPS;

        {
            int best = 0, k;
            avd_real best_d = d[0];
            for (k = 1; k < 4; k++) {
                if (d[k] < best_d) { best_d = d[k]; best = k; }
            }

            *taz = avd_clamp(proj_az[best], env->az_min, env->az_max);
            *tel = avd_clamp(proj_el[best], env->el_min, env->el_max);
            if (az_wrap) *taz = avd_normalize_az(*taz);
        }
        break;
    }
}

/* ================================================================
 *  EMERGENCY ESCAPE
 * ================================================================ */
static int try_escape(avd_real cur_az, avd_real cur_el,
                      const AvdRect *f,
                      const AvdRect *env,
                      int az_wrap,
                      AvdOutput *out)
{
    int i;
    for (i = 0; i < AVD_MAX_FORBIDDEN; i++) {
        if (!f[i].valid) continue;
        if (!point_in_rect(cur_az, cur_el, &f[i])) continue;

        {
            avd_real cand_az[4], cand_el[4], cand_d[4];
            int order[4] = {0, 1, 2, 3};
            int j, k;

            cand_az[0] = f[i].az_min - AVD_CORNER_EPS;  cand_el[0] = cur_el;
            cand_d[0]  = avd_abs(cur_az - f[i].az_min);

            cand_az[1] = f[i].az_max + AVD_CORNER_EPS;  cand_el[1] = cur_el;
            cand_d[1]  = avd_abs(cur_az - f[i].az_max);

            cand_az[2] = cur_az;  cand_el[2] = f[i].el_min - AVD_CORNER_EPS;
            cand_d[2]  = avd_abs(cur_el - f[i].el_min);

            cand_az[3] = cur_az;  cand_el[3] = f[i].el_max + AVD_CORNER_EPS;
            cand_d[3]  = avd_abs(cur_el - f[i].el_max);

            /* Sort by distance (insertion sort, 4 elements) */
            for (j = 1; j < 4; j++) {
                int key = order[j];
                avd_real kd = cand_d[key];
                k = j - 1;
                while (k >= 0 && cand_d[order[k]] > kd) {
                    order[k + 1] = order[k];
                    k--;
                }
                order[k + 1] = key;
            }

            for (j = 0; j < 4; j++) {
                int idx = order[j];
                avd_real az = avd_clamp(cand_az[idx], env->az_min, env->az_max);
                avd_real el = avd_clamp(cand_el[idx], env->el_min, env->el_max);
                if (az_wrap) az = avd_normalize_az(az);

                if (!point_in_envelope(az, el, env)) continue;
                if (point_in_any_forbidden(az, el, f)) continue;

                out->az_next = az;
                out->el_next = el;
                out->status  = AVD_OK_WAYPOINT;
                return 1;
            }
        }
        return 0;
    }
    return 0;
}

/* ================================================================
 *  NEAREST REACHABLE GRAPH NODE
 *
 *  Find closest node that has path_is_clear from given position.
 *  Falls back to closest-by-distance if no reachable node found.
 * ================================================================ */
static int nearest_reachable_node(avd_real az, avd_real el,
                                  const AvdGraph *graph,
                                  const AvdRect *f,
                                  AvdMotionProfile profile,
                                  int az_wrap)
{
    int i;
    int best_reachable = -1;
    avd_real best_reach_d = 1.0e30f;
    int best_any = -1;
    avd_real best_any_d = 1.0e30f;

    for (i = 0; i < graph->nc; i++) {
        avd_real d = avd_manhattan(az, el,
                                    graph->naz[i], graph->nel[i], az_wrap);
        if (d < best_any_d) {
            best_any_d = d;
            best_any = i;
        }
        if (path_is_clear(az, el, graph->naz[i], graph->nel[i],
                          f, profile, az_wrap)) {
            if (d < best_reach_d) {
                best_reach_d = d;
                best_reachable = i;
            }
        }
    }
    return (best_reachable >= 0) ? best_reachable : best_any;
}

/* For debug output: simple nearest by distance */
static int nearest_node(avd_real az, avd_real el,
                        const AvdGraph *graph, int az_wrap)
{
    int i, best = -1;
    avd_real best_d = 1.0e30f;
    for (i = 0; i < graph->nc; i++) {
        avd_real d = avd_manhattan(az, el,
                                    graph->naz[i], graph->nel[i], az_wrap);
        if (d < best_d) {
            best_d = d;
            best = i;
        }
    }
    return best;
}

/* ================================================================
 *  A* ON PRECOMPUTED GRAPH
 * ================================================================ */
static int astar_on_graph(int src_idx, int dst_idx,
                           const AvdGraph *graph, int az_wrap,
                           avd_real *out_path_az, avd_real *out_path_el,
                           int max_path_len)
{
    int nc = graph->nc;
    int i, j, cur;
    avd_real best_f_val, tent_g, edge_cost;
    avd_real goal_az, goal_el;

    avd_real g_cost[AVD_MAX_FIXED_NODES];
    avd_real f_cost[AVD_MAX_FIXED_NODES];
    int parent[AVD_MAX_FIXED_NODES];
    int closed[AVD_MAX_FIXED_NODES];

    if (src_idx < 0 || dst_idx < 0 || src_idx == dst_idx) return 0;

    goal_az = graph->naz[dst_idx];
    goal_el = graph->nel[dst_idx];

    for (i = 0; i < nc; i++) {
        g_cost[i] = 1.0e30f;
        f_cost[i] = 1.0e30f;
        parent[i] = -1;
        closed[i] = 0;
    }
    g_cost[src_idx] = 0.0f;
    f_cost[src_idx] = avd_manhattan(graph->naz[src_idx], graph->nel[src_idx],
                                     goal_az, goal_el, az_wrap);

    for (;;) {
        cur = -1;
        best_f_val = 1.0e30f;
        for (i = 0; i < nc; i++) {
            if (!closed[i] && f_cost[i] < best_f_val) {
                best_f_val = f_cost[i];
                cur = i;
            }
        }
        if (cur < 0) return 0;
        if (cur == dst_idx) break;

        closed[cur] = 1;

        for (j = 0; j < nc; j++) {
            if (closed[j] || j == cur) continue;
            if (!adj_get(graph->adj, cur, j)) continue;

            edge_cost = avd_manhattan(graph->naz[cur], graph->nel[cur],
                                       graph->naz[j], graph->nel[j], az_wrap);
            tent_g = g_cost[cur] + edge_cost;

            if (tent_g < g_cost[j]) {
                g_cost[j] = tent_g;
                f_cost[j] = tent_g + avd_manhattan(graph->naz[j], graph->nel[j],
                                                    goal_az, goal_el, az_wrap);
                parent[j] = cur;
            }
        }
    }

    /* reconstruct path (exclude src, include dst) */
    {
        int total = 0, plen, node;

        node = dst_idx;
        while (node != src_idx) {
            total++;
            node = parent[node];
            if (node < 0 || total > nc) return 0;
        }

        plen = total < max_path_len ? total : max_path_len;

        node = dst_idx;
        for (i = 0; i < total - plen; i++)
            node = parent[node];

        for (i = plen - 1; i >= 0; i--) {
            out_path_az[i] = graph->naz[node];
            out_path_el[i] = graph->nel[node];
            node = parent[node];
        }
        return plen;
    }
}

/* ================================================================
 *  Avoidance_Init  —  Reset state at t=0
 * ================================================================ */
static void Avoidance_Init(AvdState *state, AvdMotionProfile profile,
                           int az_wrap)
{
    state->profile     = profile;
    state->az_wrap     = az_wrap;
    state->path_len    = 0;
    state->path_idx    = 0;
    state->path_valid  = 0;
    state->path_tgt_az = 0.0f;
    state->path_tgt_el = 0.0f;
}

/* ================================================================
 *  Avoidance_Step  —  One planning cycle (called every 1 ms)
 *  5-step algorithm: escape, project, direct, A* path, no-path
 * ================================================================ */
static AvdOutput Avoidance_Step(const AvdInput *in, AvdState *state,
                                const AvdGraph *graph)
{
    AvdOutput out;
    const AvdRect *env = &in->envelope;
    const AvdRect *f   = in->forbidden;
    AvdMotionProfile prof = state->profile;
    int wrap           = state->az_wrap;

    avd_real cur_az = in->az_now;
    avd_real cur_el = in->el_now;
    avd_real tgt_az, tgt_el;

    /* Step 1: Escape if currently inside a forbidden zone */
    if (try_escape(cur_az, cur_el, f, env, wrap, &out)) {
        state->path_valid = 0;
        return out;
    }

    /* Step 2: Clamp + project target */
    tgt_az = avd_clamp(in->az_cmd, env->az_min, env->az_max);
    tgt_el = avd_clamp(in->el_cmd, env->el_min, env->el_max);
    if (wrap) tgt_az = avd_normalize_az(tgt_az);

    project_target(&tgt_az, &tgt_el, f, env, wrap);

    /* Step 3: Direct path clear -> OK_DIRECT */
    if (path_is_clear(cur_az, cur_el, tgt_az, tgt_el, f, prof, wrap)) {
        out.az_next   = tgt_az;
        out.el_next   = tgt_el;
        out.status    = AVD_OK_DIRECT;
        state->path_valid = 0;
        return out;
    }

    /* Step 4: A* path */

    /* 4a. Invalidate cache if target moved beyond threshold */
    if (state->path_valid) {
        if (avd_manhattan(tgt_az, tgt_el,
                          state->path_tgt_az, state->path_tgt_el, wrap)
            > AVD_TGT_MOVE_THRESH) {
            state->path_valid = 0;
        }
    }

    /* 4b. Compute new A* path if no valid cache */
    if (!state->path_valid && graph) {
        int src = nearest_reachable_node(cur_az, cur_el, graph, f, prof, wrap);
        int dst = nearest_reachable_node(tgt_az, tgt_el, graph, f, prof, wrap);
        int plen = astar_on_graph(src, dst, graph, wrap,
                                   state->path_az, state->path_el,
                                   AVD_MAX_PATH_LEN);
        if (plen > 0) {
            state->path_len    = plen;
            state->path_idx    = 0;
            state->path_valid  = 1;
            state->path_tgt_az = tgt_az;
            state->path_tgt_el = tgt_el;
        }
    }

    /* 4c. Follow cached path waypoint by waypoint */
    if (state->path_valid && state->path_idx < state->path_len) {
        avd_real wp_az = state->path_az[state->path_idx];
        avd_real wp_el = state->path_el[state->path_idx];

        if (avd_manhattan(cur_az, cur_el, wp_az, wp_el, wrap)
            < AVD_WP_REACH_THRESH) {
            state->path_idx++;
            if (state->path_idx >= state->path_len) {
                state->path_valid = 0;
            } else {
                wp_az = state->path_az[state->path_idx];
                wp_el = state->path_el[state->path_idx];
            }
        }

        if (state->path_valid && state->path_idx < state->path_len) {
            /* Shortcut: try direct to target */
            if (path_is_clear(cur_az, cur_el, tgt_az, tgt_el,
                              f, prof, wrap)) {
                out.az_next   = tgt_az;
                out.el_next   = tgt_el;
                out.status    = AVD_OK_DIRECT;
                state->path_valid = 0;
                return out;
            }

            if (path_is_clear(cur_az, cur_el, wp_az, wp_el,
                              f, prof, wrap) &&
                point_in_envelope(wp_az, wp_el, env) &&
                !point_in_any_forbidden(wp_az, wp_el, f)) {
                out.az_next = wp_az;
                out.el_next = wp_el;
                out.status  = AVD_OK_WAYPOINT;
                return out;
            }

            state->path_valid = 0;
        }
    }

    /* Retry A* if path just got invalidated */
    if (!state->path_valid && graph) {
        int src = nearest_reachable_node(cur_az, cur_el, graph, f, prof, wrap);
        int dst = nearest_reachable_node(tgt_az, tgt_el, graph, f, prof, wrap);
        int plen = astar_on_graph(src, dst, graph, wrap,
                                   state->path_az, state->path_el,
                                   AVD_MAX_PATH_LEN);
        if (plen > 0) {
            state->path_len    = plen;
            state->path_idx    = 0;
            state->path_valid  = 1;
            state->path_tgt_az = tgt_az;
            state->path_tgt_el = tgt_el;

            out.az_next = state->path_az[0];
            out.el_next = state->path_el[0];
            out.status  = AVD_OK_WAYPOINT;
            return out;
        }
    }

    /* Step 5: No path found — hold position */
    out.az_next = cur_az;
    out.el_next = cur_el;
    out.status  = AVD_NO_PATH;
    state->path_valid = 0;
    return out;
}

/* ================================================================
 *  S-FUNCTION METHODS
 * ================================================================ */

#define NUM_INPUTS   15
#define NUM_OUTPUTS  4
#define FZONE_ROWS   32
#define FZONE_COLS   5

/* DWork index for AvdState */
#define DWORK_STATE  0

/* Enable DWork and mdlInitializeConditions */
#define MDL_START
#define MDL_INITIALIZE_CONDITIONS

static void mdlInitializeSizes(SimStruct *S)
{
    int_T i;

    ssSetNumSFcnParams(S, 0);

    /* === Input Ports === */
    if (!ssSetNumInputPorts(S, NUM_INPUTS)) return;

    /* Ports 0-3: az_now, el_now, az_cmd, el_cmd (SS_SINGLE scalars) */
    for (i = 0; i < 4; i++) {
        ssSetInputPortWidth(S, i, 1);
        ssSetInputPortDirectFeedThrough(S, i, 1);
        ssSetInputPortDataType(S, i, SS_SINGLE);
        ssSetInputPortRequiredContiguous(S, i, 1);
    }

    /* Ports 4-7: env_az_min, env_el_min, env_az_max, env_el_max */
    for (i = 4; i < 8; i++) {
        ssSetInputPortWidth(S, i, 1);
        ssSetInputPortDirectFeedThrough(S, i, 1);
        ssSetInputPortDataType(S, i, SS_SINGLE);
        ssSetInputPortRequiredContiguous(S, i, 1);
    }

    /* Port 8: forbidden zone matrix [32x5] single */
    ssSetInputPortMatrixDimensions(S, 8, FZONE_ROWS, FZONE_COLS);
    ssSetInputPortDirectFeedThrough(S, 8, 1);
    ssSetInputPortDataType(S, 8, SS_SINGLE);
    ssSetInputPortRequiredContiguous(S, 8, 1);

    /* Port 9: az_wrap (SS_SINGLE scalar) */
    ssSetInputPortWidth(S, 9, 1);
    ssSetInputPortDirectFeedThrough(S, 9, 1);
    ssSetInputPortDataType(S, 9, SS_SINGLE);
    ssSetInputPortRequiredContiguous(S, 9, 1);

    /* Port 10: motion_profile (SS_SINGLE scalar) */
    ssSetInputPortWidth(S, 10, 1);
    ssSetInputPortDirectFeedThrough(S, 10, 1);
    ssSetInputPortDataType(S, 10, SS_SINGLE);
    ssSetInputPortRequiredContiguous(S, 10, 1);

    /* Port 11: graph_nc (SS_INT16 scalar, from PreOp) */
    ssSetInputPortWidth(S, 11, 1);
    ssSetInputPortDirectFeedThrough(S, 11, 1);
    ssSetInputPortDataType(S, 11, SS_INT16);
    ssSetInputPortRequiredContiguous(S, 11, 1);

    /* Port 12: graph_naz[256] (SS_SINGLE vector, from PreOp) */
    ssSetInputPortWidth(S, 12, AVD_MAX_FIXED_NODES);
    ssSetInputPortDirectFeedThrough(S, 12, 1);
    ssSetInputPortDataType(S, 12, SS_SINGLE);
    ssSetInputPortRequiredContiguous(S, 12, 1);

    /* Port 13: graph_nel[256] (SS_SINGLE vector, from PreOp) */
    ssSetInputPortWidth(S, 13, AVD_MAX_FIXED_NODES);
    ssSetInputPortDirectFeedThrough(S, 13, 1);
    ssSetInputPortDataType(S, 13, SS_SINGLE);
    ssSetInputPortRequiredContiguous(S, 13, 1);

    /* Port 14: graph_adj[8192] (SS_UINT8 vector, from PreOp) */
    ssSetInputPortWidth(S, 14, AVD_MAX_FIXED_NODES * AVD_ADJ_BYTES);
    ssSetInputPortDirectFeedThrough(S, 14, 1);
    ssSetInputPortDataType(S, 14, SS_UINT8);
    ssSetInputPortRequiredContiguous(S, 14, 1);

    /* === Output Ports === */
    if (!ssSetNumOutputPorts(S, NUM_OUTPUTS)) return;

    /* Port 0: az_next (SS_SINGLE scalar) */
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortDataType(S, 0, SS_SINGLE);

    /* Port 1: el_next (SS_SINGLE scalar) */
    ssSetOutputPortWidth(S, 1, 1);
    ssSetOutputPortDataType(S, 1, SS_SINGLE);

    /* Port 2: status (SS_INT16 scalar: 0=DIRECT, 1=WAYPOINT, 2=NO_PATH) */
    ssSetOutputPortWidth(S, 2, 1);
    ssSetOutputPortDataType(S, 2, SS_INT16);

    /* Port 3: debug_node (SS_SINGLE scalar, target node index) */
    ssSetOutputPortWidth(S, 3, 1);
    ssSetOutputPortDataType(S, 3, SS_SINGLE);

    /* === DWork: persistent AvdState === */
    ssSetNumDWork(S, 1);
    ssSetDWorkWidth(S, DWORK_STATE,
                    (int_T)((sizeof(AvdState) + sizeof(real_T) - 1) / sizeof(real_T)));
    ssSetDWorkDataType(S, DWORK_STATE, SS_DOUBLE);

    ssSetNumSampleTimes(S, 1);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

/* Called once at simulation start */
static void mdlStart(SimStruct *S)
{
    /* Zero the DWork memory */
    real_T *dw = (real_T *)ssGetDWork(S, DWORK_STATE);
    int_T n = ssGetDWorkWidth(S, DWORK_STATE);
    int_T i;
    for (i = 0; i < n; i++) dw[i] = 0.0;
}

/* Called at t=0 (and after reset) to initialize state */
static void mdlInitializeConditions(SimStruct *S)
{
    AvdState *state = (AvdState *)ssGetDWork(S, DWORK_STATE);

    /* Read az_wrap and motion_profile from contiguous input ports */
    const real32_T *p9  = (const real32_T *)ssGetInputPortSignal(S, 9);
    const real32_T *p10 = (const real32_T *)ssGetInputPortSignal(S, 10);

    int az_wrap = (int)(p9[0]);
    AvdMotionProfile profile = (AvdMotionProfile)(int)(p10[0]);

    Avoidance_Init(state, profile, az_wrap);
}

/* Function: mdlOutputs -- called every sample step */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    uint16_T i, j;
    AvdInput  in_data;
    AvdGraph  graph;
    AvdOutput out;

    AvdState *state = (AvdState *)ssGetDWork(S, DWORK_STATE);

    /* ============================================================
     *  ODE SOLVER PROTECTION
     *
     *  Continuous solvers (ode3, ode45, etc.) call mdlOutputs
     *  multiple times per step (minor steps).  The avoidance
     *  algorithm modifies persistent state (active, history,
     *  path_valid, etc.) on every call.  Running it on minor
     *  steps corrupts the state and causes wrong waypoints.
     *
     *  Fix: only run the algorithm on major time steps.
     *  On minor steps, return the last computed output.
     * ============================================================ */
    if (!ssIsMajorTimeStep(S)) {
        if (state->has_cached) {
            real32_T *y_az     = (real32_T *)ssGetOutputPortSignal(S, 0);
            real32_T *y_el     = (real32_T *)ssGetOutputPortSignal(S, 1);
            int16_T  *y_status = (int16_T  *)ssGetOutputPortSignal(S, 2);
            real32_T *y_node   = (real32_T *)ssGetOutputPortSignal(S, 3);
            *y_az     = (real32_T)state->cached_az_next;
            *y_el     = (real32_T)state->cached_el_next;
            *y_status = (int16_T)state->cached_status;
            *y_node   = (real32_T)state->cached_node;
        }
        return;
    }

    /* ============================================================
     *  READ INPUTS  (all ports are contiguous -- use direct pointers)
     * ============================================================ */

    /* Ports 0-3: az_now, el_now, az_cmd, el_cmd */
    {
        const real32_T *p0 = (const real32_T *)ssGetInputPortSignal(S, 0);
        const real32_T *p1 = (const real32_T *)ssGetInputPortSignal(S, 1);
        const real32_T *p2 = (const real32_T *)ssGetInputPortSignal(S, 2);
        const real32_T *p3 = (const real32_T *)ssGetInputPortSignal(S, 3);

        in_data.az_now = p0[0];
        in_data.el_now = p1[0];
        in_data.az_cmd = p2[0];
        in_data.el_cmd = p3[0];
    }

    /* Ports 4-7: envelope */
    {
        const real32_T *p4 = (const real32_T *)ssGetInputPortSignal(S, 4);
        const real32_T *p5 = (const real32_T *)ssGetInputPortSignal(S, 5);
        const real32_T *p6 = (const real32_T *)ssGetInputPortSignal(S, 6);
        const real32_T *p7 = (const real32_T *)ssGetInputPortSignal(S, 7);

        in_data.envelope.valid  = 1;
        in_data.envelope.az_min = p4[0];
        in_data.envelope.el_min = p5[0];
        in_data.envelope.az_max = p6[0];
        in_data.envelope.el_max = p7[0];
    }

    /* Port 8: forbidden zone matrix [32x5] column-major
     *   Col 0 = valid, Col 1 = az_min, Col 2 = el_min,
     *   Col 3 = az_max, Col 4 = el_max                          */
    {
        const real32_T *p8 = (const real32_T *)ssGetInputPortSignal(S, 8);
        for (i = 0; i < FZONE_ROWS; i++) {
            in_data.forbidden[i].valid  = (int)(p8[i + 0 * FZONE_ROWS]);
            in_data.forbidden[i].az_min =       p8[i + 1 * FZONE_ROWS];
            in_data.forbidden[i].el_min =       p8[i + 2 * FZONE_ROWS];
            in_data.forbidden[i].az_max =       p8[i + 3 * FZONE_ROWS];
            in_data.forbidden[i].el_max =       p8[i + 4 * FZONE_ROWS];
        }
    }

    /* Port 9: az_wrap, Port 10: motion_profile */
    {
        const real32_T *p9  = (const real32_T *)ssGetInputPortSignal(S, 9);
        const real32_T *p10 = (const real32_T *)ssGetInputPortSignal(S, 10);

        state->az_wrap = (int)(p9[0]);
        state->profile = (AvdMotionProfile)(int)(p10[0]);
    }

    /* Wrap-around envelope: add gap zone, expand to [-180,180] */
    if (state->az_wrap && in_data.envelope.az_min > in_data.envelope.az_max) {
        uint16_T slot;
        for (slot = 0; slot < AVD_MAX_FORBIDDEN; slot++) {
            if (!in_data.forbidden[slot].valid) {
                in_data.forbidden[slot].valid  = 1;
                in_data.forbidden[slot].az_min = in_data.envelope.az_max;
                in_data.forbidden[slot].az_max = in_data.envelope.az_min;
                in_data.forbidden[slot].el_min = in_data.envelope.el_min;
                in_data.forbidden[slot].el_max = in_data.envelope.el_max;
                break;
            }
        }
        in_data.envelope.az_min = -180.0f;
        in_data.envelope.az_max =  180.0f;
    }

    /* Split any wrapped zones (az_min > az_max) into two non-wrapping zones */
    split_wrapped_zones(in_data.forbidden, AVD_MAX_FORBIDDEN,
                        in_data.envelope.az_min, in_data.envelope.az_max);

    /* Ports 11-14: graph data from PreOp block */
    {
        const int16_T  *p11 = (const int16_T  *)ssGetInputPortSignal(S, 11);
        const real32_T *p12 = (const real32_T *)ssGetInputPortSignal(S, 12);
        const real32_T *p13 = (const real32_T *)ssGetInputPortSignal(S, 13);
        const uint8_T  *p14 = (const uint8_T  *)ssGetInputPortSignal(S, 14);

        graph.nc = (int)(p11[0]);

        for (i = 0; i < AVD_MAX_FIXED_NODES; i++) {
            graph.naz[i] = p12[i];
            graph.nel[i] = p13[i];
        }

        /* Copy adjacency: p14 is flat row-major [256*32] */
        for (i = 0; i < AVD_MAX_FIXED_NODES; i++) {
            for (j = 0; j < AVD_ADJ_BYTES; j++) {
                graph.adj[i][j] = p14[i * AVD_ADJ_BYTES + j];
            }
        }
    }

    /* ============================================================
     *  CALL Avoidance_Step
     * ============================================================ */
    out = Avoidance_Step(&in_data, state, &graph);

    /* Find target node index for debug output */
    {
        int debug_node_idx;
        if (out.status == AVD_OK_DIRECT)
            debug_node_idx = -1;
        else if (out.status == AVD_NO_PATH)
            debug_node_idx = -2;
        else
            debug_node_idx = nearest_node(out.az_next, out.el_next,
                                          &graph, state->az_wrap);

    /* ============================================================
     *  OUTPUT RATE LIMITER
     *
     *  Limits how far the command can jump from the current turret
     *  position.  Without this, the algorithm can output waypoints
     *  far from the turret (e.g., AZ=150 when turret is at AZ=30),
     *  causing severe oscillation with real dynamics.
     *
     *  This is equivalent to the C test's move_toward() function:
     *  it shapes the raw waypoint into a reachable next position.
     *
     *  Tune AVD_MAX_STEP to your turret's max_velocity × sample_time.
     * ============================================================ */
    if (AVD_MAX_STEP > 0.0f && out.status != AVD_NO_PATH) {
        avd_real daz = out.az_next - in_data.az_now;
        avd_real del = out.el_next - in_data.el_now;

        /* Wrap-aware AZ difference */
        if (state->az_wrap) {
            if (daz > 180.0f)  daz -= 360.0f;
            if (daz < -180.0f) daz += 360.0f;
        }

        {
            avd_real dist = avd_abs(daz) + avd_abs(del);
            if (dist > AVD_MAX_STEP) {
                avd_real scale = AVD_MAX_STEP / dist;
                out.az_next = in_data.az_now + daz * scale;
                out.el_next = in_data.el_now + del * scale;

                if (state->az_wrap)
                    out.az_next = avd_normalize_az(out.az_next);

                /* Keep within envelope */
                out.az_next = avd_clamp(out.az_next,
                                        in_data.envelope.az_min,
                                        in_data.envelope.az_max);
                out.el_next = avd_clamp(out.el_next,
                                        in_data.envelope.el_min,
                                        in_data.envelope.el_max);
            }
        }
    }

    /* ============================================================
     *  WRITE OUTPUTS + CACHE FOR MINOR STEPS
     * ============================================================ */
    {
        real32_T *y_az     = (real32_T *)ssGetOutputPortSignal(S, 0);
        real32_T *y_el     = (real32_T *)ssGetOutputPortSignal(S, 1);
        int16_T  *y_status = (int16_T  *)ssGetOutputPortSignal(S, 2);
        real32_T *y_node   = (real32_T *)ssGetOutputPortSignal(S, 3);

        *y_az     = (real32_T)out.az_next;
        *y_el     = (real32_T)out.el_next;
        *y_status = (int16_T)out.status;
        *y_node   = (real32_T)debug_node_idx;

        /* Cache for minor time steps */
        state->cached_az_next = out.az_next;
        state->cached_el_next = out.el_next;
        state->cached_status  = (int)out.status;
        state->cached_node    = debug_node_idx;
        state->has_cached     = 1;
    }
    } /* end debug_node scope */
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
