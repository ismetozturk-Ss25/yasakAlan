/*
 * avoidance.c — Forbidden-Area Avoidance Command Shaper
 *
 * Two-phase architecture:
 *
 *   PRE-OP:  Avoidance_BuildGraph()
 *     Builds visibility graph: zone corner/midpoint nodes + adjacency matrix.
 *     All V^2 × N geometry checks done here (time unconstrained).
 *
 *   OP MODE: Avoidance_Step()
 *     1. Greedy one-step planner (fast, handles simple cases)
 *     2. A* fallback on precomputed graph (when greedy oscillates)
 *        - Only geometry: connect start/goal to graph (2 × V × N)
 *        - Search: pure array lookups on precomputed adjacency
 *        - Result cached for subsequent calls
 */

#include "avoidance.h"

/* ==============================================================
 *  INLINE MATH HELPERS  (no <math.h> dependency)
 * ============================================================== */

static avd_real avd_abs(avd_real v)
{
    return v < 0.0f ? -v : v;
}

static avd_real avd_min(avd_real a, avd_real b)
{
    return a < b ? a : b;
}

static avd_real avd_max(avd_real a, avd_real b)
{
    return a > b ? a : b;
}

static avd_real avd_clamp(avd_real v, avd_real lo, avd_real hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ==============================================================
 *  AZ WRAP-AROUND HELPERS
 * ============================================================== */

static avd_real avd_normalize_az(avd_real az)
{
    if (az > 180.0f)  return az - 360.0f;
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

/* ==============================================================
 *  GEOMETRIC PREDICATES
 * ============================================================== */

static int point_in_rect(avd_real az, avd_real el, const AvdRect *r)
{
    return (az > r->az_min && az < r->az_max &&
            el > r->el_min && el < r->el_max);
}

static int point_in_any_forbidden(avd_real az, avd_real el,
                                  const AvdRect *f)
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

/* ==============================================================
 *  LIANG-BARSKY  LINE-SEGMENT  vs  AABB  INTERSECTION
 * ============================================================== */

static int segment_hits_rect(avd_real p0x, avd_real p0y,
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

/* ==============================================================
 *  SEGMENT-CLEAR CHECK  (wrap-aware)
 * ============================================================== */

static int segment_is_clear(avd_real p0x, avd_real p0y,
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

/* ==============================================================
 *  PATH-CLEAR  (respects motion profile + wrap)
 * ============================================================== */

static int path_is_clear(avd_real p0az, avd_real p0el,
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

/* ==============================================================
 *  TARGET PROJECTION
 * ============================================================== */

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

        int best = 0;
        avd_real best_d = d[0];
        int k;
        for (k = 1; k < 4; k++) {
            if (d[k] < best_d) { best_d = d[k]; best = k; }
        }

        *taz = avd_clamp(proj_az[best], env->az_min, env->az_max);
        *tel = avd_clamp(proj_el[best], env->el_min, env->el_max);
        if (az_wrap) *taz = avd_normalize_az(*taz);
        break;
    }
}

/* ==============================================================
 *  EMERGENCY ESCAPE
 * ============================================================== */

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

        /* 4 escape candidates: left, right, bottom, top */
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

        /* Sort by distance (insertion sort on 4 elements, max 6 swaps) */
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

        /* Try nearest first; skip if blocked by another zone or envelope */
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

        /* All 4 sides blocked — should not happen in practice */
        return 0;
    }
    return 0;
}

/* ==============================================================
 *  OSCILLATION DETECTION HELPERS
 * ============================================================== */

static int waypoint_in_history(avd_real waz, avd_real wel,
                               const AvdState *state, int az_wrap)
{
    int i, idx;
    for (i = 0; i < state->hist_count; i++) {
        idx = (state->hist_idx - 1 - i + AVD_HISTORY_SIZE) % AVD_HISTORY_SIZE;
        if (avd_manhattan(waz, wel, state->hist_az[idx], state->hist_el[idx], az_wrap)
            < AVD_OSCILLATION_THRESH) {
            return 1;
        }
    }
    return 0;
}

static void add_to_history(avd_real waz, avd_real wel, AvdState *state)
{
    state->hist_az[state->hist_idx] = waz;
    state->hist_el[state->hist_idx] = wel;
    state->hist_idx = (state->hist_idx + 1) % AVD_HISTORY_SIZE;
    if (state->hist_count < AVD_HISTORY_SIZE) {
        state->hist_count++;
    }
}

static void clear_history(AvdState *state)
{
    state->hist_idx   = 0;
    state->hist_count = 0;
}

/* ==============================================================
 *  GREEDY WAYPOINT CANDIDATE EVALUATION
 * ============================================================== */

static int find_best_waypoint(avd_real cur_az, avd_real cur_el,
                              avd_real tgt_az, avd_real tgt_el,
                              const AvdRect *f,
                              const AvdRect *env,
                              AvdMotionProfile profile,
                              int az_wrap,
                              const AvdState *state,
                              avd_real *out_az, avd_real *out_el)
{
    avd_real best_through = 1.0e30f;
    avd_real best_inter   = 1.0e30f;
    avd_real through_az = 0, through_el = 0;
    avd_real inter_az   = 0, inter_el   = 0;
    int found_through = 0, found_inter = 0;
    int i, c;

    for (i = 0; i < AVD_MAX_FORBIDDEN; i++) {
        if (!f[i].valid) continue;
        avd_real az_lo = f[i].az_min;
        avd_real az_hi = f[i].az_max;
        avd_real el_lo = f[i].el_min;
        avd_real el_hi = f[i].el_max;
        avd_real az_mid = (az_lo + az_hi) * 0.5f;
        avd_real el_mid = (el_lo + el_hi) * 0.5f;
        avd_real eps = AVD_CORNER_EPS;

        avd_real cand_az[8], cand_el[8];

        cand_az[0] = az_lo - eps;   cand_el[0] = el_lo - eps;
        cand_az[1] = az_hi + eps;   cand_el[1] = el_lo - eps;
        cand_az[2] = az_hi + eps;   cand_el[2] = el_hi + eps;
        cand_az[3] = az_lo - eps;   cand_el[3] = el_hi + eps;
        cand_az[4] = az_mid;        cand_el[4] = el_lo - eps;
        cand_az[5] = az_mid;        cand_el[5] = el_hi + eps;
        cand_az[6] = az_lo - eps;   cand_el[6] = el_mid;
        cand_az[7] = az_hi + eps;   cand_el[7] = el_mid;

        for (c = 0; c < AVD_CANDIDATES_PER_RECT; c++) {
            avd_real waz = cand_az[c];
            avd_real wel = cand_el[c];
            avd_real score;
            int sees_target;

            if (az_wrap) waz = avd_normalize_az(waz);

            if (!point_in_envelope(waz, wel, env)) continue;
            if (point_in_any_forbidden(waz, wel, f)) continue;
            if (!path_is_clear(cur_az, cur_el, waz, wel, f,
                               profile, az_wrap))
                continue;
            if (avd_manhattan(cur_az, cur_el, waz, wel, az_wrap)
                < AVD_WP_REACH_THRESH)
                continue;

            if (waypoint_in_history(waz, wel, state, az_wrap))
                continue;

            score = avd_manhattan(cur_az, cur_el, waz, wel, az_wrap)
                  + avd_manhattan(waz, wel, tgt_az, tgt_el, az_wrap);

            sees_target = path_is_clear(waz, wel, tgt_az, tgt_el,
                                        f, profile, az_wrap);

            if (sees_target) {
                if (score < best_through) {
                    best_through = score;
                    through_az = waz;  through_el = wel;
                    found_through = 1;
                }
            } else {
                if (score < best_inter) {
                    best_inter = score;
                    inter_az = waz;  inter_el = wel;
                    found_inter = 1;
                }
            }
        }
    }

    if (found_through) {
        *out_az = through_az;  *out_el = through_el;
        return 1;
    }
    if (found_inter) {
        *out_az = inter_az;  *out_el = inter_el;
        return 1;
    }
    return 0;
}

/* ==============================================================
 *  BIT-PACKED  ADJACENCY  HELPERS
 * ============================================================== */

static void adj_set(unsigned char adj[][AVD_ADJ_BYTES], int i, int j)
{
    adj[i][j >> 3] |= (unsigned char)(1u << (j & 7));
}

static int adj_get(const unsigned char adj[][AVD_ADJ_BYTES], int i, int j)
{
    return (adj[i][j >> 3] >> (j & 7)) & 1;
}

/* ==============================================================
 *  PRECOMPUTED VISIBILITY GRAPH  (pre-op mode)
 *
 *  Generates candidate nodes at zone corners + edge midpoints,
 *  then tests all-pairs visibility using Liang-Barsky.
 *  Result: bit-packed adjacency matrix (~2 KB for 128 nodes).
 *
 *  Time complexity: O(V^2 × N)  where V<=128, N<=16
 *  Typical time: 1-10 ms.  Called ONCE, no deadline.
 * ============================================================== */

void Avoidance_BuildGraph(AvdGraph *graph,
                          const AvdRect *forbidden,
                          const AvdRect *envelope,
                          AvdMotionProfile profile, int az_wrap)
{
    int i, c, j;
    int nc = 0;

    /* ---- generate candidate nodes ---- */
    for (i = 0; i < AVD_MAX_FORBIDDEN && nc < AVD_MAX_FIXED_NODES; i++) {
        if (!forbidden[i].valid) continue;
        avd_real az_lo = forbidden[i].az_min;
        avd_real az_hi = forbidden[i].az_max;
        avd_real el_lo = forbidden[i].el_min;
        avd_real el_hi = forbidden[i].el_max;
        avd_real az_mid = (az_lo + az_hi) * 0.5f;
        avd_real el_mid = (el_lo + el_hi) * 0.5f;
        avd_real eps = AVD_CORNER_EPS;

        avd_real caz[8], cel[8];
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
            if (!point_in_envelope(waz, wel, envelope)) continue;
            if (point_in_any_forbidden(waz, wel, forbidden)) continue;
            graph->naz[nc] = waz;
            graph->nel[nc] = wel;
            nc++;
        }
    }

    graph->nc = nc;

    /* ---- compute all-pairs visibility → adjacency matrix ---- */
    for (i = 0; i < nc; i++) {
        for (j = 0; j < AVD_ADJ_BYTES; j++) {
            graph->adj[i][j] = 0;
        }
    }

    for (i = 0; i < nc; i++) {
        for (j = i + 1; j < nc; j++) {
            if (path_is_clear(graph->naz[i], graph->nel[i],
                              graph->naz[j], graph->nel[j],
                              forbidden,
                              profile, az_wrap)) {
                adj_set(graph->adj, i, j);
                adj_set(graph->adj, j, i);
            }
        }
    }
}

/* ==============================================================
 *  NEAREST  GRAPH  NODE  (Manhattan distance, O(V))
 *
 *  Snaps a position to the closest precomputed graph node.
 *  Cost: V comparisons = ~80 × 3 ops = ~240 ops.
 * ============================================================== */

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

/* ==============================================================
 *  A*  ON  PRECOMPUTED  GRAPH  —  ZERO GEOMETRY AT RUNTIME
 *
 *  Takes source and destination as graph NODE INDICES.
 *  All adjacency is precomputed — only array lookups during search.
 *  Edge costs = Manhattan distance (add + abs, no geometry).
 *
 *  Runtime cost:
 *    Find 2 nearest nodes: 2 × V × 3 ops     = ~480 ops
 *    A* search:            V × V bit-lookups   = ~6 400 ops
 *    Manhattan costs:      V × V × 3 ops       = ~19 200 ops
 *    Total:                                     ~26 000 ops
 *
 *  At 300 MHz DSP: ~0.09 ms.  One-time, cached.
 * ============================================================== */

static int astar_on_graph(int src_idx, int dst_idx,
                           const AvdGraph *graph, int az_wrap,
                           avd_real *out_path_az, avd_real *out_path_el,
                           int max_path_len)
{
    int nc = graph->nc;
    int i, j, cur;
    avd_real best_f_val, tent_g, edge_cost;
    avd_real goal_az, goal_el;

    /* A* working arrays (all on stack, fixed size) */
    avd_real g_cost[AVD_MAX_FIXED_NODES];
    avd_real f_cost[AVD_MAX_FIXED_NODES];
    int parent[AVD_MAX_FIXED_NODES];
    int closed[AVD_MAX_FIXED_NODES];

    if (src_idx < 0 || dst_idx < 0 || src_idx == dst_idx) return 0;

    goal_az = graph->naz[dst_idx];
    goal_el = graph->nel[dst_idx];

    /* ---- initialise A* ---- */
    for (i = 0; i < nc; i++) {
        g_cost[i] = 1.0e30f;
        f_cost[i] = 1.0e30f;
        parent[i] = -1;
        closed[i] = 0;
    }
    g_cost[src_idx] = 0.0f;
    f_cost[src_idx] = avd_manhattan(graph->naz[src_idx], graph->nel[src_idx],
                                     goal_az, goal_el, az_wrap);

    /* ---- A* main loop (pure array lookups, NO geometry) ---- */
    for (;;) {
        cur = -1;
        best_f_val = 1.0e30f;
        for (i = 0; i < nc; i++) {
            if (!closed[i] && f_cost[i] < best_f_val) {
                best_f_val = f_cost[i];
                cur = i;
            }
        }
        if (cur < 0) return 0;             /* no path */
        if (cur == dst_idx) break;          /* reached goal */

        closed[cur] = 1;

        for (j = 0; j < nc; j++) {
            if (closed[j] || j == cur) continue;

            /* ONE BIT LOOKUP — all precomputed */
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

    /* ---- reconstruct path (exclude src, include dst) ---- */
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

/* ==============================================================
 *  PUBLIC  API
 * ============================================================== */

void Avoidance_Init(AvdState *state, AvdMotionProfile profile, int az_wrap)
{
    int i;
    state->active  = 0;
    state->wp_az   = 0.0f;
    state->wp_el   = 0.0f;
    state->profile = profile;
    state->az_wrap = az_wrap;
    state->hist_idx   = 0;
    state->hist_count = 0;
    for (i = 0; i < AVD_HISTORY_SIZE; i++) {
        state->hist_az[i] = 0.0f;
        state->hist_el[i] = 0.0f;
    }
    state->path_len    = 0;
    state->path_idx    = 0;
    state->path_valid  = 0;
    state->path_tgt_az = 0.0f;
    state->path_tgt_el = 0.0f;
    state->greedy_wp_count = 0;
}

AvdOutput Avoidance_Step(const AvdInput *in, AvdState *state,
                         const AvdGraph *graph)
{
    AvdOutput out;
    const AvdRect *env = &in->envelope;
    const AvdRect *f   = in->forbidden;
    AvdMotionProfile prof = state->profile;
    int wrap           = state->az_wrap;

    avd_real cur_az = in->az_now;
    avd_real cur_el = in->el_now;

    /* ---- 0. Escape if currently inside a forbidden zone ---- */
    if (try_escape(cur_az, cur_el, f, env, wrap, &out)) {
        state->active = 0;
        state->path_valid = 0;
        return out;
    }

    /* ---- 1. Clamp target to envelope ---- */
    avd_real tgt_az = avd_clamp(in->az_cmd, env->az_min, env->az_max);
    avd_real tgt_el = avd_clamp(in->el_cmd, env->el_min, env->el_max);
    if (wrap) tgt_az = avd_normalize_az(tgt_az);

    /* ---- 2. Project target out of forbidden zones ---- */
    project_target(&tgt_az, &tgt_el, f, env, wrap);

    /* ---- 3. Direct path clear → OK_DIRECT ---- */
    if (path_is_clear(cur_az, cur_el, tgt_az, tgt_el, f, prof, wrap)) {
        out.az_next   = tgt_az;
        out.el_next   = tgt_el;
        out.status    = AVD_OK_DIRECT;
        state->active = 0;
        state->path_valid = 0;
        state->greedy_wp_count = 0;
        clear_history(state);
        return out;
    }

    /* ---- 4. If following a cached A* path, continue it ---- */
    if (state->path_valid) {
        if (avd_manhattan(tgt_az, tgt_el,
                          state->path_tgt_az, state->path_tgt_el, wrap)
            > AVD_WP_REACH_THRESH) {
            state->path_valid = 0;
        }
    }

    if (state->path_valid && state->path_idx < state->path_len) {
        avd_real wp_az = state->path_az[state->path_idx];
        avd_real wp_el = state->path_el[state->path_idx];

        /* advance if current waypoint reached AND next is reachable */
        if (avd_manhattan(cur_az, cur_el, wp_az, wp_el, wrap)
            < AVD_WP_REACH_THRESH) {
            if (state->path_idx + 1 >= state->path_len) {
                state->path_valid = 0;   /* path complete */
            } else {
                avd_real nxt_az = state->path_az[state->path_idx + 1];
                avd_real nxt_el = state->path_el[state->path_idx + 1];
                if (path_is_clear(cur_az, cur_el, nxt_az, nxt_el,
                                  f, prof, wrap)) {
                    state->path_idx++;
                    wp_az = nxt_az;
                    wp_el = nxt_el;
                }
            }
        }

        /* verify waypoint still reachable */
        if (state->path_valid && state->path_idx < state->path_len) {
            if (point_in_envelope(wp_az, wp_el, env) &&
                !point_in_any_forbidden(wp_az, wp_el, f) &&
                path_is_clear(cur_az, cur_el, wp_az, wp_el,
                              f, prof, wrap)) {
                out.az_next = wp_az;
                out.el_next = wp_el;
                out.status  = AVD_OK_WAYPOINT;
                return out;
            }
            state->path_valid = 0;
        }
    }

    /* ---- 5. GREEDY: check committed waypoint ---- */
    if (state->active) {
        avd_real dist = avd_manhattan(cur_az, cur_el,
                                      state->wp_az, state->wp_el, wrap);

        if (dist < AVD_WP_REACH_THRESH) {
            state->active = 0;
        } else if (point_in_envelope(state->wp_az, state->wp_el, env) &&
                   !point_in_any_forbidden(state->wp_az, state->wp_el, f) &&
                   path_is_clear(cur_az, cur_el,
                                 state->wp_az, state->wp_el, f,
                                 prof, wrap)) {
            out.az_next = state->wp_az;
            out.el_next = state->wp_el;
            out.status  = AVD_OK_WAYPOINT;
            return out;
        } else {
            state->active = 0;
        }
    }

    /* ---- 6. GREEDY: search for best waypoint ---- */
    {
        avd_real wp_az, wp_el;
        if (find_best_waypoint(cur_az, cur_el, tgt_az, tgt_el,
                               f, env, prof, wrap, state,
                               &wp_az, &wp_el)) {
            state->active = 1;
            state->wp_az  = wp_az;
            state->wp_el  = wp_el;
            add_to_history(wp_az, wp_el, state);
            state->greedy_wp_count++;

            /* Greedy escalation: trigger A* on precomputed graph */
            if (graph && state->greedy_wp_count >= AVD_GREEDY_LIMIT) {
                int src = nearest_node(cur_az, cur_el, graph, wrap);
                int dst = nearest_node(tgt_az, tgt_el, graph, wrap);
                int plen = astar_on_graph(src, dst, graph, wrap,
                                           state->path_az, state->path_el,
                                           AVD_MAX_PATH_LEN);
                if (plen > 0) {
                    state->path_len    = plen;
                    state->path_idx    = 0;
                    state->path_valid  = 1;
                    state->path_tgt_az = tgt_az;
                    state->path_tgt_el = tgt_el;
                    state->active      = 0;
                    state->greedy_wp_count = 0;
                    clear_history(state);

                    out.az_next = state->path_az[0];
                    out.el_next = state->path_el[0];
                    out.status  = AVD_OK_WAYPOINT;
                    return out;
                }
            }

            out.az_next   = wp_az;
            out.el_next   = wp_el;
            out.status    = AVD_OK_WAYPOINT;
            return out;
        }
    }

    /* ---- 7. Greedy failed → A* fallback ---- */
    if (graph) {
        clear_history(state);
        {
            int src = nearest_node(cur_az, cur_el, graph, wrap);
            int dst = nearest_node(tgt_az, tgt_el, graph, wrap);
            int plen = astar_on_graph(src, dst, graph, wrap,
                                       state->path_az, state->path_el,
                                       AVD_MAX_PATH_LEN);
            if (plen > 0) {
                state->path_len    = plen;
                state->path_idx    = 0;
                state->path_valid  = 1;
                state->path_tgt_az = tgt_az;
                state->path_tgt_el = tgt_el;
                state->active      = 0;
                state->greedy_wp_count = 0;

                out.az_next = state->path_az[0];
                out.el_next = state->path_el[0];
                out.status  = AVD_OK_WAYPOINT;
                return out;
            }
        }
    }

    /* ---- 8. Truly no path ---- */
    out.az_next = cur_az;
    out.el_next = cur_el;
    out.status  = AVD_NO_PATH;
    state->active = 0;
    state->path_valid = 0;
    return out;
}
