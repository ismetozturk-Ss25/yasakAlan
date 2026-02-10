/*
 * avoidance_opmode.c — Op-Mode Avoidance Step Function
 *
 * Real-time path planner called every 1 ms.
 * Algorithm (5 steps):
 *   1. Escape if inside forbidden zone (safety net)
 *   2. Clamp + project target out of forbidden zones
 *   3. Direct path clear? → OK_DIRECT, clear cache
 *   4. A* path: compute or follow cached path
 *   5. No path found → hold position
 *
 * This file compiles independently:
 *   gcc -c avoidance_opmode.c
 *
 * No dependencies on avoidance_preop.c.
 */

#include "avoidance_simulink.h"

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

        /* Sort by distance (insertion sort on 4 elements) */
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
 *  BIT-PACKED  ADJACENCY  HELPER  (read-only, local to opmode)
 * ============================================================== */

static int adj_get(const unsigned char adj[][AVD_ADJ_BYTES], int i, int j)
{
    return (adj[i][j >> 3] >> (j & 7)) & 1;
}

/* ==============================================================
 *  NEAREST  REACHABLE  GRAPH  NODE
 *
 *  Find closest node that has path_is_clear from given position.
 *  Falls back to closest-by-distance if no reachable node found.
 *  O(V) calls to path_is_clear.
 * ============================================================== */

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

        /* Track closest overall (fallback) */
        if (d < best_any_d) {
            best_any_d = d;
            best_any = i;
        }

        /* Check reachability */
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

/* ==============================================================
 *  A*  ON  PRECOMPUTED  GRAPH  —  ZERO GEOMETRY AT RUNTIME
 *
 *  Takes source and destination as graph NODE INDICES.
 *  All adjacency is precomputed — only array lookups during search.
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
    state->profile     = profile;
    state->az_wrap     = az_wrap;
    state->path_len    = 0;
    state->path_idx    = 0;
    state->path_valid  = 0;
    state->path_tgt_az = 0.0f;
    state->path_tgt_el = 0.0f;
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

    /* ---- Step 1: Escape if currently inside a forbidden zone ---- */
    if (try_escape(cur_az, cur_el, f, env, wrap, &out)) {
        state->path_valid = 0;
        return out;
    }

    /* ---- Step 2: Clamp + project target ---- */
    avd_real tgt_az = avd_clamp(in->az_cmd, env->az_min, env->az_max);
    avd_real tgt_el = avd_clamp(in->el_cmd, env->el_min, env->el_max);
    if (wrap) tgt_az = avd_normalize_az(tgt_az);

    project_target(&tgt_az, &tgt_el, f, env, wrap);

    /* ---- Step 3: Direct path clear -> OK_DIRECT ---- */
    if (path_is_clear(cur_az, cur_el, tgt_az, tgt_el, f, prof, wrap)) {
        out.az_next   = tgt_az;
        out.el_next   = tgt_el;
        out.status    = AVD_OK_DIRECT;
        state->path_valid = 0;
        return out;
    }

    /* ---- Step 4: A* path ---- */

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

        /* Advance if current waypoint reached */
        if (avd_manhattan(cur_az, cur_el, wp_az, wp_el, wrap)
            < AVD_WP_REACH_THRESH) {
            state->path_idx++;
            if (state->path_idx >= state->path_len) {
                state->path_valid = 0;
                /* Fall through: re-check direct or recompute next call */
            } else {
                wp_az = state->path_az[state->path_idx];
                wp_el = state->path_el[state->path_idx];
            }
        }

        /* At each waypoint, try shortcut direct to target */
        if (state->path_valid && state->path_idx < state->path_len) {
            if (path_is_clear(cur_az, cur_el, tgt_az, tgt_el,
                              f, prof, wrap)) {
                out.az_next   = tgt_az;
                out.el_next   = tgt_el;
                out.status    = AVD_OK_DIRECT;
                state->path_valid = 0;
                return out;
            }

            /* Verify waypoint still reachable */
            if (path_is_clear(cur_az, cur_el, wp_az, wp_el,
                              f, prof, wrap) &&
                point_in_envelope(wp_az, wp_el, env) &&
                !point_in_any_forbidden(wp_az, wp_el, f)) {
                out.az_next = wp_az;
                out.el_next = wp_el;
                out.status  = AVD_OK_WAYPOINT;
                return out;
            }

            /* Waypoint became unreachable — invalidate and recompute */
            state->path_valid = 0;
        }
    }

    /* If path just got invalidated, try one more A* computation */
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

    /* ---- Step 5: No path found — hold position ---- */
    out.az_next = cur_az;
    out.el_next = cur_el;
    out.status  = AVD_NO_PATH;
    state->path_valid = 0;
    return out;
}
