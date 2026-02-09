/*
 * avoidance_opmode.c — Op-Mode Avoidance Step Function
 *
 * Real-time path planner called every 1 ms.
 *   1. Greedy one-step planner (fast, ~2 700 ops)
 *   2. If greedy oscillates -> A* on precomputed graph
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
                           const AvdRect *f, int n,
                           const AvdRect *env,
                           int az_wrap)
{
    int i;
    for (i = 0; i < n; i++) {
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
                      const AvdRect *f, int n,
                      const AvdRect *env,
                      int az_wrap,
                      AvdOutput *out)
{
    int i;
    for (i = 0; i < n; i++) {
        if (!point_in_rect(cur_az, cur_el, &f[i])) continue;

        avd_real d_left  = avd_abs(cur_az - f[i].az_min);
        avd_real d_right = avd_abs(cur_az - f[i].az_max);
        avd_real d_bot   = avd_abs(cur_el - f[i].el_min);
        avd_real d_top   = avd_abs(cur_el - f[i].el_max);

        avd_real best = d_left;
        out->az_next = f[i].az_min - AVD_CORNER_EPS;
        out->el_next = cur_el;

        if (d_right < best) {
            best = d_right;
            out->az_next = f[i].az_max + AVD_CORNER_EPS;
            out->el_next = cur_el;
        }
        if (d_bot < best) {
            best = d_bot;
            out->az_next = cur_az;
            out->el_next = f[i].el_min - AVD_CORNER_EPS;
        }
        if (d_top < best) {
            out->az_next = cur_az;
            out->el_next = f[i].el_max + AVD_CORNER_EPS;
        }

        out->az_next = avd_clamp(out->az_next, env->az_min, env->az_max);
        out->el_next = avd_clamp(out->el_next, env->el_min, env->el_max);
        if (az_wrap) out->az_next = avd_normalize_az(out->az_next);
        out->status  = AVD_OK_WAYPOINT;
        return 1;
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
                              const AvdRect *f, int n,
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

    for (i = 0; i < n; i++) {
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
            if (point_in_any_forbidden(waz, wel, f, n)) continue;
            if (!path_is_clear(cur_az, cur_el, waz, wel, f, n,
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
                                        f, n, profile, az_wrap);

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
 *  BIT-PACKED  ADJACENCY  HELPER  (read-only, local to opmode)
 * ============================================================== */

static int adj_get(const unsigned char adj[][AVD_ADJ_BYTES], int i, int j)
{
    return (adj[i][j >> 3] >> (j & 7)) & 1;
}

/* ==============================================================
 *  NEAREST  GRAPH  NODE  (Manhattan distance, O(V))
 *
 *  Snaps a position to the closest precomputed graph node.
 *  Cost: V comparisons = ~80 x 3 ops = ~240 ops.
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
 *    Find 2 nearest nodes: 2 x V x 3 ops     = ~480 ops
 *    A* search:            V x V bit-lookups   = ~6 400 ops
 *    Manhattan costs:      V x V x 3 ops       = ~19 200 ops
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
    int n              = in->forbidden_count;
    AvdMotionProfile prof = state->profile;
    int wrap           = state->az_wrap;

    avd_real cur_az = in->az_now;
    avd_real cur_el = in->el_now;

    /* ---- 0. Escape if currently inside a forbidden zone ---- */
    if (try_escape(cur_az, cur_el, f, n, env, wrap, &out)) {
        state->active = 0;
        state->path_valid = 0;
        return out;
    }

    /* ---- 1. Clamp target to envelope ---- */
    avd_real tgt_az = avd_clamp(in->az_cmd, env->az_min, env->az_max);
    avd_real tgt_el = avd_clamp(in->el_cmd, env->el_min, env->el_max);
    if (wrap) tgt_az = avd_normalize_az(tgt_az);

    /* ---- 2. Project target out of forbidden zones ---- */
    project_target(&tgt_az, &tgt_el, f, n, env, wrap);

    /* ---- 3. Direct path clear -> OK_DIRECT ---- */
    if (path_is_clear(cur_az, cur_el, tgt_az, tgt_el, f, n, prof, wrap)) {
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
                                  f, n, prof, wrap)) {
                    state->path_idx++;
                    wp_az = nxt_az;
                    wp_el = nxt_el;
                }
            }
        }

        /* verify waypoint still reachable */
        if (state->path_valid && state->path_idx < state->path_len) {
            if (point_in_envelope(wp_az, wp_el, env) &&
                !point_in_any_forbidden(wp_az, wp_el, f, n) &&
                path_is_clear(cur_az, cur_el, wp_az, wp_el,
                              f, n, prof, wrap)) {
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
                   !point_in_any_forbidden(state->wp_az, state->wp_el, f, n) &&
                   path_is_clear(cur_az, cur_el,
                                 state->wp_az, state->wp_el, f, n,
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
                               f, n, env, prof, wrap, state,
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

    /* ---- 7. Greedy failed -> A* fallback ---- */
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
