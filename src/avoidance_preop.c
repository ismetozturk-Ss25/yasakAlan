/*
 * avoidance_preop.c — Pre-Op Visibility Graph Builder
 *
 * Builds visibility graph from forbidden zones.
 * All V^2 x N geometry checks done here (time unconstrained).
 *
 * This file compiles independently:
 *   gcc -c avoidance_preop.c
 *
 * Define AVD_DEBUG_PRINT to enable debug output (zones, nodes, adjacency).
 *   Output goes to both console AND preop_debug.txt.
 * Disable for Simulink / embedded targets.
 *
 * No dependencies on avoidance_opmode.c.
 */

#include "avoidance_simulink.h"

#ifdef AVD_DEBUG_PRINT
#include <stdio.h>
#include <stdarg.h>

static FILE *g_dbg_file = NULL;

static void dbg_print(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    if (g_dbg_file) {
        va_start(args, fmt);
        vfprintf(g_dbg_file, fmt, args);
        va_end(args);
    }
}
#endif

/* ==============================================================
 *  BIT-PACKED  ADJACENCY  HELPER  (local to preop)
 * ============================================================== */

static void adj_set(unsigned char adj[][AVD_ADJ_BYTES], int i, int j)
{
    adj[i][j >> 3] |= (unsigned char)(1u << (j & 7));
}

#ifdef AVD_DEBUG_PRINT
static int adj_get_local(unsigned char adj[][AVD_ADJ_BYTES], int i, int j)
{
    return (adj[i][j >> 3] >> (j & 7)) & 1;
}
#endif

/* ==============================================================
 *  PRECOMPUTED VISIBILITY GRAPH  (pre-op mode)
 *
 *  Generates candidate nodes at zone corners + edge midpoints,
 *  then tests all-pairs visibility using Liang-Barsky.
 *  Result: bit-packed adjacency matrix (~2 KB for 128 nodes).
 *
 *  Time complexity: O(V^2 x N)  where V<=128, N<=16
 *  Typical time: 1-10 ms.  Called ONCE, no deadline.
 * ============================================================== */

void Avoidance_BuildGraph(AvdGraph *graph,
                          const AvdRect *forbidden,
                          const AvdRect *envelope,
                          AvdMotionProfile profile, int az_wrap)
{
    int i, c, j;
    int nc = 0;

    /* Wrap-envelope local copies (used only when az_min > az_max) */
    AvdRect local_f[AVD_MAX_FORBIDDEN];
    AvdRect local_env;

    /* If envelope is not valid, produce empty graph */
    if (!envelope->valid) {
        graph->nc = 0;
        return;
    }

    /* Handle wrap-around scenarios when az_wrap is enabled:
     * 1. Wrap-around envelope (az_min > az_max): add gap [az_max, az_min]
     *    as forbidden zone, expand envelope to [-180, 180].
     * 2. Wrap-around forbidden zones (az_min > az_max): split into two
     *    non-wrapping zones at the +/-180 boundary so Liang-Barsky works.
     * Both must be resolved before node generation and visibility checks. */
    if (az_wrap) {
        int need_local = 0;
        if (envelope->az_min > envelope->az_max) need_local = 1;
        if (!need_local) {
            for (i = 0; i < AVD_MAX_FORBIDDEN; i++) {
                if (forbidden[i].valid && forbidden[i].az_min > forbidden[i].az_max) {
                    need_local = 1; break;
                }
            }
        }
        if (need_local) {
            int s;
            for (i = 0; i < AVD_MAX_FORBIDDEN; i++)
                local_f[i] = forbidden[i];
            local_env = *envelope;

            /* 1. Wrap-around envelope → add gap, expand to [-180,180] */
            if (local_env.az_min > local_env.az_max) {
                for (i = 0; i < AVD_MAX_FORBIDDEN; i++) {
                    if (!local_f[i].valid) {
                        local_f[i].valid  = 1;
                        local_f[i].az_min = local_env.az_max;
                        local_f[i].az_max = local_env.az_min;
                        local_f[i].el_min = local_env.el_min;
                        local_f[i].el_max = local_env.el_max;
                        break;
                    }
                }
                local_env.az_min = -180.0f;
                local_env.az_max =  180.0f;
            }

            /* 2. Split wrapped forbidden zones at +/-180 boundary */
            for (i = 0; i < AVD_MAX_FORBIDDEN; i++) {
                avd_real orig_max;
                if (!local_f[i].valid) continue;
                if (local_f[i].az_min <= local_f[i].az_max) continue;
                /* AZ[az_min, az_max] with az_min>az_max → wraps ±180
                 * Split: Zone A [az_min, 180], Zone B [-180, az_max] */
                orig_max = local_f[i].az_max;
                local_f[i].az_max = 180.0f;
                for (s = 0; s < AVD_MAX_FORBIDDEN; s++) {
                    if (!local_f[s].valid) {
                        local_f[s].valid  = 1;
                        local_f[s].az_min = -180.0f;
                        local_f[s].el_min = local_f[i].el_min;
                        local_f[s].az_max = orig_max;
                        local_f[s].el_max = local_f[i].el_max;
                        break;
                    }
                }
            }

            forbidden = local_f;
            envelope  = &local_env;
        }
    }

#ifdef AVD_DEBUG_PRINT
    g_dbg_file = fopen("preop_debug.txt", "a");
    /* ---- print forbidden zones ---- */
    dbg_print("============================================================\n");
    dbg_print("  Forbidden zones:\n");
    dbg_print("============================================================\n");
    for (i = 0; i < AVD_MAX_FORBIDDEN; i++) {
        if (!forbidden[i].valid) continue;
        dbg_print("  [%2d] AZ[%7.1f, %7.1f]  EL[%5.1f, %5.1f]\n",
               i, (double)forbidden[i].az_min, (double)forbidden[i].az_max,
               (double)forbidden[i].el_min, (double)forbidden[i].el_max);
    }
#endif

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
     * These nodes ensure graph connectivity across long AZ spans
     * where zone corners may be far apart.  We place nodes at
     * regular AZ intervals along the envelope EL boundaries and
     * midpoint.  Only nodes that are NOT inside any forbidden zone
     * are kept.
     */
    {
        avd_real sweep_az;
        avd_real sweep_el[3];
        int ne, ns;
        avd_real az_step = 30.0f;  /* one node every 30 degrees */

        sweep_el[0] = envelope->el_min + AVD_CORNER_EPS;  /* bottom edge */
        sweep_el[1] = envelope->el_max - AVD_CORNER_EPS;  /* top edge    */
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

#ifdef AVD_DEBUG_PRINT
    dbg_print("\n============================================================\n");
    dbg_print("  Graph: %d nodes\n", nc);
    dbg_print("============================================================\n");
    for (i = 0; i < nc; i++) {
        dbg_print("  Node %3d: AZ=%7.1f  EL=%5.1f\n",
               i, (double)graph->naz[i], (double)graph->nel[i]);
    }
    dbg_print("\nBuilding adjacency (all-pairs visibility)...\n");
#endif

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
                              forbidden,
                              profile, az_wrap)) {
                adj_set(graph->adj, i, j);
                adj_set(graph->adj, j, i);
            }
        }
    }

#ifdef AVD_DEBUG_PRINT
    {
        int edge_count = 0;
        for (i = 0; i < nc; i++) {
            for (j = i + 1; j < nc; j++) {
                if (adj_get_local(graph->adj, i, j))
                    edge_count++;
            }
        }
        dbg_print("  %d edges\n", edge_count);

        dbg_print("\n============================================================\n");
        dbg_print("  OUTPUT SUMMARY\n");
        dbg_print("============================================================\n");
        dbg_print("  graph_nc  = %d\n", nc);
        dbg_print("  graph_naz = float [1x%d]  (%d valid)\n", AVD_MAX_FIXED_NODES, nc);
        dbg_print("  graph_nel = float [1x%d]  (%d valid)\n", AVD_MAX_FIXED_NODES, nc);
        dbg_print("  graph_adj = uint8 [1x%d] (%d edges)\n",
               AVD_MAX_FIXED_NODES * AVD_ADJ_BYTES, edge_count);

        dbg_print("\n============================================================\n");
        dbg_print("  ADJACENCY LIST\n");
        dbg_print("============================================================\n");
        for (i = 0; i < nc; i++) {
            dbg_print("  Node %2d (%7.1f,%5.1f) -> [",
                   i, (double)graph->naz[i], (double)graph->nel[i]);
            for (j = 0; j < nc; j++) {
                if (adj_get_local(graph->adj, i, j))
                    dbg_print(" %d", j);
            }
            dbg_print(" ]\n");
        }

        if (g_dbg_file) {
            fclose(g_dbg_file);
            g_dbg_file = NULL;
        }
    }
#endif
}
