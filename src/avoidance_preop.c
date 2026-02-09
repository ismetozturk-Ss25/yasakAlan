/*
 * avoidance_preop.c — Pre-Op Visibility Graph Builder
 *
 * Builds visibility graph from forbidden zones.
 * All V^2 x N geometry checks done here (time unconstrained).
 *
 * This file compiles independently:
 *   gcc -c avoidance_preop.c
 *
 * No dependencies on avoidance_opmode.c.
 */

#include "avoidance_simulink.h"

/* ==============================================================
 *  BIT-PACKED  ADJACENCY  HELPER  (local to preop)
 * ============================================================== */

static void adj_set(unsigned char adj[][AVD_ADJ_BYTES], int i, int j)
{
    adj[i][j >> 3] |= (unsigned char)(1u << (j & 7));
}

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
                          const AvdRect *forbidden, int forbidden_count,
                          const AvdRect *envelope,
                          AvdMotionProfile profile, int az_wrap)
{
    int i, c, j;
    int nc = 0;

    /* ---- generate candidate nodes ---- */
    for (i = 0; i < forbidden_count && nc < AVD_MAX_FIXED_NODES; i++) {
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
            if (point_in_any_forbidden(waz, wel, forbidden, forbidden_count)) continue;
            graph->naz[nc] = waz;
            graph->nel[nc] = wel;
            nc++;
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
                              forbidden, forbidden_count,
                              profile, az_wrap)) {
                adj_set(graph->adj, i, j);
                adj_set(graph->adj, j, i);
            }
        }
    }
}
