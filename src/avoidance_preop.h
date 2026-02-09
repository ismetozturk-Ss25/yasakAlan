/*
 * avoidance_preop.h — Pre-Op Visibility Graph Builder
 *
 * Simulink C S-Function Block: "Avoidance_PreOp"
 * Runs ONCE when forbidden zones are defined (time unconstrained).
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │  SIMULINK BLOCK I/O                                         │
 * ├──────────────────────────────────────────────────────────────┤
 * │  INPUTS (Simulink input ports):                             │
 * │    Port 1: forbidden[16x4]  (az_min,az_max,el_min,el_max   │
 * │            per zone)                  double[16][4]         │
 * │    Port 2: forbidden_count  (int, 0..16)        int32      │
 * │    Port 3: envelope[4]      (az_min,az_max,el_min,el_max)  │
 * │                                                double[4]   │
 * │    Port 4: motion_profile   (int, 0/1/2)        int32      │
 * │    Port 5: az_wrap          (int, 0/1)           int32      │
 * ├──────────────────────────────────────────────────────────────┤
 * │  OUTPUTS (Simulink output ports):                           │
 * │    Port 1: graph_nc         (int, node count)    int32      │
 * │    Port 2: graph_naz[128]   (node AZ coords)   double[128] │
 * │    Port 3: graph_nel[128]   (node EL coords)   double[128] │
 * │    Port 4: graph_adj[128x16](adjacency bytes)  uint8[2048] │
 * └──────────────────────────────────────────────────────────────┘
 */

#ifndef AVOIDANCE_PREOP_H
#define AVOIDANCE_PREOP_H

#include "avoidance_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Avoidance_BuildGraph
 *   Precomputes the visibility graph from forbidden zones.
 *   Call ONCE in pre-op mode (time unconstrained).
 *   Forbidden zones must NOT change after this call.
 *
 *   Computes: ~V^2 x N segment checks (V<=128, N<=16).
 *   Typical time: 1-10 ms on DSP, no deadline.
 */
void Avoidance_BuildGraph(AvdGraph *graph,
                          const AvdRect *forbidden, int forbidden_count,
                          const AvdRect *envelope,
                          AvdMotionProfile profile, int az_wrap);

#ifdef __cplusplus
}
#endif

#endif /* AVOIDANCE_PREOP_H */
