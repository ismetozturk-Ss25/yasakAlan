/*
 * avoidance_opmode.h — Op-Mode Avoidance Step Function
 *
 * Simulink C S-Function Block: "Avoidance_OpMode"
 * Runs every 1 ms sample step.
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │  SIMULINK BLOCK I/O                                         │
 * ├──────────────────────────────────────────────────────────────┤
 * │  INPUTS (Simulink input ports):                             │
 * │    Port  1: az_now           (current AZ position) double   │
 * │    Port  2: el_now           (current EL position) double   │
 * │    Port  3: az_cmd           (commanded AZ target) double   │
 * │    Port  4: el_cmd           (commanded EL target) double   │
 * │    Port  5: envelope[4]      (az_min,az_max,el_min,el_max) │
 * │                                                double[4]   │
 * │    Port  6: forbidden[16x4]  (az_min,az_max,el_min,el_max  │
 * │             per zone)                         double[64]   │
 * │    Port  7: forbidden_count  (int)              int32      │
 * │    Port  8: graph_nc         (int)              int32      │
 * │    Port  9: graph_naz[128]   (node AZ coords) double[128] │
 * │    Port 10: graph_nel[128]   (node EL coords) double[128] │
 * │    Port 11: graph_adj[128x16](adjacency bytes) uint8[2048]│
 * ├──────────────────────────────────────────────────────────────┤
 * │  OUTPUTS (Simulink output ports):                           │
 * │    Port 1: az_next  (next AZ command)            double    │
 * │    Port 2: el_next  (next EL command)            double    │
 * │    Port 3: status   (int: 0=DIRECT,1=WP,2=NO)   int32     │
 * ├──────────────────────────────────────────────────────────────┤
 * │  PERSISTENT STATE (Simulink DWork / persistent memory):     │
 * │    AvdState (entire struct, ~600 bytes)                     │
 * │    Initialized via Avoidance_Init() at t=0                  │
 * └──────────────────────────────────────────────────────────────┘
 */

#ifndef AVOIDANCE_OPMODE_H
#define AVOIDANCE_OPMODE_H

#include "avoidance_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Avoidance_Init
 *   Resets internal state. Must be called once before the first Step.
 */
void Avoidance_Init(AvdState *state, AvdMotionProfile profile, int az_wrap);

/*
 * Avoidance_Step
 *   Executes one planning cycle (call every 1 ms).
 *
 *   graph — precomputed visibility graph from Avoidance_BuildGraph.
 *           If NULL, only greedy planner is used (no A* fallback).
 *           If provided, A* fallback uses the precomputed graph.
 */
AvdOutput Avoidance_Step(const AvdInput *in, AvdState *state,
                         const AvdGraph *graph);

#ifdef __cplusplus
}
#endif

#endif /* AVOIDANCE_OPMODE_H */
