/*
 * benchmark.c -- Measures Pre-Op and Op-Mode execution times
 *
 * Two-phase timing:
 *   Phase 1: Avoidance_BuildGraph  (pre-op, runs once)
 *   Phase 2: Avoidance_Step        (op-mode, runs every 1 ms)
 *
 * Op-mode scenarios at different forbidden zone counts:
 *   1. N=0   (no zones, direct path — best case)
 *   2. N=1   (single obstacle, greedy waypoint routing)
 *   3. N=16  (worst case, maximum zones, greedy + A* fallback)
 *   4. N=16  (A* cached path follow — sustained worst case)
 *
 * Build (split modules):
 *   gcc -O2 -Wall -Wextra -pedantic -std=c99 -Isrc \
 *       src/avoidance_preop.c src/avoidance_opmode.c \
 *       test/benchmark.c -o benchmark.exe
 *
 * Build (original single file):
 *   gcc -O2 -Wall -Wextra -pedantic -std=c99 -Isrc \
 *       src/avoidance.c test/benchmark.c -o benchmark.exe
 */

#include <stdio.h>
#include <string.h>
#include <time.h>
#include "avoidance.h"

#define ITERATIONS  1000000

/* Portable timer using clock() */
static double get_time_sec(void)
{
    return (double)clock() / (double)CLOCKS_PER_SEC;
}

/* Prevent compiler from optimizing away the result */
static volatile float sink;

/* ================================================================
 *  PHASE 1: Pre-Op BuildGraph benchmark
 * ================================================================ */
static void bench_build_graph(const char *name,
                              const AvdRect *forbidden, int forbidden_count,
                              const AvdRect *envelope,
                              int az_wrap,
                              AvdGraph *graph_out)
{
    double t0, t1, elapsed, per_call_us;
    int i;
    int build_iters = 10000;

    /* First call to get the graph for later use */
    Avoidance_BuildGraph(graph_out, forbidden, forbidden_count,
                         envelope, AVD_MOTION_LINEAR, az_wrap);

    /* Timed runs */
    t0 = get_time_sec();
    for (i = 0; i < build_iters; i++) {
        AvdGraph tmp;
        Avoidance_BuildGraph(&tmp, forbidden, forbidden_count,
                             envelope, AVD_MOTION_LINEAR, az_wrap);
        sink = (float)tmp.nc;
    }
    t1 = get_time_sec();

    elapsed = t1 - t0;
    per_call_us = (elapsed / build_iters) * 1e6;

    printf("  [PRE-OP] %s\n", name);
    printf("    Forbidden zones : %d\n", forbidden_count);
    printf("    Graph nodes     : %d\n", graph_out->nc);
    printf("    Iterations      : %d\n", build_iters);
    printf("    Total time      : %.3f s\n", elapsed);
    printf("    Per call        : %.2f us  (%.3f ms)\n", per_call_us, per_call_us / 1000.0);
    printf("\n");
}

/* ================================================================
 *  PHASE 2: Op-Mode Step benchmark
 * ================================================================ */
static void bench_step(const char *name, AvdInput *in, int az_wrap,
                       const AvdGraph *graph)
{
    AvdState state;
    AvdOutput out;
    double t0, t1, elapsed, per_call_us, per_call_ns;
    int i;
    int status_counts[3] = {0, 0, 0};

    /* Save start position */
    float start_az = in->az_now;
    float start_el = in->el_now;

    Avoidance_Init(&state, AVD_MOTION_LINEAR, az_wrap);

    t0 = get_time_sec();

    for (i = 0; i < ITERATIONS; i++) {
        out = Avoidance_Step(in, &state, graph);
        sink = out.az_next + out.el_next;

        /* Count status distribution */
        if (out.status >= 0 && out.status <= 2)
            status_counts[out.status]++;

        /*
         * Reset position + state every 10 calls.
         * This forces the planner to re-evaluate from scratch,
         * exercising the full waypoint search / A* path each time
         * instead of just following a cached result.
         */
        if ((i + 1) % 10 == 0) {
            in->az_now = start_az;
            in->el_now = start_el;
            Avoidance_Init(&state, AVD_MOTION_LINEAR, az_wrap);
        }
    }

    t1 = get_time_sec();
    elapsed = t1 - t0;
    per_call_us = (elapsed / ITERATIONS) * 1e6;
    per_call_ns = (elapsed / ITERATIONS) * 1e9;

    printf("  [OP-MODE] %s\n", name);
    printf("    Forbidden zones : %d\n", in->forbidden_count);
    printf("    Graph provided  : %s\n", graph ? "YES" : "NO");
    printf("    Iterations      : %d\n", ITERATIONS);
    printf("    Total time      : %.3f s\n", elapsed);
    printf("    Per call        : %.2f us  (%.0f ns)\n", per_call_us, per_call_ns);
    printf("    Max call rate   : %.0f kHz\n", 1000.0 / per_call_us);
    printf("    1 kHz budget    : %.2f%%\n", (per_call_us / 1000.0) * 100.0);
    printf("    Fits 1ms DSP?   : %s\n", per_call_us < 1000.0 ? "YES" : "NO");
    printf("    Status counts   : DIRECT=%d  WAYPOINT=%d  NO_PATH=%d\n",
           status_counts[0], status_counts[1], status_counts[2]);
    printf("\n");
}

/* ================================================================
 *  MAIN
 * ================================================================ */
int main(void)
{
    AvdInput in;
    AvdGraph graph;
    AvdRect envelope;
    int i;

    printf("========================================================\n");
    printf("  Avoidance Pre-Op + Op-Mode Benchmark\n");
    printf("  Op-mode iterations: %d per scenario\n", ITERATIONS);
    printf("  Compiler: %s\n",
#ifdef __GNUC__
           "GCC " __VERSION__
#else
           "Unknown"
#endif
    );
    printf("========================================================\n\n");

    /* Common envelope */
    envelope.az_min = -180.0f;  envelope.az_max = 180.0f;
    envelope.el_min =  -30.0f;  envelope.el_max =  60.0f;

    /* ==============================================================
     *  PRE-OP: BuildGraph timing
     * ============================================================== */
    printf("--------------------------------------------------------\n");
    printf("  PHASE 1: Pre-Op  (Avoidance_BuildGraph)\n");
    printf("--------------------------------------------------------\n\n");

    /* Pre-op N=1 */
    {
        AvdRect f1[1];
        f1[0].az_min = -40.0f;  f1[0].az_max = 40.0f;
        f1[0].el_min = -15.0f;  f1[0].el_max = 15.0f;
        bench_build_graph("N=1  Single zone", f1, 1, &envelope, 1, &graph);
    }

    /* Pre-op N=16 */
    {
        AvdRect f16[16];
        for (i = 0; i < 16; i++) {
            float base_az = -160.0f + (float)(i % 4) * 85.0f;
            float base_el = -20.0f  + (float)(i / 4) * 20.0f;
            f16[i].az_min = base_az;
            f16[i].az_max = base_az + 20.0f;
            f16[i].el_min = base_el;
            f16[i].el_max = base_el + 10.0f;
        }
        bench_build_graph("N=16 Maximum zones", f16, 16, &envelope, 1, &graph);
    }

    /* ==============================================================
     *  OP-MODE: Step timing
     * ============================================================== */
    printf("--------------------------------------------------------\n");
    printf("  PHASE 2: Op-Mode  (Avoidance_Step @ 1 kHz)\n");
    printf("--------------------------------------------------------\n\n");

    /* Scenario 1: N=0, direct path, no graph */
    memset(&in, 0, sizeof(in));
    in.az_now =  150.0f;  in.el_now =   0.0f;
    in.az_cmd = -150.0f;  in.el_cmd =  30.0f;
    in.envelope = envelope;
    in.forbidden_count = 0;

    bench_step("N=0  Direct path (best case)", &in, 1, NULL);

    /* Scenario 2: N=1, greedy waypoint, no graph */
    memset(&in, 0, sizeof(in));
    in.az_now = -100.0f;  in.el_now =  0.0f;
    in.az_cmd =  100.0f;  in.el_cmd =  0.0f;
    in.envelope = envelope;
    in.forbidden_count = 1;
    in.forbidden[0].az_min = -40.0f;  in.forbidden[0].az_max = 40.0f;
    in.forbidden[0].el_min = -15.0f;  in.forbidden[0].el_max = 15.0f;

    bench_step("N=1  Greedy waypoint (no graph)", &in, 1, NULL);

    /* Scenario 3: N=1, with graph (A* available) */
    {
        AvdGraph g1;
        Avoidance_BuildGraph(&g1, in.forbidden, in.forbidden_count,
                             &envelope, AVD_MOTION_LINEAR, 1);

        memset(&in, 0, sizeof(in));
        in.az_now = -100.0f;  in.el_now =  0.0f;
        in.az_cmd =  100.0f;  in.el_cmd =  0.0f;
        in.envelope = envelope;
        in.forbidden_count = 1;
        in.forbidden[0].az_min = -40.0f;  in.forbidden[0].az_max = 40.0f;
        in.forbidden[0].el_min = -15.0f;  in.forbidden[0].el_max = 15.0f;

        bench_step("N=1  With graph (A* available)", &in, 1, &g1);
    }

    /*
     * Scenario 4: N=1, path truly BLOCKED
     * Zone spans full EL range, forcing waypoint around AZ.
     * Start and target at same EL=0, zone blocks AZ=-10..+10 across full EL.
     * path_is_clear checks linear + two L-shape legs, all hit the zone.
     */
    memset(&in, 0, sizeof(in));
    in.az_now = -60.0f;  in.el_now =  0.0f;
    in.az_cmd =  60.0f;  in.el_cmd =  0.0f;
    in.envelope = envelope;
    in.forbidden_count = 1;
    in.forbidden[0].az_min = -10.0f;  in.forbidden[0].az_max = 10.0f;
    in.forbidden[0].el_min = -29.0f;  in.forbidden[0].el_max = 59.0f;

    bench_step("N=1  Full-height wall (greedy only)", &in, 1, NULL);

    /* Scenario 5: Same wall but with graph */
    {
        AvdGraph g1b;
        Avoidance_BuildGraph(&g1b, in.forbidden, in.forbidden_count,
                             &envelope, AVD_MOTION_LINEAR, 1);

        in.az_now = -60.0f;  in.el_now =  0.0f;
        in.az_cmd =  60.0f;  in.el_cmd =  0.0f;

        bench_step("N=1  Full-height wall (with graph)", &in, 1, &g1b);
    }

    /*
     * Scenario 6: N=4, maze-like layout forcing multi-waypoint
     * Two horizontal walls + two vertical walls creating a maze.
     */
    memset(&in, 0, sizeof(in));
    in.az_now = -150.0f;  in.el_now = -20.0f;
    in.az_cmd =  150.0f;  in.el_cmd =  40.0f;
    in.envelope = envelope;
    in.forbidden_count = 4;
    /* Vertical wall left */
    in.forbidden[0].az_min = -60.0f;  in.forbidden[0].az_max = -40.0f;
    in.forbidden[0].el_min = -29.0f;  in.forbidden[0].el_max =  40.0f;
    /* Vertical wall right */
    in.forbidden[1].az_min =  40.0f;  in.forbidden[1].az_max =  60.0f;
    in.forbidden[1].el_min = -10.0f;  in.forbidden[1].el_max =  59.0f;
    /* Horizontal wall top */
    in.forbidden[2].az_min = -40.0f;  in.forbidden[2].az_max =  40.0f;
    in.forbidden[2].el_min =  30.0f;  in.forbidden[2].el_max =  50.0f;
    /* Horizontal wall bottom */
    in.forbidden[3].az_min = -40.0f;  in.forbidden[3].az_max =  40.0f;
    in.forbidden[3].el_min = -20.0f;  in.forbidden[3].el_max =   0.0f;

    {
        AvdGraph gmaze;
        Avoidance_BuildGraph(&gmaze, in.forbidden, in.forbidden_count,
                             &envelope, AVD_MOTION_LINEAR, 1);

        bench_step("N=4  Maze layout (with graph)", &in, 1, &gmaze);
    }

    /* Scenario 7: N=16, scattered zones */
    memset(&in, 0, sizeof(in));
    in.az_now = -170.0f;  in.el_now = -25.0f;
    in.az_cmd =  170.0f;  in.el_cmd =  55.0f;
    in.envelope = envelope;
    in.forbidden_count = 16;
    for (i = 0; i < 16; i++) {
        float base_az = -160.0f + (float)(i % 4) * 85.0f;
        float base_el = -20.0f  + (float)(i / 4) * 20.0f;
        in.forbidden[i].az_min = base_az;
        in.forbidden[i].az_max = base_az + 20.0f;
        in.forbidden[i].el_min = base_el;
        in.forbidden[i].el_max = base_el + 10.0f;
    }

    bench_step("N=16 Scattered (greedy only)", &in, 1, NULL);

    /* Scenario 8: N=16, with precomputed graph */
    {
        AvdGraph g16;
        Avoidance_BuildGraph(&g16, in.forbidden, in.forbidden_count,
                             &envelope, AVD_MOTION_LINEAR, 1);

        in.az_now = -170.0f;  in.el_now = -25.0f;
        in.az_cmd =  170.0f;  in.el_cmd =  55.0f;

        bench_step("N=16 Scattered (with graph)", &in, 1, &g16);
    }

    /*
     * Scenario 9: N=16, dense staggered wall (worst case)
     * Alternating tall/short zones forming a wall across AZ.
     * Forces complex waypoint routing through narrow gaps.
     */
    {
        AvdGraph gwall;
        memset(&in, 0, sizeof(in));
        in.az_now = -170.0f;  in.el_now =  0.0f;
        in.az_cmd =  170.0f;  in.el_cmd =  0.0f;
        in.envelope = envelope;
        in.forbidden_count = 16;
        for (i = 0; i < 16; i++) {
            float base_az = -160.0f + (float)i * 20.0f;
            in.forbidden[i].az_min = base_az;
            in.forbidden[i].az_max = base_az + 18.0f;
            /* Alternate tall and short to create staggered gaps */
            if (i % 2 == 0) {
                in.forbidden[i].el_min = -29.0f;
                in.forbidden[i].el_max =  40.0f;
            } else {
                in.forbidden[i].el_min = -10.0f;
                in.forbidden[i].el_max =  59.0f;
            }
        }

        Avoidance_BuildGraph(&gwall, in.forbidden, in.forbidden_count,
                             &envelope, AVD_MOTION_LINEAR, 1);

        bench_step("N=16 Staggered wall (worst case)", &in, 1, &gwall);
    }

    /* ==============================================================
     *  Summary
     * ============================================================== */
    printf("========================================================\n");
    printf("  DSP target  : 1 kHz (1000 us budget per Step call)\n");
    printf("  Pre-op      : runs once, no deadline\n");
    printf("  Op-mode     : must complete within 1 ms\n");
    printf("  Algorithm   : no trig, no sqrt, no malloc\n");
    printf("========================================================\n");

    return 0;
}
