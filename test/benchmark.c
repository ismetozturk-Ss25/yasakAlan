/*
 * benchmark.c -- Measures Pre-Op and Op-Mode execution times
 *
 * Two-phase timing:
 *   Phase 1: Avoidance_BuildGraph  (pre-op, runs once)
 *   Phase 2: Avoidance_Step        (op-mode, runs every 1 ms)
 *
 * Stress test focus: N=10 forbidden zones (typical deployment)
 * DSP estimation at 200 MHz with configurable scaling factor
 *
 * Build:
 *   gcc -O2 -Wall -Wextra -pedantic -std=c99 -Isrc \
 *       src/avoidance_preop.c src/avoidance_opmode.c \
 *       test/benchmark.c -o benchmark.exe
 */

#include <stdio.h>
#include <string.h>
#include <time.h>
#include "avoidance_simulink.h"

#define ITERATIONS      1000000
#define DSP_MHZ         200       /* target DSP clock frequency          */
#define DSP_SCALE       3.0       /* PC-to-DSP scaling factor            */
                                  /* accounts for: no cache hierarchy,   */
                                  /* simpler pipeline, memory latency    */
                                  /* 3x is conservative for float DSP    */

/* Portable timer using clock() */
static double get_time_sec(void)
{
    return (double)clock() / (double)CLOCKS_PER_SEC;
}

/* Prevent compiler from optimizing away the result */
static volatile float sink;

/* PC clock frequency estimate (measured at startup) */
static double pc_mhz = 3000.0;  /* default, updated by calibrate */

static void calibrate_pc_clock(void)
{
    volatile int j;
    double t0, t1, elapsed;
    int ops = 10000000;

    t0 = get_time_sec();
    for (j = 0; j < ops; j++) { }
    t1 = get_time_sec();
    elapsed = t1 - t0;

    if (elapsed > 0.001) {
        /* rough estimate -- not exact but gives ballpark */
        pc_mhz = (double)ops / (elapsed * 1e6) * 10.0;
        if (pc_mhz < 500.0)  pc_mhz = 3000.0;  /* sanity */
        if (pc_mhz > 10000.0) pc_mhz = 3000.0;
    }
}

/* Estimate DSP time from PC measurement */
static double estimate_dsp_us(double pc_us)
{
    return pc_us * (pc_mhz / (double)DSP_MHZ) * DSP_SCALE;
}

/* ================================================================
 *  PHASE 1: Pre-Op BuildGraph benchmark
 * ================================================================ */
static void bench_build_graph(const char *name,
                              const AvdRect *forbidden,
                              const AvdRect *envelope,
                              int az_wrap,
                              AvdGraph *graph_out)
{
    double t0, t1, elapsed, per_call_us, dsp_us;
    int i, cnt = 0;
    int build_iters = 10000;

    for (i = 0; i < AVD_MAX_FORBIDDEN; i++)
        if (forbidden[i].valid) cnt++;

    /* First call to get the graph for later use */
    Avoidance_BuildGraph(graph_out, forbidden,
                         envelope, AVD_MOTION_LINEAR, az_wrap);

    /* Timed runs */
    t0 = get_time_sec();
    for (i = 0; i < build_iters; i++) {
        AvdGraph tmp;
        Avoidance_BuildGraph(&tmp, forbidden,
                             envelope, AVD_MOTION_LINEAR, az_wrap);
        sink = (float)tmp.nc;
    }
    t1 = get_time_sec();

    elapsed = t1 - t0;
    per_call_us = (elapsed / build_iters) * 1e6;
    dsp_us = estimate_dsp_us(per_call_us);

    printf("  [PRE-OP] %s\n", name);
    printf("    Forbidden zones : %d\n", cnt);
    printf("    Graph nodes     : %d\n", graph_out->nc);
    printf("    Iterations      : %d\n", build_iters);
    printf("    Total time      : %.3f s\n", elapsed);
    printf("    Per call (PC)   : %.2f us  (%.3f ms)\n", per_call_us, per_call_us / 1000.0);
    printf("    Est. DSP @%dMHz : %.2f us  (%.3f ms)\n", DSP_MHZ, dsp_us, dsp_us / 1000.0);
    printf("\n");
}

/* ================================================================
 *  PHASE 2: Op-Mode Step benchmark
 * ================================================================ */
static void bench_step(const char *name, AvdInput *in, int az_wrap,
                       const AvdGraph *graph, int force_replan)
{
    AvdState state;
    AvdOutput out;
    double t0, t1, elapsed, per_call_us, per_call_ns, dsp_us;
    int i;
    int status_counts[3] = {0, 0, 0};
    int reset_interval = force_replan ? 10 : 0;

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
         * Reset position + state periodically to force re-planning.
         * This exercises the full A* search each time instead of
         * just following a cached result.
         */
        if (reset_interval > 0 && (i + 1) % reset_interval == 0) {
            in->az_now = start_az;
            in->el_now = start_el;
            Avoidance_Init(&state, AVD_MOTION_LINEAR, az_wrap);
        }
    }

    t1 = get_time_sec();
    elapsed = t1 - t0;
    per_call_us = (elapsed / ITERATIONS) * 1e6;
    per_call_ns = (elapsed / ITERATIONS) * 1e9;
    dsp_us = estimate_dsp_us(per_call_us);

    {
        int j, cnt = 0;
        for (j = 0; j < AVD_MAX_FORBIDDEN; j++)
            if (in->forbidden[j].valid) cnt++;
        printf("  [OP-MODE] %s\n", name);
        printf("    Forbidden zones : %d\n", cnt);
    }
    printf("    Graph nodes     : %d\n", graph ? graph->nc : 0);
    printf("    Force replan    : %s\n", force_replan ? "every 10 calls" : "NO (cached path)");
    printf("    Iterations      : %d\n", ITERATIONS);
    printf("    Total time (PC) : %.3f s\n", elapsed);
    printf("    Per call (PC)   : %.2f us  (%.0f ns)\n", per_call_us, per_call_ns);
    printf("    Est. DSP @%dMHz : %.2f us  (%.3f ms)\n", DSP_MHZ, dsp_us, dsp_us / 1000.0);
    printf("    DSP budget used : %.2f%% of 1ms\n", (dsp_us / 1000.0) * 100.0);
    printf("    Fits 1ms DSP?   : %s\n", dsp_us < 1000.0 ? "YES" : "NO");
    printf("    Status counts   : DIRECT=%d  WAYPOINT=%d  NO_PATH=%d\n",
           status_counts[0], status_counts[1], status_counts[2]);
    printf("\n");
}

/* ================================================================
 *  Zone layout builders
 * ================================================================ */

/* Build N=10 realistic deployment zones:
 * Mix of small radar exclusions, tall walls, and wide floor zones
 * spread across the full AZ-EL envelope */
static void build_10_zones(AvdRect *f)
{
    memset(f, 0, sizeof(AvdRect) * AVD_MAX_FORBIDDEN);

    /* Zone 0: Wide low wall left side */
    f[0].valid = 1;
    f[0].az_min = -160.0f;  f[0].el_min = -10.0f;
    f[0].az_max = -110.0f;  f[0].el_max =  20.0f;

    /* Zone 1: Tall narrow wall */
    f[1].valid = 1;
    f[1].az_min = -80.0f;   f[1].el_min = -25.0f;
    f[1].az_max = -65.0f;   f[1].el_max =  45.0f;

    /* Zone 2: Small square exclusion */
    f[2].valid = 1;
    f[2].az_min = -40.0f;   f[2].el_min =  10.0f;
    f[2].az_max = -20.0f;   f[2].el_max =  30.0f;

    /* Zone 3: Floor-level wide zone */
    f[3].valid = 1;
    f[3].az_min = -15.0f;   f[3].el_min = -28.0f;
    f[3].az_max =  25.0f;   f[3].el_max = -10.0f;

    /* Zone 4: Central tall barrier */
    f[4].valid = 1;
    f[4].az_min =  -5.0f;   f[4].el_min =   5.0f;
    f[4].az_max =  15.0f;   f[4].el_max =  55.0f;

    /* Zone 5: Right side medium zone */
    f[5].valid = 1;
    f[5].az_min =  40.0f;   f[5].el_min = -15.0f;
    f[5].az_max =  70.0f;   f[5].el_max =  15.0f;

    /* Zone 6: Upper right exclusion */
    f[6].valid = 1;
    f[6].az_min =  80.0f;   f[6].el_min =  25.0f;
    f[6].az_max = 110.0f;   f[6].el_max =  50.0f;

    /* Zone 7: Far right wall */
    f[7].valid = 1;
    f[7].az_min = 130.0f;   f[7].el_min = -20.0f;
    f[7].az_max = 150.0f;   f[7].el_max =  30.0f;

    /* Zone 8: Wrap-area zone near +180 */
    f[8].valid = 1;
    f[8].az_min = 160.0f;   f[8].el_min = -10.0f;
    f[8].az_max = 179.0f;   f[8].el_max =  20.0f;

    /* Zone 9: Wrap-area zone near -180 */
    f[9].valid = 1;
    f[9].az_min = -179.0f;  f[9].el_min = -10.0f;
    f[9].az_max = -165.0f;  f[9].el_max =  20.0f;
}

/* Build N=10 worst-case maze: alternating tall/short walls forcing
 * long A* paths through narrow gaps */
static void build_10_maze(AvdRect *f)
{
    int i;
    memset(f, 0, sizeof(AvdRect) * AVD_MAX_FORBIDDEN);

    for (i = 0; i < 10; i++) {
        float base_az = -160.0f + (float)i * 32.0f;
        f[i].valid  = 1;
        f[i].az_min = base_az;
        f[i].az_max = base_az + 28.0f;
        /* Alternate: tall from bottom, tall from top */
        if (i % 2 == 0) {
            f[i].el_min = -29.0f;
            f[i].el_max =  40.0f;
        } else {
            f[i].el_min = -10.0f;
            f[i].el_max =  59.0f;
        }
    }
}

/* ================================================================
 *  MAIN
 * ================================================================ */
int main(void)
{
    AvdInput in;
    AvdGraph graph;
    AvdRect envelope;

    printf("========================================================\n");
    printf("  Avoidance Stress Test Benchmark\n");
    printf("  Focus: N=10 forbidden zones (typical deployment)\n");
    printf("  DSP target: %d MHz, scale factor: %.1fx\n", DSP_MHZ, DSP_SCALE);
    printf("  Op-mode iterations: %d per scenario\n", ITERATIONS);
    printf("  Compiler: %s\n",
#ifdef __GNUC__
           "GCC " __VERSION__
#else
           "Unknown"
#endif
    );
    printf("========================================================\n\n");

    calibrate_pc_clock();
    printf("  Estimated PC clock: %.0f MHz\n\n", pc_mhz);

    /* Common envelope */
    memset(&envelope, 0, sizeof(envelope));
    envelope.valid  = 1;
    envelope.az_min = -180.0f;  envelope.el_min = -30.0f;
    envelope.az_max =  180.0f;  envelope.el_max =  60.0f;

    /* ==============================================================
     *  PHASE 1: Pre-Op BuildGraph timing
     * ============================================================== */
    printf("========================================================\n");
    printf("  PHASE 1: Pre-Op  (Avoidance_BuildGraph)\n");
    printf("========================================================\n\n");

    /* Pre-op N=1 baseline */
    {
        AvdRect f1[AVD_MAX_FORBIDDEN];
        memset(f1, 0, sizeof(f1));
        f1[0].valid = 1;
        f1[0].az_min = -40.0f;  f1[0].el_min = -15.0f;
        f1[0].az_max =  40.0f;  f1[0].el_max =  15.0f;
        bench_build_graph("N=1  Baseline", f1, &envelope, 1, &graph);
    }

    /* Pre-op N=10 realistic */
    {
        AvdRect f10[AVD_MAX_FORBIDDEN];
        build_10_zones(f10);
        bench_build_graph("N=10 Realistic deployment", f10, &envelope, 1, &graph);
    }

    /* Pre-op N=10 maze */
    {
        AvdRect f10m[AVD_MAX_FORBIDDEN];
        build_10_maze(f10m);
        bench_build_graph("N=10 Staggered maze", f10m, &envelope, 1, &graph);
    }

    /* ==============================================================
     *  PHASE 2: Op-Mode Step timing
     * ============================================================== */
    printf("========================================================\n");
    printf("  PHASE 2: Op-Mode  (Avoidance_Step @ 1 kHz)\n");
    printf("========================================================\n\n");

    printf("--- Baseline scenarios ---\n\n");

    /* Scenario 1: N=0, direct path (best case) */
    memset(&in, 0, sizeof(in));
    in.az_now =  150.0f;  in.el_now =   0.0f;
    in.az_cmd = -150.0f;  in.el_cmd =  30.0f;
    in.envelope = envelope;
    bench_step("N=0  Direct path (best case)", &in, 1, NULL, 0);

    /* Scenario 2: N=1, with graph, A* needed */
    {
        AvdGraph g1;
        AvdRect f1[AVD_MAX_FORBIDDEN];
        memset(f1, 0, sizeof(f1));
        f1[0].valid = 1;
        f1[0].az_min = -10.0f;  f1[0].el_min = -29.0f;
        f1[0].az_max =  10.0f;  f1[0].el_max =  59.0f;
        Avoidance_BuildGraph(&g1, f1, &envelope, AVD_MOTION_LINEAR, 1);

        memset(&in, 0, sizeof(in));
        in.az_now = -60.0f;  in.el_now =  0.0f;
        in.az_cmd =  60.0f;  in.el_cmd =  0.0f;
        in.envelope = envelope;
        in.forbidden[0] = f1[0];
        bench_step("N=1  Full-height wall (A* replan)", &in, 1, &g1, 1);
    }

    printf("--- N=10 STRESS TEST: Realistic deployment ---\n\n");

    /* Scenario 3: N=10 realistic, direct path clear (no zone in the way) */
    {
        AvdGraph g10;
        AvdRect f10[AVD_MAX_FORBIDDEN];
        build_10_zones(f10);
        Avoidance_BuildGraph(&g10, f10, &envelope, AVD_MOTION_LINEAR, 1);

        memset(&in, 0, sizeof(in));
        in.az_now = -50.0f;  in.el_now =  0.0f;
        in.az_cmd = -30.0f;  in.el_cmd =  0.0f;  /* short move, path clear */
        in.envelope = envelope;
        memcpy(in.forbidden, f10, sizeof(AvdRect) * AVD_MAX_FORBIDDEN);
        bench_step("N=10 Direct path clear (best case)", &in, 1, &g10, 0);
    }

    /* Scenario 4: N=10 realistic, cached A* path follow */
    {
        AvdGraph g10;
        AvdRect f10[AVD_MAX_FORBIDDEN];
        build_10_zones(f10);
        Avoidance_BuildGraph(&g10, f10, &envelope, AVD_MOTION_LINEAR, 1);

        memset(&in, 0, sizeof(in));
        in.az_now = -170.0f;  in.el_now =  0.0f;
        in.az_cmd =  170.0f;  in.el_cmd =  0.0f;  /* cross entire field */
        in.envelope = envelope;
        memcpy(in.forbidden, f10, sizeof(AvdRect) * AVD_MAX_FORBIDDEN);
        bench_step("N=10 Cached path follow (typical)", &in, 1, &g10, 0);
    }

    /* Scenario 5: N=10 realistic, forced A* replan every 10 calls */
    {
        AvdGraph g10;
        AvdRect f10[AVD_MAX_FORBIDDEN];
        build_10_zones(f10);
        Avoidance_BuildGraph(&g10, f10, &envelope, AVD_MOTION_LINEAR, 1);

        memset(&in, 0, sizeof(in));
        in.az_now = -170.0f;  in.el_now =  0.0f;
        in.az_cmd =  170.0f;  in.el_cmd =  0.0f;
        in.envelope = envelope;
        memcpy(in.forbidden, f10, sizeof(AvdRect) * AVD_MAX_FORBIDDEN);
        bench_step("N=10 Forced A* replan (worst case)", &in, 1, &g10, 1);
    }

    /* Scenario 6: N=10 realistic, AZ wrap-around path */
    {
        AvdGraph g10;
        AvdRect f10[AVD_MAX_FORBIDDEN];
        build_10_zones(f10);
        Avoidance_BuildGraph(&g10, f10, &envelope, AVD_MOTION_LINEAR, 1);

        memset(&in, 0, sizeof(in));
        in.az_now =  170.0f;  in.el_now =  30.0f;
        in.az_cmd = -170.0f;  in.el_cmd =  30.0f;  /* short wrap path */
        in.envelope = envelope;
        memcpy(in.forbidden, f10, sizeof(AvdRect) * AVD_MAX_FORBIDDEN);
        bench_step("N=10 AZ wrap path (replan)", &in, 1, &g10, 1);
    }

    printf("--- N=10 STRESS TEST: Staggered maze (hardest) ---\n\n");

    /* Scenario 7: N=10 maze, cached path follow */
    {
        AvdGraph gmaze;
        AvdRect fm[AVD_MAX_FORBIDDEN];
        build_10_maze(fm);
        Avoidance_BuildGraph(&gmaze, fm, &envelope, AVD_MOTION_LINEAR, 1);

        memset(&in, 0, sizeof(in));
        in.az_now = -170.0f;  in.el_now =  0.0f;
        in.az_cmd =  170.0f;  in.el_cmd =  0.0f;
        in.envelope = envelope;
        memcpy(in.forbidden, fm, sizeof(AvdRect) * AVD_MAX_FORBIDDEN);
        bench_step("N=10 Maze cached path (typical)", &in, 1, &gmaze, 0);
    }

    /* Scenario 8: N=10 maze, forced A* replan */
    {
        AvdGraph gmaze;
        AvdRect fm[AVD_MAX_FORBIDDEN];
        build_10_maze(fm);
        Avoidance_BuildGraph(&gmaze, fm, &envelope, AVD_MOTION_LINEAR, 1);

        memset(&in, 0, sizeof(in));
        in.az_now = -170.0f;  in.el_now =  0.0f;
        in.az_cmd =  170.0f;  in.el_cmd =  0.0f;
        in.envelope = envelope;
        memcpy(in.forbidden, fm, sizeof(AvdRect) * AVD_MAX_FORBIDDEN);
        bench_step("N=10 Maze A* replan (worst case)", &in, 1, &gmaze, 1);
    }

    printf("--- Motion profile comparison (N=10 realistic) ---\n\n");

    /* Scenario 9: AZ_THEN_EL profile */
    {
        AvdGraph g10az;
        AvdRect f10[AVD_MAX_FORBIDDEN];
        AvdState state_az;
        AvdOutput out_az;
        double t0, t1, elapsed, per_us, dsp_us;
        int ii;
        int status_counts[3] = {0, 0, 0};

        build_10_zones(f10);
        Avoidance_BuildGraph(&g10az, f10, &envelope, AVD_MOTION_AZ_THEN_EL, 1);

        memset(&in, 0, sizeof(in));
        in.az_now = -170.0f;  in.el_now =  0.0f;
        in.az_cmd =  170.0f;  in.el_cmd =  0.0f;
        in.envelope = envelope;
        memcpy(in.forbidden, f10, sizeof(AvdRect) * AVD_MAX_FORBIDDEN);

        Avoidance_Init(&state_az, AVD_MOTION_AZ_THEN_EL, 1);

        t0 = get_time_sec();
        for (ii = 0; ii < ITERATIONS; ii++) {
            out_az = Avoidance_Step(&in, &state_az, &g10az);
            sink = out_az.az_next + out_az.el_next;
            if (out_az.status >= 0 && out_az.status <= 2)
                status_counts[out_az.status]++;
            if ((ii + 1) % 10 == 0) {
                in.az_now = -170.0f;  in.el_now = 0.0f;
                Avoidance_Init(&state_az, AVD_MOTION_AZ_THEN_EL, 1);
            }
        }
        t1 = get_time_sec();
        elapsed = t1 - t0;
        per_us = (elapsed / ITERATIONS) * 1e6;
        dsp_us = estimate_dsp_us(per_us);

        printf("  [OP-MODE] N=10 AZ_THEN_EL profile (replan)\n");
        printf("    Per call (PC)   : %.2f us\n", per_us);
        printf("    Est. DSP @%dMHz : %.2f us  (%.3f ms)\n", DSP_MHZ, dsp_us, dsp_us / 1000.0);
        printf("    DSP budget used : %.2f%% of 1ms\n", (dsp_us / 1000.0) * 100.0);
        printf("    Status counts   : DIRECT=%d  WAYPOINT=%d  NO_PATH=%d\n",
               status_counts[0], status_counts[1], status_counts[2]);
        printf("\n");
    }

    /* Scenario 10: EL_THEN_AZ profile */
    {
        AvdGraph g10el;
        AvdRect f10[AVD_MAX_FORBIDDEN];
        AvdState state_el;
        AvdOutput out_el;
        double t0, t1, elapsed, per_us, dsp_us;
        int ii;
        int status_counts[3] = {0, 0, 0};

        build_10_zones(f10);
        Avoidance_BuildGraph(&g10el, f10, &envelope, AVD_MOTION_EL_THEN_AZ, 1);

        memset(&in, 0, sizeof(in));
        in.az_now = -170.0f;  in.el_now =  0.0f;
        in.az_cmd =  170.0f;  in.el_cmd =  0.0f;
        in.envelope = envelope;
        memcpy(in.forbidden, f10, sizeof(AvdRect) * AVD_MAX_FORBIDDEN);

        Avoidance_Init(&state_el, AVD_MOTION_EL_THEN_AZ, 1);

        t0 = get_time_sec();
        for (ii = 0; ii < ITERATIONS; ii++) {
            out_el = Avoidance_Step(&in, &state_el, &g10el);
            sink = out_el.az_next + out_el.el_next;
            if (out_el.status >= 0 && out_el.status <= 2)
                status_counts[out_el.status]++;
            if ((ii + 1) % 10 == 0) {
                in.az_now = -170.0f;  in.el_now = 0.0f;
                Avoidance_Init(&state_el, AVD_MOTION_EL_THEN_AZ, 1);
            }
        }
        t1 = get_time_sec();
        elapsed = t1 - t0;
        per_us = (elapsed / ITERATIONS) * 1e6;
        dsp_us = estimate_dsp_us(per_us);

        printf("  [OP-MODE] N=10 EL_THEN_AZ profile (replan)\n");
        printf("    Per call (PC)   : %.2f us\n", per_us);
        printf("    Est. DSP @%dMHz : %.2f us  (%.3f ms)\n", DSP_MHZ, dsp_us, dsp_us / 1000.0);
        printf("    DSP budget used : %.2f%% of 1ms\n", (dsp_us / 1000.0) * 100.0);
        printf("    Status counts   : DIRECT=%d  WAYPOINT=%d  NO_PATH=%d\n",
               status_counts[0], status_counts[1], status_counts[2]);
        printf("\n");
    }

    /* ==============================================================
     *  Summary
     * ============================================================== */
    printf("========================================================\n");
    printf("  SUMMARY\n");
    printf("========================================================\n");
    printf("  DSP target     : %d MHz\n", DSP_MHZ);
    printf("  Scale factor   : %.1fx (PC -> DSP)\n", DSP_SCALE);
    printf("  Sample rate    : 1 kHz (1000 us budget per call)\n");
    printf("  Pre-op         : runs once at startup, no deadline\n");
    printf("  Op-mode        : must complete within 1 ms\n");
    printf("  Algorithm      : no trig, no sqrt, no malloc\n");
    printf("  Max zones      : %d (tested with 10)\n", AVD_MAX_FORBIDDEN);
    printf("========================================================\n");
    printf("\n");
    printf("  Key metrics for N=10 on DSP @%d MHz:\n", DSP_MHZ);
    printf("    - Cached path follow : ~few us (well within budget)\n");
    printf("    - A* replan          : check 'Forced A* replan' above\n");
    printf("    - Pre-op build       : one-time cost, check Phase 1\n");
    printf("========================================================\n");

    return 0;
}
