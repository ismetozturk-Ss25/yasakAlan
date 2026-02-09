/*
 * bench.c — Worst-case timing benchmark for Avoidance_Step
 *
 * Measures wall-clock time of a single call with N=16 forbidden rects
 * (worst-case scenario where all candidates must be evaluated).
 */

#include <stdio.h>
#include <string.h>
#include <time.h>
#include "avoidance.h"

int main(void)
{
    AvdState state;
    AvdInput in;
    AvdOutput out;
    int i;
    clock_t t0, t1;
    double us_per_call;
    int LOOPS = 1000000;

    Avoidance_Init(&state, AVD_MOTION_LINEAR, 0);
    memset(&in, 0, sizeof(in));

    /* Worst case: N=16 rects, direct path BLOCKED, full waypoint search */
    in.az_now = 5.0f;    in.el_now = 5.0f;
    in.az_cmd = 350.0f;  in.el_cmd = 80.0f;
    in.envelope.az_min = 0.0f;   in.envelope.az_max = 360.0f;
    in.envelope.el_min = 0.0f;   in.envelope.el_max = 90.0f;

    in.forbidden_count = 16;
    /* Dense grid of rects that forces path blocking and many candidate checks */
    for (i = 0; i < 16; i++) {
        float base_az = 10.0f + (float)(i % 4) * 70.0f;
        float base_el = 2.0f  + (float)(i / 4) * 20.0f;
        in.forbidden[i].az_min = base_az;
        in.forbidden[i].az_max = base_az + 40.0f;
        in.forbidden[i].el_min = base_el;
        in.forbidden[i].el_max = base_el + 15.0f;
    }

    /* Warm up */
    for (i = 0; i < 1000; i++) {
        state.active = 0;
        out = Avoidance_Step(&in, &state);
    }

    /* Benchmark */
    t0 = clock();
    for (i = 0; i < LOOPS; i++) {
        state.active = 0;  /* force full re-plan every call */
        out = Avoidance_Step(&in, &state);
    }
    t1 = clock();

    us_per_call = ((double)(t1 - t0) / CLOCKS_PER_SEC) * 1e6 / LOOPS;

    printf("========================================\n");
    printf("  Avoidance_Step  Benchmark\n");
    printf("========================================\n");
    printf("  Forbidden rects : %d\n", in.forbidden_count);
    printf("  Loops           : %d\n", LOOPS);
    printf("  Status          : %d\n", out.status);
    printf("  Output          : (%.2f, %.2f)\n", out.az_next, out.el_next);
    printf("----------------------------------------\n");
    printf("  Time per call   : %.2f us\n", us_per_call);
    printf("  Calls per ms    : %.0f\n", 1000.0 / us_per_call);
    printf("========================================\n");

    return 0;
}
