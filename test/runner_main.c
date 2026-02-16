/*
 * runner_main.c -- Test harness for the Avoidance Command Shaper
 *
 * All tests use a unified envelope and az_wrap=1:
 *   Envelope:  AZ[-180, +180]   EL[-30, +60]
 *   az_wrap = 1  (AZ wraps at +/-180 boundary)
 *
 * Test scenarios:
 *   T1  No forbidden zones          -> OK_DIRECT, wrap path
 *   T2  Single obstacle             -> waypoint routing
 *   T3  N=16 stress                 -> many small zones
 *   T4  Dynamic target              -> target changes mid-run
 *   T5  Target outside envelope     -> clipping
 *   T6  NO_PATH                     -> full-height wall
 *   T7  Three obstacles, diagonal   -> multi-waypoint
 *   T8  Wrap with obstacle at +/-180 -> forced wrap routing
 *
 * Build:
 *   gcc -O2 -Wall -Wextra -pedantic -std=c99 -Isrc \
 *       src/avoidance.c test/runner_main.c -o test_runner.exe
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "avoidance_simulink.h"

/* ================================================================
 *  OUTPUT FILE  (dual output to console and file)
 * ================================================================ */
#define OUTPUT_FILENAME "forbidden_zones_c.txt"

static FILE *g_output_file = NULL;

/* Print to both console and file */
static void dual_printf(const char *fmt, ...)
{
    va_list args;

    /* Print to console */
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);

    /* Print to file if open */
    if (g_output_file) {
        va_start(args, fmt);
        vfprintf(g_output_file, fmt, args);
        va_end(args);
    }
}

/* ================================================================
 *  COMMON ENVELOPE  (edit these to match your hardware)
 * ================================================================ */
#define ENV_AZ_MIN  -180.0f
#define ENV_AZ_MAX   180.0f
#define ENV_EL_MIN    -30.0f
#define ENV_EL_MAX    60.0f
#define AZ_WRAP       1

static void set_envelope(AvdInput *in)
{
    in->envelope.valid  = 1;
    in->envelope.az_min = ENV_AZ_MIN;
    in->envelope.el_min = ENV_EL_MIN;
    in->envelope.az_max = ENV_AZ_MAX;
    in->envelope.el_max = ENV_EL_MAX;
}

/* ================================================================
 *  HELPERS
 * ================================================================ */

static float fabsf_local(float v) { return v < 0.0f ? -v : v; }

static float move_toward(float current, float target, float rate)
{
    float diff = target - current;
    if (diff >  rate) return current + rate;
    if (diff < -rate) return current - rate;
    return target;
}

/* Wrap-aware AZ movement: takes shortest path through +/-180 boundary */
static float move_toward_wrap(float current, float target, float rate)
{
    float diff = target - current;
    float next;
    if (diff > 180.0f)  diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;

    if (diff >  rate)      next = current + rate;
    else if (diff < -rate) next = current - rate;
    else                   next = target;

    if (next > 180.0f)   next -= 360.0f;
    if (next <= -180.0f) next += 360.0f;
    return next;
}

static const char *status_str(AvdStatus s)
{
    switch (s) {
    case AVD_OK_DIRECT:   return "OK_DIRECT  ";
    case AVD_OK_WAYPOINT: return "OK_WAYPOINT";
    case AVD_NO_PATH:     return "NO_PATH    ";
    default:              return "???        ";
    }
}

/* Check safety invariants; returns number of violations */
static int check_safety(const AvdOutput *out, const AvdInput *in)
{
    int violations = 0;
    int i;

    /* envelope check (wrap-aware for AZ) */
    if (in->envelope.az_min > in->envelope.az_max) {
        /* Wrap-around envelope: valid AZ is [az_min,180] U [-180,az_max]
         * Invalid AZ is strictly inside the gap (az_max, az_min) */
        if (out->az_next > in->envelope.az_max + 0.2f &&
            out->az_next < in->envelope.az_min - 0.2f) {
            dual_printf("  !! VIOLATION: output (%.2f,%.2f) in AZ gap [%.1f,%.1f]\n",
                   out->az_next, out->el_next,
                   in->envelope.az_max, in->envelope.az_min);
            violations++;
        }
        if (out->el_next < in->envelope.el_min - 0.2f ||
            out->el_next > in->envelope.el_max + 0.2f) {
            dual_printf("  !! VIOLATION: output (%.2f,%.2f) outside EL envelope\n",
                   out->az_next, out->el_next);
            violations++;
        }
    } else if (out->az_next < in->envelope.az_min - 0.2f ||
        out->az_next > in->envelope.az_max + 0.2f ||
        out->el_next < in->envelope.el_min - 0.2f ||
        out->el_next > in->envelope.el_max + 0.2f) {
        dual_printf("  !! VIOLATION: output (%.2f,%.2f) outside envelope\n",
               out->az_next, out->el_next);
        violations++;
    }

    /* forbidden zone check (strict interior) */
    for (i = 0; i < AVD_MAX_FORBIDDEN; i++) {
        const AvdRect *r = &in->forbidden[i];
        if (!r->valid) continue;
        if (out->az_next > r->az_min && out->az_next < r->az_max &&
            out->el_next > r->el_min && out->el_next < r->el_max) {
            dual_printf("  !! VIOLATION: output (%.2f,%.2f) inside forbidden[%d]\n",
                   out->az_next, out->el_next, i);
            violations++;
        }
    }

    return violations;
}

/* Run a simulation and return total violations.
 * Always prints pos and cmd every step (verbose). */
static int run_sim(const char *name,
                   AvdInput *in,
                   float move_rate,
                   int max_steps)
{
    AvdState state;
    AvdGraph graph;
    AvdOutput out;
    int step, total_violations = 0;
    int reached = 0;

    /* PRE-OP: build visibility graph (time unconstrained) */
    Avoidance_BuildGraph(&graph, in->forbidden,
                         &in->envelope, AVD_MOTION_LINEAR, AZ_WRAP);

    Avoidance_Init(&state, AVD_MOTION_LINEAR, AZ_WRAP);

    dual_printf("\n======================================\n");
    dual_printf("  %s\n", name);
    dual_printf("======================================\n");
    dual_printf("  Envelope: AZ[%.1f,%.1f]  EL[%.1f,%.1f]  (az_wrap=%d)\n",
           in->envelope.az_min, in->envelope.az_max,
           in->envelope.el_min, in->envelope.el_max, AZ_WRAP);
    {
        int i, cnt = 0;
        for (i = 0; i < AVD_MAX_FORBIDDEN; i++)
            if (in->forbidden[i].valid) cnt++;
        dual_printf("  Forbidden zones: %d\n", cnt);
        for (i = 0; i < AVD_MAX_FORBIDDEN; i++) {
            if (!in->forbidden[i].valid) continue;
            dual_printf("    [%2d] AZ[%7.1f, %7.1f]  EL[%6.1f, %6.1f]\n",
                   i,
                   in->forbidden[i].az_min, in->forbidden[i].az_max,
                   in->forbidden[i].el_min, in->forbidden[i].el_max);
        }
    }
    dual_printf("  Precomputed graph: %d nodes, %d bytes adjacency\n",
           graph.nc, graph.nc * AVD_ADJ_BYTES);
    dual_printf("  Start: (%.2f, %.2f)  Target: (%.2f, %.2f)\n",
           in->az_now, in->el_now, in->az_cmd, in->el_cmd);
    dual_printf("--------------------------------------\n");

    for (step = 0; step < max_steps; step++) {
        out = Avoidance_Step(in, &state, &graph);

        dual_printf("  [%3d] pos=(%7.2f,%7.2f) cmd=(%7.2f,%7.2f)"
               " -> next=(%7.2f,%7.2f) %s\n",
               step,
               in->az_now, in->el_now,
               in->az_cmd, in->el_cmd,
               out.az_next, out.el_next,
               status_str(out.status));

        total_violations += check_safety(&out, in);

        /* simulate movement toward output */
        in->az_now = move_toward_wrap(in->az_now, out.az_next, move_rate);
        in->el_now = move_toward(in->el_now, out.el_next, move_rate);

        /* check if reached target (wrap-aware for AZ) */
        {
            float daz = fabsf_local(in->az_now - in->az_cmd);
            if (daz > 180.0f) daz = 360.0f - daz;
            if (daz < 0.5f && fabsf_local(in->el_now - in->el_cmd) < 0.5f) {
                if (!reached) {
                    dual_printf("  >> Target reached at step %d\n", step);
                    reached = 1;
                    break;
                }
            }
        }

        /* for NO_PATH test, break if holding */
        if (out.status == AVD_NO_PATH) {
            dual_printf("  >> NO_PATH -- holding at (%.2f, %.2f)\n",
                   out.az_next, out.el_next);
            break;
        }
    }

    dual_printf("--------------------------------------\n");
    if (total_violations == 0)
        dual_printf("  RESULT: PASS  (0 violations, %d steps)\n", step);
    else
        dual_printf("  RESULT: FAIL  (%d violations)\n", total_violations);

    return total_violations;
}

/* ================================================================
 *  TEST SCENARIOS
 *  All use envelope AZ[-180,+180] EL[-30,+60] with az_wrap=1
 * ================================================================ */

/* T1: No forbidden zones -- wrap path from 150 to -150 (short 60-deg wrap) */
static int test_T1(void)
{
    AvdInput in;
    memset(&in, 0, sizeof(in));
    set_envelope(&in);
    in.az_now =  2.0f;   in.el_now = 2.0f;
    in.az_cmd = 100.0f;   in.el_cmd = 0.0f;

    /* Zone 0: AZ[0,10] EL[0,10] */
    in.forbidden[0].valid = 1;
    in.forbidden[0].az_min =  0.0f;  in.forbidden[0].el_min =  0.0f;
    in.forbidden[0].az_max =  10.0f;  in.forbidden[0].el_max = 10.0f;

    /* Zone 1: AZ[30,40] EL[0,34] */
    in.forbidden[1].valid = 0;
    in.forbidden[1].az_min = -180.0f;  in.forbidden[1].el_min = -30.0f;
    in.forbidden[1].az_max = -170.0f;  in.forbidden[1].el_max = 36.0f;

    /* Zone 2: AZ[60,90] EL[0,15] */
    in.forbidden[2].valid = 0;
    in.forbidden[2].az_min = 115.0f;  in.forbidden[2].el_min = -15.0f;
    in.forbidden[2].az_max = 150.0f;  in.forbidden[2].el_max = 25.0f;

    /* Zone 3: AZ[-20,-10] EL[0,18] */
    in.forbidden[3].valid = 0;
    in.forbidden[3].az_min = -20.0f;  in.forbidden[3].el_min = 0.0f;
    in.forbidden[3].az_max = -10.0f;  in.forbidden[3].el_max = 18.0f;

    /* Zone 4: AZ[-60,-40] EL[0,20] */
    in.forbidden[4].valid = 0;
    in.forbidden[4].az_min = -60.0f;  in.forbidden[4].el_min = 0.0f;
    in.forbidden[4].az_max = -40.0f;  in.forbidden[4].el_max = 20.0f;

    /* Zone 5: AZ[150,160] EL[0,35] */
    in.forbidden[5].valid = 0;
    in.forbidden[5].az_min = 150.0f;  in.forbidden[5].el_min = 0.0f;
    in.forbidden[5].az_max = 160.0f;  in.forbidden[5].el_max = 35.0f;





    return run_sim("T1: (110,0)->(21,0) wrap env[35,30], 2 split zones",
                   &in, 2.0f, 200*100);
}



/*c ================================================================
 *  MAIN
 * ================================================================ */

int main(void)
{
    int total_fails = 0;

    /* Delete and create output file */
    remove(OUTPUT_FILENAME);
    g_output_file = fopen(OUTPUT_FILENAME, "w");
    if (!g_output_file) {
        printf("Warning: Could not open %s for writing\n", OUTPUT_FILENAME);
    }

    dual_printf("================================================\n");
    dual_printf("  Forbidden-Area Avoidance -- Acceptance Tests\n");
    dual_printf("  Envelope: AZ[%.0f,%.0f] EL[%.0f,%.0f]  az_wrap=%d\n",
           ENV_AZ_MIN, ENV_AZ_MAX, ENV_EL_MIN, ENV_EL_MAX, AZ_WRAP);
    dual_printf("================================================\n");

    total_fails += test_T1();

    dual_printf("\n================================================\n");
    if (total_fails == 0)
        dual_printf("  ALL TESTS PASSED\n");
    else
        dual_printf("  %d TOTAL VIOLATIONS -- SOME TESTS FAILED\n", total_fails);
    dual_printf("================================================\n");

    /* Close output file */
    if (g_output_file) {
        fclose(g_output_file);
        dual_printf("\nOutput also written to: %s\n", OUTPUT_FILENAME);
    }

    return total_fails > 0 ? 1 : 0;
}
