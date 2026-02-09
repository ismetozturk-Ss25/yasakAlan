/*
 * avoidance_helpers.h — Shared static inline helper functions
 *
 * Used by both avoidance_preop.c and avoidance_opmode.c.
 * All functions are declared static inline so each .c file gets its own
 * copy — no linker issues, fully Simulink-compatible.
 *
 * Contents (12 functions):
 *   Math:      avd_abs, avd_min, avd_max, avd_clamp
 *   AZ wrap:   avd_normalize_az, avd_manhattan
 *   Geometry:  point_in_rect, point_in_any_forbidden, point_in_envelope
 *   Segment:   segment_hits_rect, segment_is_clear, path_is_clear
 */

#ifndef AVOIDANCE_HELPERS_H
#define AVOIDANCE_HELPERS_H

#include "avoidance_types.h"

/* ==============================================================
 *  INLINE MATH HELPERS  (no <math.h> dependency)
 * ============================================================== */

static inline avd_real avd_abs(avd_real v)
{
    return v < 0.0f ? -v : v;
}

static inline avd_real avd_min(avd_real a, avd_real b)
{
    return a < b ? a : b;
}

static inline avd_real avd_max(avd_real a, avd_real b)
{
    return a > b ? a : b;
}

static inline avd_real avd_clamp(avd_real v, avd_real lo, avd_real hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ==============================================================
 *  AZ WRAP-AROUND HELPERS
 * ============================================================== */

static inline avd_real avd_normalize_az(avd_real az)
{
    if (az > 180.0f)  return az - 360.0f;
    if (az <= -180.0f) return az + 360.0f;
    return az;
}

static inline avd_real avd_manhattan(avd_real ax, avd_real ay,
                                     avd_real bx, avd_real by,
                                     int az_wrap)
{
    avd_real daz = avd_abs(ax - bx);
    if (az_wrap && daz > 180.0f) daz = 360.0f - daz;
    return daz + avd_abs(ay - by);
}

/* ==============================================================
 *  GEOMETRIC PREDICATES
 * ============================================================== */

static inline int point_in_rect(avd_real az, avd_real el, const AvdRect *r)
{
    return (az > r->az_min && az < r->az_max &&
            el > r->el_min && el < r->el_max);
}

static inline int point_in_any_forbidden(avd_real az, avd_real el,
                                         const AvdRect *f, int n)
{
    int i;
    for (i = 0; i < n; i++) {
        if (point_in_rect(az, el, &f[i])) return 1;
    }
    return 0;
}

static inline int point_in_envelope(avd_real az, avd_real el, const AvdRect *env)
{
    return (az >= env->az_min && az <= env->az_max &&
            el >= env->el_min && el <= env->el_max);
}

/* ==============================================================
 *  LIANG-BARSKY  LINE-SEGMENT  vs  AABB  INTERSECTION
 * ============================================================== */

static inline int segment_hits_rect(avd_real p0x, avd_real p0y,
                                    avd_real p1x, avd_real p1y,
                                    const AvdRect *r)
{
    avd_real dx = p1x - p0x;
    avd_real dy = p1y - p0y;
    avd_real t_near = 0.0f;
    avd_real t_far  = 1.0f;
    avd_real t1, t2, inv, tmp;

    if (dx > -1e-9f && dx < 1e-9f) {
        if (p0x <= r->az_min || p0x >= r->az_max) return 0;
    } else {
        inv = 1.0f / dx;
        t1  = (r->az_min - p0x) * inv;
        t2  = (r->az_max - p0x) * inv;
        if (t1 > t2) { tmp = t1; t1 = t2; t2 = tmp; }
        t_near = avd_max(t_near, t1);
        t_far  = avd_min(t_far,  t2);
        if (t_near >= t_far) return 0;
    }

    if (dy > -1e-9f && dy < 1e-9f) {
        if (p0y <= r->el_min || p0y >= r->el_max) return 0;
    } else {
        inv = 1.0f / dy;
        t1  = (r->el_min - p0y) * inv;
        t2  = (r->el_max - p0y) * inv;
        if (t1 > t2) { tmp = t1; t1 = t2; t2 = tmp; }
        t_near = avd_max(t_near, t1);
        t_far  = avd_min(t_far,  t2);
        if (t_near >= t_far) return 0;
    }

    return 1;
}

/* ==============================================================
 *  SEGMENT-CLEAR CHECK  (wrap-aware)
 * ============================================================== */

static inline int segment_is_clear(avd_real p0x, avd_real p0y,
                                   avd_real p1x, avd_real p1y,
                                   const AvdRect *f, int n,
                                   int az_wrap)
{
    int i;
    avd_real daz = p1x - p0x;

    if (az_wrap && (daz > 180.0f || daz < -180.0f)) {
        avd_real eff_daz, bnd, t_split, el_split;

        if (daz < -180.0f) {
            eff_daz = daz + 360.0f;
            bnd     = 180.0f;
        } else {
            eff_daz = daz - 360.0f;
            bnd     = -180.0f;
        }

        if (avd_abs(eff_daz) < 1e-9f) return 1;

        t_split  = (bnd - p0x) / eff_daz;
        el_split = p0y + t_split * (p1y - p0y);

        for (i = 0; i < n; i++)
            if (segment_hits_rect(p0x, p0y, bnd, el_split, &f[i]))
                return 0;

        for (i = 0; i < n; i++)
            if (segment_hits_rect(-bnd, el_split, p1x, p1y, &f[i]))
                return 0;

        return 1;
    }

    for (i = 0; i < n; i++) {
        if (segment_hits_rect(p0x, p0y, p1x, p1y, &f[i]))
            return 0;
    }
    return 1;
}

/* ==============================================================
 *  PATH-CLEAR  (respects motion profile + wrap)
 * ============================================================== */

static inline int path_is_clear(avd_real p0az, avd_real p0el,
                                avd_real p1az, avd_real p1el,
                                const AvdRect *f, int n,
                                AvdMotionProfile profile,
                                int az_wrap)
{
    switch (profile) {
    case AVD_MOTION_AZ_THEN_EL:
        if (!segment_is_clear(p0az, p0el, p1az, p0el, f, n, az_wrap)) return 0;
        return segment_is_clear(p1az, p0el, p1az, p1el, f, n, az_wrap);

    case AVD_MOTION_EL_THEN_AZ:
        if (!segment_is_clear(p0az, p0el, p0az, p1el, f, n, az_wrap)) return 0;
        return segment_is_clear(p0az, p1el, p1az, p1el, f, n, az_wrap);

    default: /* AVD_MOTION_LINEAR */
        if (!segment_is_clear(p0az, p0el, p1az, p1el, f, n, az_wrap)) return 0;
        if (!segment_is_clear(p0az, p1el, p1az, p1el, f, n, az_wrap)) return 0;
        if (!segment_is_clear(p1az, p0el, p1az, p1el, f, n, az_wrap)) return 0;
        return 1;
    }
}

#endif /* AVOIDANCE_HELPERS_H */
