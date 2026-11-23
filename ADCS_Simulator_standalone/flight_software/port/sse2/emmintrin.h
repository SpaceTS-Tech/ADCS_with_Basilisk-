/*
 * emmintrin.h -- portable plain-C stand-in for the x86 SSE2 header
 *
 * Simulink Coder generated ADCS_Software_v1.c for an x86-64 host, so it uses
 * eight SSE2 double-precision intrinsics (_mm_add_pd ... _mm_storeu_pd).  A
 * non-x86 compiler (ARM, RISC-V, PowerPC, LEON/SPARC) has no <emmintrin.h>.
 *
 * Add this directory to the include path ONLY for non-x86 targets
 * (-Iport/sse2); it then shadows the compiler header and maps each intrinsic
 * to two scalar double operations with identical IEEE-754 results.
 *
 * Remove once the model is regenerated with the target's Hardware
 * Implementation settings (Device vendor/type = the flight processor), which
 * stops Simulink Coder emitting SSE2 code.
 */
#ifndef ADCS_PORTABLE_EMMINTRIN_H
#define ADCS_PORTABLE_EMMINTRIN_H

/* Element 0 is the low lane, as on x86. */
typedef struct {
    double v[2];
} __m128d;

static inline __m128d _mm_set_pd(double e1, double e0)
{
    __m128d r;
    r.v[0] = e0;
    r.v[1] = e1;
    return r;
}

static inline __m128d _mm_set1_pd(double a)
{
    __m128d r;
    r.v[0] = a;
    r.v[1] = a;
    return r;
}

static inline __m128d _mm_loadu_pd(const double *p)
{
    __m128d r;
    r.v[0] = p[0];
    r.v[1] = p[1];
    return r;
}

static inline void _mm_storeu_pd(double *p, __m128d a)
{
    p[0] = a.v[0];
    p[1] = a.v[1];
}

static inline __m128d _mm_add_pd(__m128d a, __m128d b)
{
    __m128d r;
    r.v[0] = a.v[0] + b.v[0];
    r.v[1] = a.v[1] + b.v[1];
    return r;
}

static inline __m128d _mm_sub_pd(__m128d a, __m128d b)
{
    __m128d r;
    r.v[0] = a.v[0] - b.v[0];
    r.v[1] = a.v[1] - b.v[1];
    return r;
}

static inline __m128d _mm_mul_pd(__m128d a, __m128d b)
{
    __m128d r;
    r.v[0] = a.v[0] * b.v[0];
    r.v[1] = a.v[1] * b.v[1];
    return r;
}

static inline __m128d _mm_div_pd(__m128d a, __m128d b)
{
    __m128d r;
    r.v[0] = a.v[0] / b.v[0];
    r.v[1] = a.v[1] / b.v[1];
    return r;
}

#endif /* ADCS_PORTABLE_EMMINTRIN_H */
