#ifndef STIR_MATH_NEON_IMPL_HPP_
#define STIR_MATH_NEON_IMPL_HPP_

#include <arm_neon.h>
#include <cstring>
#include <cassert>

namespace stir {

namespace neon::impl {

const float EPS = 1e-20f;

inline float32x4_t assign_4f_2f(float x, float y) {
    float32x2_t lo { x, y };
    return vcombine_f32(lo, vdup_n_f32(0));
}

// template<unsigned N>
// struct swizz<N,N,N,N> { static float32x4_t get(float32x4_t a) { return vdupq_laneq_f32(a, N); } };

inline float32x4_t assign_4f_1f(float x) { return (float32x4_t){x,x,x,x}; }
inline float32x4_t assign_4f_1f(float x, float w) { return (float32x4_t){x, x, x, w}; }
inline float32x4_t assign_4f_3f(float x, float y, float z) {
#if 0
    float32x2_t lo = vcreate_f32( ((uint64_t)*(uint32_t*)&y << 32) | (uint64_t)*(uint32_t*)&x);
    float32x2_t hi = vcreate_f32( ((uint64_t)*(uint32_t*)&z) );
    return vcombine_f32(lo, hi);
#elif 1
    float32x2_t lo { x, y };
    float32x2_t hi { z, 0 };
    return vcombine_f32(lo, hi);
#elif 0
    float32x2_t lo = { x, y };
    float32x4_t v  = vcombine_f32(lo, vdup_n_f32(0.0f));
    return vsetq_lane_f32(z, v, 2);
#endif
}
inline float32x4_t assign_4f_4f(float x, float y, float z, float w) { return (float32x4_t){x, y, z, w}; }
inline float32x4_t assign_4f_3f_1f(float32x4_t v, float w) { return vsetq_lane_f32(w, v, 3); }

inline float32x4_t load_3f(float const* p) {
    float32x2_t a = vld1_f32(p);
    float32x4_t a1 = vcombine_f32(a, vdup_n_f32(0));
    float32x4_t b = vld1q_lane_f32(p + 2, a1, 2);
    return b;
}

inline void store_3f(float* p, float32x4_t v) {
    vst1_f32(p, vget_low_f32(v));
    vst1q_lane_f32(p, v, 2);
}

inline float32x4_t load_4f(float const* p) {
    return vld1q_f32(p);
}

inline void store_4f(float* p, float32x4_t v) {
    vst1q_f32(p, v);
}

inline float elem_4f(float32x4_t a, size_t i) { return a[i]; }
inline float elem_12f(float32x4x3_t a, size_t i, size_t j) { return a.val[j][i]; }
inline float elem_16f(float32x4x4_t a, size_t i, size_t j) { return a.val[j][i]; }

inline float32x4_t to_vec3(float32x4_t a) { return vsetq_lane_f32(0, a, 3); }

inline bool eq_2f(float32x4_t a, float32x4_t b) {
    uint32x4_t cmp = vceqq_f32(a, b);
    uint32x2_t cmpL = vget_low_f32(cmp);
    return vminv_u32(cmpL) == 0xFFFFFFFFu;
}

inline bool eq_3f(float32x4_t a, float32x4_t b) {
    uint32x4_t cmp = vceqq_f32(a, b);
    cmp = vsetq_lane_u32(0xFFFFFFFFu, cmp, 3);
    return vminvq_u32(cmp) == 0xFFFFFFFFu;
}

inline bool eq_4f(float32x4_t a, float32x4_t b) {
    uint32x4_t cmp = vceqq_f32(a, b);
    return vminvq_u32(cmp) == 0xFFFFFFFFu;
}

inline bool eq_16f(float32x4x4_t a, float32x4x4_t b) {
    uint32x4_t m0 = vceqq_f32(a.val[0], b.val[0]);
    uint32x4_t m1 = vceqq_f32(a.val[1], b.val[1]);
    uint32x4_t m2 = vceqq_f32(a.val[2], b.val[2]);
    uint32x4_t m3 = vceqq_f32(a.val[3], b.val[3]);
    uint32x4_t all = vandq_u32(vandq_u32(m0, m1), vandq_u32(m2, m3));
    return vminvq_u32(all) == 0xFFFFFFFFu;
}

inline float32x4_t addf(float32x4_t a, float32x4_t b) {
    return vaddq_f32(a, b);
}

inline float32x4_t subf(float32x4_t a, float32x4_t b) {
    return vsubq_f32(a, b);
}

inline float32x4_t mulf(float32x4_t a, float32x4_t b) {
    return vmulq_f32(a, b);
}

inline float32x4_t mulf(float32x4_t a, float32_t b) {
    return vmulq_f32(a, vdupq_n_f32(b));
}

template<unsigned PRECISE>
inline float32x4_t recp(float32x4_t a) {
    float32x4_t re = vrecpeq_f32(a);
    // Xn+1 = 2 - d*Xn
    re = vmulq_f32(re, vrecpsq_f32(a, re));
    if constexpr (PRECISE >= 2)
        re = vmulq_f32(re, vrecpsq_f32(a, re));
    return re;
}

template<unsigned PRECISE>
inline float32x4_t rsqrt(float32x4_t a) {
    float32x4_t re = vrsqrteq_f32(a);
    // Xn+1 = Xn * (3 - d*Xn^2) / 2  and NEON does Xn+1 = Xn * (3 - a*b) / 2
    re = vmulq_f32(re, vrsqrtsq_f32(a, vmulq_f32(re,re)));
    if constexpr (PRECISE >= 2)
        re = vmulq_f32(re, vrsqrtsq_f32(a, vmulq_f32(re,re)));
    return re;
}

inline float32_t inv_lensq4(float32x4_t a) {
    float32x4_t sq = vdupq_n_f32(vaddvq_f32(vmulq_f32(a,a)));
    float32x4_t re = recp<1>(sq);
    return re[0];
}

inline float32_t inv_len4(float32x4_t a) {
    float32x4_t sq = vdupq_n_f32(vaddvq_f32(vmulq_f32(a,a)));
    float32x4_t rsq = rsqrt<1>(sq);
    return rsq[0];
}

inline float32_t len4(float32x4_t a) {
    float32x4_t sq = vdupq_n_f32(vaddvq_f32(vmulq_f32(a,a)));
    uint32x4_t cmp = vcleq_f32(sq, vdupq_n_f32(EPS));
    float32x4_t invlen = rsqrt<1>(sq);
    float32x4_t len = vmulq_f32(sq, invlen); // a*1/sqrt(a)
    float32x4_t len1 = vbslq_f32(cmp, vdupq_n_f32(0.0f), len); // take 0 if sq <= EPS
    return len1[0];
}

inline float32_t len3(float32x4_t a) {
    float32x4_t v3 = to_vec3(a);
    return len4(v3);
}

inline float32_t dot3(float32x4_t a, float32x4_t b) {
    float32x4_t a0 = vsetq_lane_f32(0.0f, a, 3);
    float32x4_t b0 = vsetq_lane_f32(0.0f, b, 3);
    float32_t x = vaddvq_f32(vmulq_f32(a0, b0));
    return x;
}

inline float32_t dot4(float32x4_t a, float32x4_t b) {
    #if defined(__aarch64__)
    float32_t x = vaddvq_f32(vmulq_f32(a, b));
    return x;
    #else
    float32x4_t sq = vmulq_f32(a, b);
    float32x2_t sum1 = vadd_f32(vget_low_f32(sq), vget_high_f32(sq));
    float32x4_t d = vdupq_lane_f32(vpadd_f32(sum1, sum1), 0);
    return d;
    #endif
}

inline float32x4_t normalized_4f(float32x4_t a) {
    float32x4_t sq = vmulq_f32(a, a);
    #if defined(__aarch64__)
    float32x4_t l2 = vdupq_n_f32(vaddvq_f32(sq));
    #else
    float32x2_t sum2 = vadd_f32(vget_low_f32(sq), vget_high_f32(sq));
    float32x4_t l2 = vdupq_lane_f32(vpadd_f32(sum2, sum2), 0);
    #endif
    float32x4_t rsq = rsqrt<1>(l2);
    return vmulq_f32(a, rsq);
}

inline float32x4_t normalized_precise_4f(float32x4_t a) {
    float32x4_t sq = vmulq_f32(a, a);
    #if defined(__aarch64__)
    float32x4_t l2 = vdupq_n_f32(vaddvq_f32(sq));
    #else
    float32x2_t sum2 = vadd_f32(vget_low_f32(sq), vget_high_f32(sq));
    float32x4_t l2 = vdupq_lane_f32(vpadd_f32(sum2, sum2), 0);
    #endif
    float32x4_t rsq = rsqrt<2>(l2);
    return vmulq_f32(a, rsq);
}

inline float32x4_t normalized_3f(float32x4_t a) {
    float32x4_t v0 = vsetq_lane_f32(0.0f, a, 3);
    return normalized_4f(v0);
}

inline float32x4_t normalized_precise_3f(float32x4_t a) {
    float32x4_t v0 = vsetq_lane_f32(0.0f, a, 3);
    return normalized_precise_4f(v0);
}

inline float32x4_t cross(float32x4_t a, float32x4_t b) {
    // _ (ay*bz, az*bx, ax*by) [v1]
    //   (az*by, ax*bz, ay*bx) [v2]
    // ax ay az * by bz bx -> v1
    // ay az ax * bx by bz -> v2
    // return v1 - v2, after swizzle xyzw -> yzx0

#if 0
    float32x4_t r = swizz<1,2,0>::get3(a*swizz<1,2,0>::get4(b) - swizz<1,2,0>::get4(a)*b, 0);
#else
    // (v1-v2) -> shuffle( y z x )
    #if 1
    float32x4_t a1 = vsetq_lane_f32(vgetq_lane_f32(a, 0), a, 3); // xyzx
    float32x4_t b1 = vsetq_lane_f32(vgetq_lane_f32(b, 0), b, 3); // xyzx
    float32x4_t a_yzx = vextq_f32(a1, a1, 1); // yzxx
    float32x4_t b_yzx = vextq_f32(b1, b1, 1); // yzxx
    #else
    float32x4_t a_yzx = (float32x4_t){ a[1], a[2], a[0], a[0]};
    float32x4_t b_yzx = (float32x4_t){ b[1], b[2], b[0], b[0]};
    #endif

    float32x4_t q1 = vmulq_f32(a, b_yzx);
    float32x4_t q2 = vmlsq_f32(q1, b, a_yzx); // v1-v2

    float32x4_t r1 = vsetq_lane_f32(vgetq_lane_f32(q2, 0), q2, 3); // yzxy
    float32x4_t r = vsetq_lane_f32(0, vextq_f32(r1, r1, 1), 3); // yzx0
#endif

    return r;
}

// quaternion
inline float32x4_t axis_quat(float32x4_t q) {
    float32x4_t oneminussq = vsubq_f32(vdupq_n_f32(1.0f), vdupq_laneq_f32(vmulq_f32(q, q), 3));
    if (vgetq_lane_f32(oneminussq,0) < 1e-6f)
        return float32x4_t{1.0f, 0.0f, 0.0f, 0.0f};
    float32x4_t rsq = rsqrt<1>(oneminussq);
    float32x4_t axis = vmulq_f32(q, rsq);
    return vsetq_lane_f32(0, axis, 3);
}

inline float32x4_t conj_quat(float32x4_t q) {
#if 0
    const uint32x4_t mask = { 0x80000000u, 0x80000000u, 0x80000000u, 0x00000000u };
    return vreinterpretq_f32_u32( veorq_u32(vreinterpretq_u32_f32(q), mask));
#else
    #if 0
    const float32x4_t sign = { -1.0f, -1.0f, -1.0f, 1.0f };
    return vmulq_f32(q, sign);
    #else
    float32x4_t n = vnegq_f32(q);
    return vcopyq_laneq_f32(n, 3, q, 3);
    #endif
#endif
}

// q-1 = 1/(x2+y2+z2+w2)(w-xi-yj-zk)
inline float32x4_t inverse_quat(float32x4_t a) {
    float32x4_t sq = vdupq_n_f32(vaddvq_f32(vmulq_f32(a,a)));
    float32x4_t re = vrecpeq_f32(sq);
    re = vmulq_f32(re, vrecpsq_f32(sq, re));
    return vmulq_f32(re, conj_quat(a));
}

// inline float32x4_t normsq_quat(float32x4_t a) {
//     return vdupq_n_f32(vaddvq_f32(vmulq_f32(a,a)));
// }

// inline float32x4_t normalize_quat(float32x4_t a) {
//     float32x4_t sq = vdupq_n_f32(vaddvq_f32(vmulq_f32(a,a)));
//     float32x4_t re = vrsqrteq_f32(sq);
//     re = vmulq_f32(re, vrsqrtsq_f32(vmulq_f32(sq, re), re));
//     //re = re * vrsqrtsq_f32(sq * re, re);
//     return vmulq_f32(re, a);
// }

// inline float lensq_quat(float32x4_t a) {
//     return vaddvq_f32(vmulq_f32(a,a));
// }

// inline float32x4_t len_quat(float32x4_t a) {
//     float32x4_t sq = vdupq_n_f32(vaddvq_f32(vmulq_f32(a,a)));
//     float32x4_t re = vrsqrteq_f32(sq);
//     re = vmulq_f32(re, vrsqrtsq_f32(vmulq_f32(sq, re), re));
//     //re = vmulq_f32(re, vrsqrtsq_f32(vmulq_f32(sq, re), re));
//     //re = re * vrsqrtsq_f32(sq * re, re);
//     return vdivq_f32(vdupq_n_f32(1.0f), re);
// }

inline float32x4_t quat_from_aa(float32x4_t v, float a) {
    // q = ([v]*sin(a/2), cos(a/2)]
    float half = 0.5f * a;
    float sa = sinf(half);
    float ca = cosf(half);

    // normalize axis
    float32x4_t nv = normalized_3f(v);
    float32x4_t q = vmulq_f32(nv, vdupq_n_f32(sa));
    q = vsetq_lane_f32(ca, q, 3);
    return q;
}

inline float32x4_t quat_fromaa_normalized0(float32x4_t v, float a) {
    // q = ([v]*sin(a/2), cos(a/2)]
    // axis must be normalized
    float half = 0.5f * a;
    float sa = sinf(half);
    float ca = cosf(half);

    float32x4_t q = vmulq_f32(v, vdupq_n_f32(sa));
    q = vsetq_lane_f32(ca, q, 3);
    return q;
}

inline float32x4_t mul_quat(float32x4_t a, float32x4_t b) {

// (+ x1w2 + y1z2 - z1y2 + w1x2)i +
// (- x1z2 + y1w2 + z1x2 + w1y2)j +
// (+ x1y2 - y1x2 + z1w2 + w1z2)k +
// (- x1x2 - y1y2 - z1z2 + w1w2)

    float32x4_t a_xxxx = vdupq_laneq_f32(a, 0);
    float32x4_t a_yyyy = vdupq_laneq_f32(a, 1);
    float32x4_t a_zzzz = vdupq_laneq_f32(a, 2);
    float32x4_t a_wwww = vdupq_laneq_f32(a, 3);

    float32x4_t b_yxwz = vrev64q_f32(b);
    float32x4_t b_wzyx = vextq_f32(b_yxwz, b_yxwz, 2); 
    float32x4_t b_zwxy = vextq_f32(b, b, 2);

    float32x4_t sx = {+1.0f, -1.0f, +1.0f, -1.0f};
    float32x4_t sy = {+1.0f, +1.0f, -1.0f, -1.0f};
    float32x4_t sz = {-1.0f, +1.0f, +1.0f, -1.0f};

    float32x4_t r = vmulq_f32(   a_wwww, b);
                r = vmlaq_f32(r, a_xxxx, vmulq_f32(b_wzyx, sx));
                r = vmlaq_f32(r, a_yyyy, vmulq_f32(b_zwxy, sy));
                r = vmlaq_f32(r, a_zzzz, vmulq_f32(b_yxwz, sz));
    return r;
}

inline float32x4_t rot_quat_vec3(float32x4_t q, float32x4_t v) {
    #if 0
    // v' = q*v*q-1
    float32x4_t q_conj = conj_quat(q);
    // float32x4_t v_as_quat = vsetq_lane_f32(0.0f, v, 3);
    float32x4_t v_as_quat = v;
    float32x4_t qv = mul_quat(q, v_as_quat);
    float32x4_t rqv = mul_quat(qv, q_conj);
    return vsetq_lane_f32(0.0f, rqv, 3);
    #elif 0
    // optimized version (1)
    // u = (qx,qy,qz), s = w
    // t = 2 * cross(u, v)
    // v' = v + s * t + cross(u, t)
    float32x4_t t_xyz = cross(q, v);
    t_xyz = vaddq_f32(t_xyz, t_xyz);

    float32x4_t o = vmlaq_f32(v, t_xyz, vdupq_laneq_f32(q, 3));
    o = vaddq_f32(o, cross(q, t_xyz));
    return vsetq_lane_f32(0.0f, o, 3);
    #else
    // optimized version (2)
    // u = (qx,qy,qz), s = w
    // t = 2 * cross(u, v)
    // v' = v + s * t + cross(u, t)
    auto perm_yzx = [](float32x4_t v) -> float32x4_t {
        float32x4_t v1 = vsetq_lane_f32(vgetq_lane_f32(v, 0), v, 3); // xyzx
        return vextq_f32(v1, v1, 1); // yzxz
    };

    // [yzx*xyz - xyz*yzx] (v1-v2) -> shuffle( y z x )
    float32x4_t v_yzx = perm_yzx(v);
    float32x4_t q_yzx = perm_yzx(q);

    // _ (ay*bz, az*bx, ax*by) [v1]
    //   (az*by, ax*bz, ay*bx) [v2]
    // ax ay az * by bz bx -> v1
    // ay az ax * bx by bz -> v2
    float32x4_t x1_zxy = vmulq_f32(q, v_yzx);
                x1_zxy = vmlsq_f32(x1_zxy, v, q_yzx); // v1-v2 in (Z:ax*by-bx*ay, X:ay*bz - by*az, Y:az*bx - ax*bz)
    float32x4_t t_zxy = vaddq_f32(x1_zxy, x1_zxy);
    float32x4_t t = perm_yzx(t_zxy); // t in XYZ order

    // q x t cross product, we do qyzx*tzxy - qzxy*tyzx. For that we need to swizzle out qzxy and tyzx
    // which we get by perm_yzx once more on yzx
    float32x4_t q_zxy = perm_yzx(q_yzx);
    float32x4_t t_yzx = perm_yzx(t);

    float32x4_t x2 = vmulq_f32(q_yzx, t_zxy);
                x2 = vmlsq_f32(x2, t_yzx, q_zxy); // v1-v2 in XYZ order

    float32x4_t vo = vmlaq_f32(v, t, vdupq_laneq_f32(q, 3));
                vo = vaddq_f32(vo, x2);

    return vsetq_lane_f32(0.0f, vo, 3);
    #endif
}

// matrix
inline float32x4x4_t mul(float32x4x4_t a, float32x4x4_t b)
{
	float32x4x4_t m;

    m.val[0] = vmulq_laneq_f32(          a.val[0], b.val[0], 0);
    m.val[1] = vmulq_laneq_f32(          a.val[0], b.val[1], 0);
    m.val[2] = vmulq_laneq_f32(          a.val[0], b.val[2], 0);
    m.val[3] = vmulq_laneq_f32(          a.val[0], b.val[3], 0);

    m.val[0] = vmlaq_laneq_f32(m.val[0], a.val[1], b.val[0], 1);
    m.val[1] = vmlaq_laneq_f32(m.val[1], a.val[1], b.val[1], 1);
    m.val[2] = vmlaq_laneq_f32(m.val[2], a.val[1], b.val[2], 1);
    m.val[3] = vmlaq_laneq_f32(m.val[3], a.val[1], b.val[3], 1);

    m.val[0] = vmlaq_laneq_f32(m.val[0], a.val[2], b.val[0], 2);
    m.val[1] = vmlaq_laneq_f32(m.val[1], a.val[2], b.val[1], 2);
    m.val[2] = vmlaq_laneq_f32(m.val[2], a.val[2], b.val[2], 2);
    m.val[3] = vmlaq_laneq_f32(m.val[3], a.val[2], b.val[3], 2);

    m.val[0] = vmlaq_laneq_f32(m.val[0], a.val[3], b.val[0], 3);
    m.val[1] = vmlaq_laneq_f32(m.val[1], a.val[3], b.val[1], 3);
    m.val[2] = vmlaq_laneq_f32(m.val[2], a.val[3], b.val[2], 3);
    m.val[3] = vmlaq_laneq_f32(m.val[3], a.val[3], b.val[3], 3);
    
	return m;
}

inline float32x4x3_t mul33(float32x4x3_t a, float32x4x3_t b)
{
	float32x4x3_t m;

    assert(a.val[0][3] == 0);
    assert(a.val[1][3] == 0);
    assert(a.val[2][3] == 0);

    m.val[0] = vmulq_laneq_f32(          a.val[0], b.val[0], 0);
    m.val[1] = vmulq_laneq_f32(          a.val[0], b.val[1], 0);
    m.val[2] = vmulq_laneq_f32(          a.val[0], b.val[2], 0);

    m.val[0] = vmlaq_laneq_f32(m.val[0], a.val[1], b.val[0], 1);
    m.val[1] = vmlaq_laneq_f32(m.val[1], a.val[1], b.val[1], 1);
    m.val[2] = vmlaq_laneq_f32(m.val[2], a.val[1], b.val[2], 1);

    m.val[0] = vmlaq_laneq_f32(m.val[0], a.val[2], b.val[0], 2);
    m.val[1] = vmlaq_laneq_f32(m.val[1], a.val[2], b.val[1], 2);
    m.val[2] = vmlaq_laneq_f32(m.val[2], a.val[2], b.val[2], 2);

    m.val[0] = vmlaq_laneq_f32(m.val[0], a.val[3], b.val[0], 3);
    m.val[1] = vmlaq_laneq_f32(m.val[1], a.val[3], b.val[1], 3);
    m.val[2] = vmlaq_laneq_f32(m.val[2], a.val[3], b.val[2], 3);
    
	return m;
}


// matrix
inline float32x4_t mul3(float32x4x4_t m, float32x4_t v)
{
	float32x4_t r = vmulq_laneq_f32(   m.val[0], v, 0);
                r = vmlaq_laneq_f32(r, m.val[1], v, 1);
                r = vmlaq_laneq_f32(r, m.val[2], v, 2);
    return r;
}

inline float32x4_t mul4(float32x4x4_t m, float32x4_t v)
{
	float32x4_t r;
    r = vmulq_laneq_f32(   m.val[0], v, 0);
    r = vmlaq_laneq_f32(r, m.val[1], v, 1);
    r = vmlaq_laneq_f32(r, m.val[2], v, 2);
    r = vmlaq_laneq_f32(r, m.val[3], v, 3);
    return r;
}

inline float32x4x4_t transpose(float32x4x4_t a) {
#if defined(__aarch64__)
    float32x4_t t0 = vtrn1q_f32(a.val[0], a.val[1]);
    float32x4_t t1 = vtrn2q_f32(a.val[0], a.val[1]);
    float32x4_t t2 = vtrn1q_f32(a.val[2], a.val[3]);
    float32x4_t t3 = vtrn2q_f32(a.val[2], a.val[3]);

    float32x4_t c0 = vcombine_f32(vget_low_f32(t0), vget_low_f32(t2));
    float32x4_t c1 = vcombine_f32(vget_low_f32(t1), vget_low_f32(t3));
    float32x4_t c2 = vcombine_f32(vget_high_f32(t0), vget_high_f32(t2));
    float32x4_t c3 = vcombine_f32(vget_high_f32(t1), vget_high_f32(t3));
    return float32x4x4_t { c0, c1, c2, c3 };
#else
    float32x4x2_t t0 = vtrnq_f32(a.val[0], a.val[1]);
    float32x4x2_t t1 = vtrnq_f32(a.val[2], a.val[3]);

    float32x4x4_t r;
    r.val[0] = vcombine_f32( vget_low_f32(t0.val[0]),  vget_low_f32(t1.val[0]));
    r.val[1] = vcombine_f32( vget_low_f32(t0.val[1]),  vget_low_f32(t1.val[1]));
    r.val[2] = vcombine_f32(vget_high_f32(t0.val[0]), vget_high_f32(t1.val[0]));
    r.val[3] = vcombine_f32(vget_high_f32(t0.val[1]), vget_high_f32(t1.val[1]));

    return r;
#endif
}

inline float32x4x4_t load_transpose(float const* a) {
    return vld4q_f32(a);
}

inline void store_transpose(float* p, float32x4x4_t a) {
    vst4q_f32(p, a);
}

// [ xx  yx  zx  Tx ]
// [ xy  yy  zy  Ty ]
// [ xz  yz  zz  Tz ]
// [  0   0   0   1 ]
inline float32x4x4_t inverse_tr(float32x4x4_t m) {
#if 1
    float32x4_t t0 = vtrn1q_f32(m.val[0], m.val[1]);
    float32x4_t t1 = vtrn2q_f32(m.val[0], m.val[1]);
    float32x4_t t2 = vtrn1q_f32(m.val[2], m.val[3]);
    float32x4_t t3 = vtrn2q_f32(m.val[2], m.val[3]);

    float32x4_t c0 = vcombine_f32(vget_low_f32(t0), vget_low_f32(t2));
    float32x4_t c1 = vcombine_f32(vget_low_f32(t1), vget_low_f32(t3));
    float32x4_t c2 = vcombine_f32(vget_high_f32(t0), vget_high_f32(t2));
#else
    float32x4_t c0 = { vgetq_lane_f32(m.val[0], 0), vgetq_lane_f32(m.val[1], 0), vgetq_lane_f32(m.val[2], 0), 0.0f };
    float32x4_t c1 = { vgetq_lane_f32(m.val[0], 1), vgetq_lane_f32(m.val[1], 1), vgetq_lane_f32(m.val[2], 1), 0.0f };
    float32x4_t c2 = { vgetq_lane_f32(m.val[0], 2), vgetq_lane_f32(m.val[1], 2), vgetq_lane_f32(m.val[2], 2), 0.0f };
#endif

    float32x4_t t = m.val[3];
    float32x4_t c3 =-vmulq_f32(c0, t);
                c3 = vfmsq_f32(c3, c1, t);
                c3 = vfmsq_f32(c3, c2, t);

    return (float32x4x4_t){ c0, c1, c2, c3 };
}

float32x4x4_t inverse_trs(float32x4x4_t a);

} // neon::impl

} // stir

#endif // STIR_MATH_NEON_IMPL_HPP_
