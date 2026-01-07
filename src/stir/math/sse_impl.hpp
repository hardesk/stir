#ifndef STIR_MATH_SSE_IMPL_HPP_
#define STIR_MATH_SSE_IMPL_HPP_

#include "sse_prefix.hpp"

namespace stir::sse::impl {

// TODO: implement __vectorcall

constexpr float EPS = 1e-30f;

inline __m128 addf(__m128 a, __m128 b) {
    return _mm_add_ps(a, b);
}

inline __m128 subf(__m128 a, __m128 b) {
    return _mm_sub_ps(a, b);
}

inline __m128 mulf(__m128 a, __m128 b) {
    return _mm_mul_ps(a, b);
}

inline __m128 mulf(__m128 a, float b) {
    return _mm_mul_ps(a, _mm_set1_ps(b));
}

template<unsigned PRECISE = 1>
inline __m128 recp(__m128 a) {
    __m128 r = _mm_rcp_ps(a);
    // refine r using one Newton-Raphson step: x = x * (2 - a*x)
    r = _mm_mul_ps(r, _mm_sub_ps(_mm_set1_ps(2.0f), _mm_mul_ps(a, r)));
    if (PRECEISE >= 2)
        r = _mm_mul_ps(r, _mm_sub_ps(_mm_set1_ps(2.0f), _mm_mul_ps(a, r)));
    return r;
}

template<unsigned PRECISE = 1>
inline __m128 rsqrts(__m128 a) {
    __m128 half = _mm_set1_ss(0.5f);
    __m128 r = _mm_rsqrt_ss(a);
    // refine rsqrt using Newton-Raphson step: x = x * (3 - a*x*x)/2
    r = _mm_mul_ss(_mm_mul_ss(r, _mm_sub_ss(_mm_set1_ss(3.0f), _mm_mul_ss(_mm_mul_ss(a, r), r)), half));
    if (PRECISE >= 2)
        r = _mm_mul_ss(_mm_mul_ss(r, _mm_sub_ss(_mm_set1_ss(3.0f), _mm_mul_ss(_mm_mul_ss(a, r), r)), half));
    return r;
}

template<unsigned PRECISE = 1>
inline __m128 rsqrt(__m128 a) {
    __m128 half = _mm_set1_ps(0.5f);
    __m128 r = _mm_rsqrt_ps(a);
    // refine rsqrt using Newton-Raphson step: x = x * (3 - a*x*x)/2
    r = _mm_mul_ps(_mm_mul_ps(r, _mm_sub_ps(_mm_set1_ps(3.0f), _mm_mul_ps(_mm_mul_ps(a, r), r)), half));
    if (PRECISE >= 2)
        r = _mm_mul_ps(_mm_mul_ps(r, _mm_sub_ps(_mm_set1_ps(3.0f), _mm_mul_ps(_mm_mul_ps(a, r), r)), half));
    return r;
}

template<unsigned PRECISE = 1>
inline float len2(__m128 a) {
    __m128 sq = _mm_mul_ps(a, a);
    sq = _mm_hadd_ps(sq, sq);
    sq = _mm_max_ss(sq, _mm_set1_ss(EPS));
    __m128 rsq = rsqrts<PRECISE>(sq);
    return _mm_cvtss_f32(_mm_mul_ss(sq, rsq));
}

template<unsigned PRECISE = 1>
inline float len4(__m128 a) {
    __m128 sq = _mm_mul_ps(a, a);
    sq = _mm_hadd_ps(sq, sq);
    sq = _mm_hadd_ps(sq, sq);
    sq = _mm_max_ss(sq, _mm_set1_ss(EPS));
    __m128 rsq = rsqrts<PRECISE>(sq);
    return _mm_cvtss_f32(_mm_mul_ss(sq, rsq));
}

inline float dot4(__m128 a, __m128 b) {
    __m128 mul = _mm_mul_ps(a, b);
    mul = _mm_hadd_ps(mul, mul);
    mul = _mm_hadd_ps(mul, mul);
    return _mm_cvtss_f32(mul);
}

template<unsigned PRECISE = 1>
inline float inv_len4(__m128 a) {
    __m128 sq = _mm_mul_ps(a, a);
    sq = _mm_hadd_ps(sq, sq);
    sq = _mm_hadd_ps(sq, sq);
    sq = _mm_max_ss(sq, _mm_set1_ss(EPS));
    __m128 rsq = rsqrts<PRECISE>(sq);
    return _mm_cvtss_f32(rsq);
}

template<unsigned PRECISE = 1>
inline __m128 normalized_4f(__m128 a) {
    __m128 sq = _mm_mul_ps(a, a);
    sq = _mm_hadd_ps(sq, sq);
    sq = _mm_hadd_ps(sq, sq);
    sq = _mm_shuffle_ps(sq, sq, _MM_SHUFFLE(0,0,0,0));
    sq = _mm_max_ps(sq, _mm_set1_ps(EPS));
    __m128 rsq = rsqrt<PRECISE>(sq);
    return _mm_mul_ps(a, rsq);
}

inline __m128 cross(__m128 a, __m128 b) {
    // r = ay*bz - az*by, az*bx - ax*bz, ax*by - ay*bx
    //   _ ax*by, ay*bz, az*bx
    //     ay*bx, az*by, ax*bz
    //       ->Z    ->X    ->Y
    __m128 a_yzx = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3,0,2,1));
    __m128 b_yzx = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3,0,2,1));
    __m128 c = subf(mulf(a, b_yzx), mulf(b, a_yzx));
    return _mm_shuffle_ps(c, c, _MM_SHUFFLE(3,0,2,1));
}

inline __m128 conj_quat(__m128 a) {
    return _mm_mul_ps(a, _mm_set_ps(-1.0f,-1.0f,-1.0f,1.0f));
}

template<unsigned PRECISE = 1>
inline __m128 inverse_quat(__m128 a) {
    // q-1 = 1/(x2+y2+z2+w2)(w-xi-yj-zk)
    __m128 sq = _mm_mul_ps(a, a);
    sq = _mm_hadd_ps(sq, sq);
    sq = _mm_hadd_ps(sq, sq);
    sq = _mm_shuffle_ps(sq, sq, _MM_SHUFFLE(0,0,0,0));
    sq = _mm_max_ps(sq, _mm_set1_ps(EPS));
    __m128 re = recp<PRECISE>(sq);
    __m128 c = conj_quat(a);
    return _mm_mul_ps(c, re);
}

inline __m128 mul_quat(__m128 a, __m128 b)
{
    // x/0 (+ a.x*b.w + a.y*b.z - a.z*b.y + a.w*b.x)i +
    // y/1 (- a.x*b.z + a.y*b.w + a.z*b.x + a.w*b.y)j +
    // z/2 (+ a.x*b.y - a.y*b.x + a.z*b.w + a.w*b.z)k +
    // w/3 (- a.x*b.x - a.y*b.y - a.z*b.z + a.w*b.w)
    __m128 ax = _mm_shuffle_ps(a, a, _MM_SHUFFLE(0,0,0,0));
    __m128 ay = _mm_shuffle_ps(a, a, _MM_SHUFFLE(1,1,1,1));
    __m128 az = _mm_shuffle_ps(a, a, _MM_SHUFFLE(2,2,2,2));
	__m128 aw = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3,3,3,3));
	
	__m128 b_yxwz = _mm_shuffle_ps(b, b, _MM_SHUFFLE(2,3,0,1)); 
	__m128 b_zwxy = _mm_shuffle_ps(b, b, _MM_SHUFFLE(1,0,3,2));
	__m128 b_xzyw = _mm_shuffle_ps(b, b, _MM_SHUFFLE(3,1,2,0));
	__m128 b_ywzx = _mm_shuffle_ps(b, b, _MM_SHUFFLE(0,2,3,1));

	// reshuffle the operands so that we can use addsub: - + - +
	//        |----------------------------------------- qc ----------------|
	//        |-------------------- qb ----------------|
	//        |-------- qa -------|
	//            q0          q1                   q2                   q3
    // y(1) = a.w * b.y - a.x * b.z [w/3->] - a.y * b.x [y/1->] - a.z * b.y [x/0->]
    // x(0) = a.w * b.x + a.x * b.w [y/1->] + a.y * b.z [x/0->] + a.z * b.w [w/3->]
    // w(3) = a.w * b.w - a.x * b.x [z/2->] - a.y * b.y [z/2->] - a.z * b.z [y/1->]
    // z(2) = a.w * b.z + a.x * b.y [x/0->] + a.y * b.w [w/3->] + a.z * b.x [z/2->]

	__m128 q0 = _mm_mul_ps(aw, b_yxwz);
	__m128 q1 = _mm_mul_ps(ax, b_zwxy);
	__m128 q2 = _mm_mul_ps(ay, b_xzyw);
	__m128 q3 = _mm_mul_ps(az, b_ywzx);

	__m128 qa = _mm_addsub_ps(q0, q1);
           qa = _mm_shuffle_ps(qa, qa, _MM_SHUFFLE(0,2,1,3));
	__m128 qb = _mm_addsub_ps(qa, q2);
           qb = _mm_shuffle_ps(qb, qb, _MM_SHUFFLE(3,2,0,1));
	__m128 qc = _mm_addsub_ps(qb, q3);
           qc = _mm_shuffle_ps(qc, qc, _MM_SHUFFLE(2,1,3,0));
	
	return qc;
}

inline F16 mul(F16 a, F16 b)
{
    F16 r;

	for (int i=0; i<4; ++i)
	{
        __m128 b0 = _mm_shuffle_ps(b.col[i], b.col[i], _MM_SHUFFLE(0,0,0,0));
        __m128 b1 = _mm_shuffle_ps(b.col[i], b.col[i], _MM_SHUFFLE(1,1,1,1));
        __m128 b2 = _mm_shuffle_ps(b.col[i], b.col[i], _MM_SHUFFLE(2,2,2,2));
        __m128 b3 = _mm_shuffle_ps(b.col[i], b.col[i], _MM_SHUFFLE(3,3,3,3));

        __m128 x = _mm_mul_ps(a.col[0], b0);
#if defined(__FMA__)
               x = _mm_fmadd_ps(x, a.col[1], b1);
               x = _mm_fmadd_ps(x, a.col[2], b2);
               x = _mm_fmadd_ps(x, a.col[3], b3);
#else
               x = _mm_add_ps(x, _mm_mul_ps(a.col[1], b1));
               x = _mm_add_ps(x, _mm_mul_ps(a.col[2], b2));
               x = _mm_add_ps(x, _mm_mul_ps(a.col[3], b3));
#endif
        r.col[i] = x;
	}

	return r;
}

inline F16 transpose(F16 m)
{
	F16 r;

	__m128 u1 = _mm_unpackhi_ps(m.col[2], m.col[0]);
	__m128 u2 = _mm_unpacklo_ps(m.col[2], m.col[0]);
	__m128 u3 = _mm_unpackhi_ps(m.col[3], m.col[1]);
	__m128 u4 = _mm_unpacklo_ps(m.col[3], m.col[1]);

	r.col[0] = _mm_unpackhi_ps(u4, u2);
	r.col[1] = _mm_unpacklo_ps(u4, u2);
	r.col[2] = _mm_unpackhi_ps(u3, u1);
	r.col[3] = _mm_unpacklo_ps(u3, u1);

	return r;
}

inline __m128 mul4(F16 m, __m128 v)
{
	__m128 r =               _mm_mul_ps(m.col[0], _mm_shuffle_ps(v, v, _MM_SHUFFLE(0,0,0,0)));
           r = _mm_add_ps(r, _mm_mul_ps(m.col[1], _mm_shuffle_ps(v, v, _MM_SHUFFLE(1,1,1,1))));
           r = _mm_add_ps(r, _mm_mul_ps(m.col[2], _mm_shuffle_ps(v, v, _MM_SHUFFLE(2,2,2,2))));
           r = _mm_add_ps(r, _mm_mul_ps(m.col[3], _mm_shuffle_ps(v, v, _MM_SHUFFLE(3,3,3,3))));
    return r;
}


} // stir::sse::impl

#endif // STIR_MATH_SSE_IMPL_HPP_
