#ifndef STIR_MATH_SSE_HPP_
#define STIR_MATH_SSE_HPP_

#include "math_def.hpp"
#include <xmmintrin.h> // sse
#include <emmintrin.h> // sse2
#include <pmmintrin.h> // sse3

namespace stir::sse {

// _MM_SHUFFLE index 0 is right-most (least significant little endian), so x:0 y:1 z:2 w:3
#define mm_shufps(r,i) _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(r), i))
//#define mm_shufps(r,i) _mm_shuffle_ps(r, r, i)
//
//inline vec2 as_vec2(S s) { return vec2(s[0], s[1]); }
inline vec3 as_vec3(__m128 s) {
	vec3 v;
	_mm_storeu_si64(&v.x,_mm_castps_si128(s));
	_mm_storeu_si32(&v.z,_mm_shuffle_epi32(_mm_castps_si128(s), _MM_SHUFFLE(2,2,2,2)));
	return v;
}
//inline vec3 as_vec3(S s) { return vec3(s[0], s[1], s[2]); }

inline vec4 as_vec4(__m128 s) { vec4 v; _mm_storeu_ps(&v.x, s); return v; }
inline quat as_quat(__m128 s) { quat v; _mm_storeu_ps(&v.x, s); return v; }

inline quat mul(quat const& a, quat const& b)
{
	// _MM_SHUFFLE index 0 is right-most (least significant little endian), so x:0 y:1 z:2 w:3
	__m128 aa = _mm_loadu_ps(&a.x);
	__m128 aw = mm_shufps(aa, 0xff), ax = mm_shufps(aa, 0x00), ay = mm_shufps(aa, 0x55), az = mm_shufps(aa, 0xaa);
	
	__m128 bb = _mm_loadu_ps(&b.x);
	__m128 b0 = mm_shufps(bb, _MM_SHUFFLE(2,3,0,1)); 
	__m128 b1 = mm_shufps(bb, _MM_SHUFFLE(1,0,3,2));
	__m128 b2 = mm_shufps(bb, _MM_SHUFFLE(3,1,2,0));
	__m128 b3 = mm_shufps(bb, _MM_SHUFFLE(0,2,3,1));

    float x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    float y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    float z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
	float w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;

	// reshuffle the operands so that we can use addsub: - + - +
	//        |----------------------------------------- qc ----------------|
	//        |-------------------- qb ----------------|
	//        |-------- qa -------|
	//            q0          q1                   q2                   q3
    // y(1) = a.w * b.y - a.x * b.z -> [z(3)] - a.y * b.x -> [x(1)] - a.z * b.y -> [0]
    // x(0) = a.w * b.x + a.x * b.w -> [x(1)] + a.y * b.z -> [z(0)] + a.z * b.w -> [3]
	// w(3) = a.w * b.w - a.x * b.x -> [w(2)] - a.y * b.y -> [w(2)] - a.z * b.z -> [1]
    // z(2) = a.w * b.z + a.x * b.y -> [y(0)] + a.y * b.w -> [y(3)] + a.z * b.x -> [2]
	//              1           2                     0                    1
	//              0           3                     2                    3
	//              3           0                     1                    2
	//              2           1                     3                    0

	__m128 q0 = _mm_mul_ps(aw, b0);
	__m128 q1 = _mm_mul_ps(ax, b1);
	__m128 q2 = _mm_mul_ps(ay, b2);
	__m128 q3 = _mm_mul_ps(az, b3);

	__m128 qa = mm_shufps(_mm_addsub_ps(q0, q1), _MM_SHUFFLE(0,2,1,3));
	__m128 qb = mm_shufps(_mm_addsub_ps(qa, q2), _MM_SHUFFLE(3,2,0,1));
	__m128 qc = mm_shufps(_mm_addsub_ps(qb, q3), _MM_SHUFFLE(2,1,3,0));
	
	return as_quat(qc);
}

inline matrix mul(matrix const& a, matrix const& b)
{
	matrix r;

	__m128 b0 = _mm_loadu_ps(b.x + 0*4);
	__m128 b1 = _mm_loadu_ps(b.x + 1*4);
	__m128 b2 = _mm_loadu_ps(b.x + 2*4);
	__m128 b3 = _mm_loadu_ps(b.x + 3*4);

	for (int i=0; i<4; ++i)
	{
		__m128i aa = _mm_loadu_ps(a.x + i*4);
		__m128i a0 = mm_shufps(aa, 0x00);
		__m128i a1 = mm_shufps(aa, 0x55);
		__m128i a2 = mm_shufps(aa, 0xaa);
		__m128i a3 = mm_shufps(aa, 0xff);

		__m128i rr = _mm_add_ps(
				_mm_add_ps(_mm_mul_ps(a0, b0), _mm_mul_ps(a1, b1)),
				_mm_add_ps(_mm_mul_ps(a2, b2), _mm_mul_ps(a3, b3))); 
		_mm_storeu_ps(r.x + i*4, rr);
	}

	return r;
}

inline matrix transpose(matrix const& m)
{
	matrix r;

	__m128 a = _mm_loadu_ps(m.x + 0*4);
	__m128 b = _mm_loadu_ps(m.x + 1*4);
	__m128 c = _mm_loadu_ps(m.x + 2*4);
	__m128 d = _mm_loadu_ps(m.x + 3*4);

	__m128 u1=_mm_unpackhi_ps(c, a);
	__m128 u2=_mm_unpacklo_ps(c, a);
	__m128 u3=_mm_unpackhi_ps(d, b);
	__m128 u4=_mm_unpacklo_ps(d, b);

	_mm_storeu_ps(r.x + 0*4, _mm_unpackhi_ps(u4, u2));
	_mm_storeu_ps(r.x + 1*4, _mm_unpacklo_ps(u4, u2));
	_mm_storeu_ps(r.x + 2*4, _mm_unpackhi_ps(u3, u1));
	_mm_storeu_ps(r.x + 3*4, _mm_unpacklo_ps(u3, u1));

	return r;
}

}

#endif // STIR_MATH_SSE_HPP_

