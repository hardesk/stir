#ifndef STIR_MATH_XSIMD_HPP_
#define STIR_MATH_XSIMD_HPP_

#include "math_def.hpp"
#include <xsimd/xsimd.hpp>

#define Ax (&a.x)
#define Bx (&b.x)
#define Tx (&this->x)
#define Xx(a) (&a.x)

namespace stir
{
#if STIR_MATH_MODE == STIR_MATH_XSIMD
inline
#endif
namespace simd_xsimd
{

constexpr int Sn = 4;
using S = xsimd::batch<float, Sn>;
using M = xsimd::batch_bool<float, Sn>;

inline vec2 as_vec2(S s) { return vec2(s[0], s[1]); }
inline vec3 as_vec3(S s) { return vec3(s[0], s[1], s[2]); }
inline vec4 as_vec4(S s) { return vec4(s[0], s[1], s[2], s[3]); }
inline quat as_quat(S s) { return quat(s[0], s[1], s[2], s[3]); }

inline quat mul(quat const& a, quat const& b)
{
	S aa(Ax);
	S a3(aa[3]), a0(aa[0]), a1(aa[1]), a2(aa[2]);

	S bb(Bx);

	S bs0{ bb[0], bb[1], bb[2], bb[3]};
	S bs1{ bb[3],-bb[2], bb[1],-bb[0]};
	S bs2{ bb[2], bb[3],-bb[0],-bb[1]};
	S bs3{-bb[1], bb[0], bb[3],-bb[2]};

	S q0 = a3 * bs0;
	S q1 = a0 * bs1;
	S q2 = a1 * bs2;
	S q3 = a2 * bs3;
	return as_quat(q0 + q1 + q2 + q3);
}

inline matrix mul(matrix const& a, matrix const& b)
{
	matrix m;
	S b0(b.x+4*0), b1(b.x+4*1), b2(b.x+4*2), b3(b.x+4*3);

	for (int i=0; i<4; ++i)
	{
		S aa(a.x+4*i);
		S a0 = S(aa[0]);
		S a1 = S(aa[1]);
		S a2 = S(aa[2]);
		S a3 = S(aa[3]);

		S row = a0*b0 + a1*b1 + a2*b2 + a3*b3;
		row.store_unaligned(m.x+4*i);
	}

	return m;
}

inline matrix transpose(matrix const& m)
{
	matrix r;

	S a(m.x+0*4), b(m.x+1*4), c(m.x+2*4), d(m.x+3*4);

	// see ref.cpp:transpose for what we want to achieve
	// hi(c a)	cw aw cz az //1			L(4 2) dx cx bx ax	
	// lo (c a) cy ay cx ax //2 -->		H(4 2) dy cy by ay
	// hi(d b)	dw bw dz bz //3 		L(3 1) dz cz bz az
	// lo (d b) dy by dx bx //4			H(3 1) dw cw bw aw
	
	return matrix(
			a[0], b[0], c[0], d[0],	
			a[1], b[1], c[1], d[1],	
			a[2], b[2], c[2], d[2],	
			a[3], b[3], c[3], d[3]);
}

} // simd_xsimd
	
}

#undef Ax
#undef Bx
#undef Xx
#undef Tx


#endif
