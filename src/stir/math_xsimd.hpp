#ifndef STIR_MATH_XSIMD_HPP_
#define STIR_MATH_XSIMD_HPP_

#include "math_def.hpp"

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
	return quat(q0 + q1 + q2 + q3);
}

} // simd_xsimd
	
}

#undef Ax
#undef Bx
#undef Xx
#undef Tx


#endif
