#include "math_ref.hpp"
#include "math_mipp.hpp"

namespace stir
{

#if STIR_MATH_MODE == STIR_MATH_MIPP
inline
#endif
namespace simd_mipp {

/*matrix mul(matrix const& a, matrix const& b)
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
		row.store(m.x+4*i);
	}

	return m;
}*/

}

}

