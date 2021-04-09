#ifndef STIR_MATH_HPP_
#define STIR_MATH_HPP_

#include <cmath>
#include <cstring>
#include <type_traits>
#include <initializer_list>


// define types
#include "math_def.hpp"

// define "implementation" functions
#if STIR_MATH_MODE == STIR_MATH_MIPP
#include "math_mipp.hpp"
#elif STIR_MATH_MODE == STIR_MATH_XSIMD
#include "math_xsimd.hpp"
#else
#include "math_ref.hpp"
#endif

// define common functions that are implemented in terms of "implementation"
#include "math_common.hpp"

// always (unless specifically disabled) include the reference implementation
#include "math_ref.hpp"

// Vector is interpreted as 1x4 (a row) matrix.
// Thus vector/matrix multiplication happens as v * M

#if 0

namespace stir
{

inline Vector4 mul(Vector3 const& v, Matrix const& a)
{
#if 1
	BF4 v0 = BF4(v.x);
	BF4 v1 = BF4(v.y);
	BF4 v2 = BF4(v.z);
#else
	BF4 vv(v.as_vec4().simd_data());
	BF4 v0 = BF4(vv[0]);
	BF4 v1 = BF4(vv[1]);
	BF4 v2 = BF4(vv[2]);
#endif

	return Vector4(v0*a.row[0] + v1*a.row[1] + v2*a.row[2] + a.row[3]);
}

inline Vector3 mul3(Vector3 const& v, Matrix const& a)
{
	BF4 v0 = BF4(v.x);
	BF4 v1 = BF4(v.y);
	BF4 v2 = BF4(v.z);

	BF4 r = v0*a.row[0] + v1*a.row[1] + v2*a.row[2];

	return Vector3(r[0], r[1], r[2]);
}


}


#endif // 0

#endif
