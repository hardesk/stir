#ifndef STIR_MATH_HPP_
#define STIR_MATH_HPP_

#include <cmath>
#include <cstring>
#include <type_traits>
#include <initializer_list>

#define STIR_MATH_REF 0
#define STIR_MATH_MIPP 1
#define STIR_MATH_XSIMD 2
#define STIR_MATH_NEON 3
// #define STIR_MATH_MODE STIR_MATH_REF
#define STIR_MATH_MODE STIR_MATH_NEON

#if defined(__x86_64)
#define STIR_MATH_X64 1
#else
#define STIR_MATH_X64 0
#endif

//#define STIR_MATH_X64 1

// The prefix files contain the underlying data definition and minimal accessor
// functions and initialization functions needed to initialize vec/quat/matrix types.
#if STIR_MATH_MODE == STIR_MATH_NEON
#include "neon_prefix.hpp"
namespace stir::simd {
    using namespace neon;
}
#elif STIR_MATH_MODE == STIR_MATH_REF
#include "ref_prefix.hpp"
namespace stir::simd {
    using namespace ref;
}
#elif STIR_MATH_MODE == STIR_MATH_MIPP
#include "mipp_prefix.hpp"
#endif

namespace stir
{
}

// define our types, it uses the simd types defined in prefix file
#include "defs.hpp"

// always (unless specifically disabled) include the reference implementation
#include "ref.hpp"

// define "implementation" functions
#if STIR_MATH_MODE == STIR_MATH_MIPP
//#include "math_mipp.hpp"
#elif STIR_MATH_MODE == STIR_MATH_MIPP
#include "math_xsimd.hpp"
#elif STIR_MATH_MODE == STIR_MATH_NEON
#include "neon.hpp"
namespace stir::simd { using namespace stir::neon; }
#else
//#include "math_ref.hpp"
#endif

// define common functions that are implemented in terms of "implementation"
//#include "common.hpp"

// Vector is interpreted as 1x4 (a row) matrix.
// Thus vector/matrix multiplication happens as v * M

namespace stir {

inline void vec2::normalize() { *this = simd::normalize(*this); }
inline void vec3::normalize() { *this = simd::normalize(*this); }
inline void vec4::normalize() { *this = simd::normalize(*this); }
inline void quat::normalize() { *this = simd::normalize(*this); }

inline vec3 quat::axis() const { return simd::axis(*this); }

// invertTRS()        // hot path
// invertRigid()
// invertAffine()
// invertProjection()
// invert4x4_LU()     // slow fallback

using namespace simd;

}

#endif
