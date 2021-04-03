#ifndef STIR_MATH_CONFIG_HPP_
#define STIR_MATH_CONFIG_HPP_

#define STIR_MATH_REF 0
#define STIR_MATH_MIPP 1
#define STIR_MATH_XSIMD 2
//#define STIR_MATH_MODE STIR_MATH_REF
#define STIR_MATH_MODE STIR_MATH_MIPP

#if defined(__x86_64)
#define STIR_MATH_X64 1
#else
#define STIR_MATH_X64 0
#endif

#define STIR_MATH_X64 1

namespace stir
{
}

#endif // STIR_MATH_CONFIG_HPP_
