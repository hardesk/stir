#ifndef STIR_MATH_COMMON_HPP_
#define STIR_MATH_COMMON_HPP_

namespace stir
{

inline quat operator*(quat const& a, quat const& b) { return mul(a, b); }

inline float rsqrt(float a) { return 1.0f/sqrt(a); }

}


#endif //  STIR_MATH_COMMON_HPP_
