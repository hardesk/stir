#ifndef STIR_MATH_COMMON_HPP_
#define STIR_MATH_COMMON_HPP_

namespace stir
{

inline quat operator*(quat const& a, quat const& b) { return mul(a, b); }

inline vec3 rot(vec3 const& v, quat const& q)
{
	quat qv(v.x, v.y, v.z, 0);
	quat m = q * qv * conj(q);
	return vec3(m.x, m.y, m.z);
}

}


#endif //  STIR_MATH_COMMON_HPP_
