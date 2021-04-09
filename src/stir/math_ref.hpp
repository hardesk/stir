#ifndef STIR_MATH_REF_HPP_
#define STIR_MATH_REF_HPP_

#include <cmath>
#include "math_def.hpp"

namespace stir
{

#if STIR_MATH_MODE == STIR_MATH_REF
inline
#endif
namespace ref
{

inline bool operator==(vec2 const& a, vec2 const& b) { return a.x==b.x && a.y==b.y; }
inline bool operator==(vec3 const& a, vec3 const& b) { return a.x==b.x && a.y==b.y && a.z==b.z; }
inline bool operator==(vec4 const& a, vec4 const& b) { return a.x==b.x && a.y==b.y && a.z==b.z && a.w==b.w; }
inline bool operator==(quat const& a, quat const& b) { return a.x==b.x && a.y==b.y && a.z==b.z && a.w==b.w; }

inline float length(vec2 const& a) { return sqrt(a.x*a.x+a.y*a.y); }
inline float length(vec3 const& a) { return sqrt(a.x*a.x+a.y*a.y+a.z*a.z); }
inline float length(vec4 const& a) { return sqrt(a.x*a.x+a.y*a.y+a.z*a.z+a.w*a.w); }
inline float length(quat const& a) { return sqrt(a.x*a.x+a.y*a.y+a.z*a.z+a.w*a.w); }

inline vec2 normalize(vec2 const& a) { float r=1.0f/ref::length(a); return vec2(a.x*r, a.y*r); }
inline vec3 normalize(vec3 const& a) { float r=1.0f/ref::length(a); return vec3(a.x*r, a.y*r, a.z*r); }
inline vec4 normalize(vec4 const& a) { float r=1.0f/ref::length(a); return vec4(a.x*r, a.y*r, a.z*r, a.w*r); }
inline quat normalize(quat const& a) { float r=1.0f/ref::length(a); return quat(a.x*r, a.y*r, a.z*r, a.w*r); }

inline vec2 operator+(vec2 const& a, vec2 const& b) { return vec2(a.x+b.x, a.y+b.y); }
inline vec3 operator+(vec3 const& a, vec3 const& b) { return vec3(a.x+b.x, a.y+b.y, a.z+b.z); }
inline vec4 operator+(vec4 const& a, vec4 const& b) { return vec4(a.x+b.x, a.y+b.y, a.z+b.z, a.w+b.w); }
inline quat operator+(quat const& a, quat const& b) { return quat(a.x+b.x, a.y+b.y, a.z+b.z, a.w+b.w); }
inline vec2 operator+(vec2 const& a, float b) { return vec2(a.x+b, a.y+b); }
inline vec3 operator+(vec3 const& a, float b) { return vec3(a.x+b, a.y+b, a.z+b); }
inline vec4 operator+(vec4 const& a, float b) { return vec4(a.x+b, a.y+b, a.z+b, a.w+b); }
inline quat operator+(quat const& a, float b) { return quat(a.x+b, a.y+b, a.z+b, a.w+b); }

inline vec2 operator-(vec2 const& a, vec2 const& b) { return vec2(a.x-b.x, a.y-b.y); }
inline vec3 operator-(vec3 const& a, vec3 const& b) { return vec3(a.x-b.x, a.y-b.y, a.z-b.z); }
inline vec4 operator-(vec4 const& a, vec4 const& b) { return vec4(a.x-b.x, a.y-b.y, a.z-b.z, a.w-b.w); }
inline quat operator-(quat const& a, quat const& b) { return quat(a.x-b.x, a.y-b.y, a.z-b.z, a.w-b.w); }
inline vec2 operator-(vec2 const& a, float b) { return vec2(a.x-b, a.y-b); }
inline vec3 operator-(vec3 const& a, float b) { return vec3(a.x-b, a.y-b, a.z-b); }
inline vec4 operator-(vec4 const& a, float b) { return vec4(a.x-b, a.y-b, a.z-b, a.w-b); }
inline quat operator-(quat const& a, float b) { return quat(a.x-b, a.y-b, a.z-b, a.w-b); }

inline vec2 operator*(vec2 const& a, float b) { return vec2(a.x*b, a.y*b); }
inline vec3 operator*(vec3 const& a, float b) { return vec3(a.x*b, a.y*b, a.z*b); }
inline vec4 operator*(vec4 const& a, float b) { return vec4(a.x*b, a.y*b, a.z*b, a.w*b); }
inline quat operator*(quat const& a, float b) { return quat(a.x*b, a.y*b, a.z*b, a.w*b); }

inline vec2 operator/(vec2 const& a, float b) { return vec2(a.x/b, a.y/b); }
inline vec3 operator/(vec3 const& a, float b) { return vec3(a.x/b, a.y/b, a.z/b); }
inline vec4 operator/(vec4 const& a, float b) { return vec4(a.x/b, a.y/b, a.z/b, a.w/b); }
inline quat operator/(quat const& a, float b) { return quat(a.x/b, a.y/b, a.z/b, a.w/b); }

inline quat conj(quat const& q)
{
	return quat(q.x, q.y, q.z, -q.w);
}

inline quat from_axis_angle(vec3 const& v, float a)
{
	float a2 = a/2.0f;
	float sa = sin(a2), ca = cos(a2);
	return quat (v.x*sa, v.y*sa, v.z*sa, ca);
}

inline float dot(quat const& a, quat const& b) { return a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w; }
quat exp(quat const& a);
quat log(quat const& a);
quat mul(quat const& a, quat const& b);
//vec3 rot(vec3 const& v, quat const& q);

matrix mul(matrix const& a, matrix const& b);
vec4 mul(vec4 const& v, matrix const& a);

vec4 mul(vec3 const& v, matrix const& a);
vec3 mul3(vec3 const& v, matrix const& a);

matrix transpose(matrix const& m);
matrix rot_matrix(quat const& q);
matrix transform_inverse(matrix const& m);
matrix full_inverse(matrix const& m);

} // namespace ref

#if STIR_MATH_MODE == STIR_MATH_REF
inline void vec2::normalize() { float r=1.0f/length(*this); x*=r; y*=r; }
inline void vec3::normalize() { float r=1.0f/length(*this); x*=r; y*=r; z*=r; }
inline void vec4::normalize() { float r=1.0f/length(*this); x*=r; y*=r; z*=r; w*=r; }
inline void quat::normalize() { float r=1.0f/length(*this); x*=r; y*=r; z*=r; w*=r; }
#endif

}

#endif
