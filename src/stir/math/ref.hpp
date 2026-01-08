#ifndef STIR_MATH_REF_HPP_
#define STIR_MATH_REF_HPP_

#include <cmath>
#include "math.hpp"

namespace stir::ref
{

// float32x4x4_t inverse_trs(float32x4x4_t a);
inline bool operator==(vec2 const& a, vec2 const& b) { return a[0]==b[0] && a[1]==b[1]; }
inline bool operator==(vec3 const& a, vec3 const& b) { return a[0]==b[0] && a[1]==b[1] && a[2]==b[2]; }
inline bool operator==(vec4 const& a, vec4 const& b) { return a[0]==b[0] && a[1]==b[1] && a[2]==b[2] && a[3]==b[3]; }
inline bool operator==(quat const& a, quat const& b) { return a[0]==b[0] && a[1]==b[1] && a[2]==b[2] && a[3]==b[3]; }
bool operator==(matrix const& a, matrix const& b);

inline float length(vec2 const& a) { return sqrt(a[0]*a[0] + a[1]*a[1]); }
inline float length(vec3 const& a) { return sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]); }
inline float length(vec4 const& a) { return sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2] + a[3]*a[3]); }
inline float length(quat const& a) { return sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2] + a[3]*a[3]); }

inline vec2 normalize(vec2 const& a) { float r=1.0f/ref::length(a); return vec2(a[0]*r, a[1]*r); }
inline vec3 normalize(vec3 const& a) { float r=1.0f/ref::length(a); return vec3(a[0]*r, a[1]*r, a[2]*r); }
inline vec4 normalize(vec4 const& a) { float r=1.0f/ref::length(a); return vec4(a[0]*r, a[1]*r, a[2]*r, a[3]*r); }
inline quat normalize(quat const& a) { float r=1.0f/ref::length(a); return quat(a[0]*r, a[1]*r, a[2]*r, a[3]*r); }

inline vec2 operator+(vec2 const& a, vec2 const& b) { return vec2(a[0]+b[0], a[1]+b[1]); }
inline vec3 operator+(vec3 const& a, vec3 const& b) { return vec3(a[0]+b[0], a[1]+b[1], a[2]+b[2]); }
inline vec4 operator+(vec4 const& a, vec4 const& b) { return vec4(a[0]+b[0], a[1]+b[1], a[2]+b[2], a[3]+b[3]); }
inline quat operator+(quat const& a, quat const& b) { return quat(a[0]+b[0], a[1]+b[1], a[2]+b[2], a[3]+b[3]); }
inline vec2 operator+(vec2 const& a, float b) { return vec2(a[0]+b, a[1]+b); }
inline vec3 operator+(vec3 const& a, float b) { return vec3(a[0]+b, a[1]+b, a[2]+b); }
inline vec4 operator+(vec4 const& a, float b) { return vec4(a[0]+b, a[1]+b, a[2]+b, a[3]+b); }
inline quat operator+(quat const& a, float b) { return quat(a[0]+b, a[1]+b, a[2]+b, a[3]+b); }

inline vec2 operator-(vec2 const& a, vec2 const& b) { return vec2(a[0]-b[0], a[1]-b[1]); }
inline vec3 operator-(vec3 const& a, vec3 const& b) { return vec3(a[0]-b[0], a[1]-b[1], a[2]-b[2]); }
inline vec4 operator-(vec4 const& a, vec4 const& b) { return vec4(a[0]-b[0], a[1]-b[1], a[2]-b[2], a[3]-b[3]); }
inline quat operator-(quat const& a, quat const& b) { return quat(a[0]-b[0], a[1]-b[1], a[2]-b[2], a[3]-b[3]); }
inline vec2 operator-(vec2 const& a, float b) { return vec2(a[0]-b, a[1]-b); }
inline vec3 operator-(vec3 const& a, float b) { return vec3(a[0]-b, a[1]-b, a[2]-b); }
inline vec4 operator-(vec4 const& a, float b) { return vec4(a[0]-b, a[1]-b, a[2]-b, a[3]-b); }
inline quat operator-(quat const& a, float b) { return quat(a[0]-b, a[1]-b, a[2]-b, a[3]-b); }

inline vec2 operator*(vec2 const& a, float b) { return vec2(a[0]*b, a[1]*b); }
inline vec3 operator*(vec3 const& a, float b) { return vec3(a[0]*b, a[1]*b, a[2]*b); }
inline vec4 operator*(vec4 const& a, float b) { return vec4(a[0]*b, a[1]*b, a[2]*b, a[3]*b); }
inline quat operator*(quat const& a, float b) { return quat(a[0]*b, a[1]*b, a[2]*b, a[3]*b); }

inline vec2 operator/(vec2 const& a, float b) { return vec2(a[0]/b, a[1]/b); }
inline vec3 operator/(vec3 const& a, float b) { return vec3(a[0]/b, a[1]/b, a[2]/b); }
inline vec4 operator/(vec4 const& a, float b) { return vec4(a[0]/b, a[1]/b, a[2]/b, a[3]/b); }
inline quat operator/(quat const& a, float b) { return quat(a[0]/b, a[1]/b, a[2]/b, a[3]/b); }

inline vec2 pp(vec2 const& a, cw) { return vec2(a[1], -a[0]); }
inline vec2 pp(vec2 const& a, ccw) { return vec2(-a[1], a[0]); }
inline vec3 cross(vec3 const& a, vec3 const& b) { return vec3(a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]); }

inline float dot(vec3 const& a, vec3 const& b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
inline float dot(vec4 const& a, vec4 const& b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]; }

inline quat inverse(quat const& q) {
	float n2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
	return quat(-q[0]/n2, -q[1]/n2, -q[2]/n2, q[3]/n2);
}
inline quat conj(quat const& q) { return quat(-q[0], -q[1], -q[2], q[3]); }

inline quat from_axis_angle(vec3 const& v, float a) {
	float a2 = a/2.0f;
	float sa = sin(a2), ca = cos(a2);
	vec3 n = normalize(v);
	return quat (n[0]*sa, n[1]*sa, n[2]*sa, ca);
}
inline vec3 axis(quat const& q) {
	float s = sqrt(1.0f - q[3]*q[3]);
	if (s < 1e-6f) return vec3(1.0f, 0.0f, 0.0f);
	return vec3(q[0]/s, q[1]/s, q[2]/s);
}

inline float dot(quat const& a, quat const& b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]; }
quat exp(quat const& a);
quat log(quat const& a);
inline vec2 mul(vec2 const& a, vec2 const& b) { return vec2{a[0]*b[0], a[1]*b[1] }; }
inline vec3 mul(vec3 const& a, vec3 const& b) { return vec3{a[0]*b[0], a[1]*b[1], a[2]*b[2] }; }
inline vec4 mul(vec4 const& a, vec4 const& b) { return vec4{a[0]*b[0], a[1]*b[1], a[2]*b[2], a[3]*b[3] }; }
quat mul(quat const& a, quat const& b);
vec4 mul(matrix const& a, vec4 const& v);

vec3 xform_pos(matrix const& m, vec3 const& v);
vec3 xform_dir(matrix const& m, vec3 const& v);

inline vec3 rot(quat const& q, vec3 const& v) {
	quat qv(v[0], v[1], v[2], 0);
	quat m = ref::mul(ref::mul(q,qv),ref::conj(q));
	return vec3(m[0], m[1], m[2]);
}

matrix mul(matrix const& a, matrix const& b);
matrix transpose(matrix const& m);
matrix rot_matrix(quat const& q);
matrix inverse_cofactor(matrix const& m);
matrix inverse(matrix const& m);
//matrix inverse_3x3(matrix const& m);
//matrix inverse_ortho(matrix const& m);

} // namespace ref

#endif
