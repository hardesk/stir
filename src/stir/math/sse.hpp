#ifndef STIR_MATH_SSE_HPP_
#define STIR_MATH_SSE_HPP_

#include "sse_impl.hpp"

namespace stir::sse {

inline bool operator==(vec2 const& a, vec2 const& b) { return impl::eq_2f(a.v, b.v); }
inline bool operator==(vec3 const& a, vec3 const& b) { return impl::eq_3f(a.v, b.v); }
inline bool operator==(vec4 const& a, vec4 const& b) { return impl::eq_4f(a.v, b.v); }
inline bool operator==(quat const& a, quat const& b) { return impl::eq_4f(a.v, b.v); }
inline bool operator==(matrix const& a, matrix const& b) { return impl::eq_16f(a.v, b.v); }

inline vec2 operator+(vec2 const& a, vec2 const& b) { return vec2{impl::addf(a.v, b.v)}; }
inline vec3 operator+(vec3 const& a, vec3 const& b) { return vec3{impl::addf(a.v, b.v)}; }
inline vec4 operator+(vec4 const& a, vec4 const& b) { return vec4{impl::addf(a.v, b.v)}; }
inline quat operator+(quat const& a, quat const& b) { return quat{impl::addf(a.v, b.v)}; }

inline vec2 operator-(vec2 const& a, vec2 const& b) { return vec2{impl::subf(a.v, b.v)}; }
inline vec3 operator-(vec3 const& a, vec3 const& b) { return vec3{impl::subf(a.v, b.v)}; }
inline vec4 operator-(vec4 const& a, vec4 const& b) { return vec4{impl::subf(a.v, b.v)}; }
inline quat operator-(quat const& a, quat const& b) { return quat{impl::subf(a.v, b.v)}; }

inline vec2 operator*(vec2 const& a, vec2 const& b) { return vec2{impl::mulf(a.v, b.v)}; }
inline vec3 operator*(vec3 const& a, vec3 const& b) { return vec3{impl::mulf(a.v, b.v)}; }
inline vec4 operator*(vec4 const& a, vec4 const& b) { return vec4{impl::mulf(a.v, b.v)}; }
inline quat operator*(quat const& a, quat const& b) { return quat{impl::mul_quat(a.v, b.v)}; }
inline vec2 operator*(vec2 const& a, float b) { return vec2{impl::mulf(a.v, b)}; }
inline vec3 operator*(vec3 const& a, float b) { return vec3{impl::mulf(a.v, b)}; }
inline vec4 operator*(vec4 const& a, float b) { return vec4{impl::mulf(a.v, b)}; }
inline quat operator*(quat const& a, float b) { return quat{impl::mulf(a.v, b)}; }

inline float length(vec2 const& a) { return impl::len2(a.v); }
inline float length(vec3 const& a) { return impl::len4(a.v); }
inline float length(vec4 const& a) { return impl::len4(a.v); }
inline float length(quat const& a) { return impl::len4(a.v); }

inline float inv_length(vec2 const& a) { return impl::inv_len4(a.v); }
inline float inv_length(vec4 const& a) { return impl::inv_len4(a.v); }
inline float inv_length(vec3 const& a) { return impl::inv_len4(a.v); }
inline float inv_length(quat const& a) { return impl::inv_len4(a.v); }
// inline float inv_sqlen(vec4 const& a) { return impl::inv_lensq4(a.v); }
// inline float inv_sqlen(quat const& a) { return impl::inv_lensq4(a.v); }

inline float dot(vec2 const& a, vec2 const& b) { return impl::dot4(a.v, b.v); }
inline float dot(vec3 const& a, vec3 const& b) { return impl::dot4(a.v, b.v); }
inline float dot(vec4 const& a, vec4 const& b) { return impl::dot4(a.v, b.v); }
inline vec3 cross(vec3 const& a, vec3 const& b) { return vec3{impl::cross(a.v, b.v)}; }


inline vec2 normalize(vec2 const& a) { return vec2{impl::normalized_4f(a.v)}; }
inline vec3 normalize(vec3 const& a) { return vec3{impl::normalized_4f(a.v)}; }
inline vec4 normalize(vec4 const& a) { return vec4{impl::normalized_4f(a.v)}; }
inline quat normalize(quat const& a) { return quat{impl::normalized_4f(a.v)}; }

inline quat conj(quat const& a) { return quat{impl::conj_quat(a.v)}; }
inline quat inverse(quat const& a) { return quat{impl::inverse_quat(a.v)}; }
// inline quat from_axis_angle(vec3 const& v, float a) { return quat{impl::quat_from_aa(v.v, a)}; }
// inline quat from_axis_angle_norm(vec3 const& v, float a) { return quat{impl::quat_fromaa_normalized0(v.v, a)}; }
// inline vec3 axis(quat const& q) { return vec3 {impl::axis_quat(q.v)}; }
// inline vec3 rot(quat const& q, vec3 const& v) { return vec3{impl::rot_quat_vec3(q.v, v.v)}; }
inline quat mul(quat const& a, quat const& b) { return quat{impl::mul_quat(a.v, b.v)}; }

inline matrix mul(matrix const& a, matrix const& b) { return matrix{ impl::mul(a.v, b.v) }; }
inline matrix operator*(matrix const& a, matrix const& b) { return matrix{ impl::mul(a.v, b.v) }; }
inline matrix transpose(matrix const& a) { return matrix{ impl::transpose(a.v) }; }
// inline matrix load_transpose(float const* a) { return matrix{ impl::load_transpose(a) }; }
// inline void store_transpose(float* p, matrix const& a) { return impl::store_transpose(p, a.v); }
inline vec4 mul(matrix const& m, vec4 const& v) { return vec4{ impl::mul4(m.v, v.v) }; }

} // stir::sse

#endif // STIR_MATH_SSE_HPP_
