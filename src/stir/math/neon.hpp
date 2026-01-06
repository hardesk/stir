#ifndef STIR_MATH_NEON_HPP_
#define STIR_MATH_NEON_HPP_

#include <arm_neon.h>
#include <cstring>
#include <cassert>

#include "neon_impl.hpp"

namespace stir::neon {

inline bool operator==(vec2 const& a, vec2 const& b) { return impl::eq_2f(a.v, b.v); }
inline bool operator==(vec3 const& a, vec3 const& b) { return impl::eq_3f(a.v, b.v); }
inline bool operator==(vec4 const& a, vec4 const& b) { return impl::eq_4f(a.v, b.v); }
inline bool operator==(quat const& a, quat const& b) { return impl::eq_4f(a.v, b.v); }
inline bool operator==(matrix const& a, matrix const& b) { return impl::eq_16f(a.v, b.v); }

inline float dot(vec2 const& a, vec2 const& b) { return impl::dot4(a.v, b.v); }
inline float dot(vec3 const& a, vec3 const& b) { return impl::dot4(a.v, b.v); }
inline float dot(vec4 const& a, vec4 const& b) { return impl::dot4(a.v, b.v); }
inline vec3 cross(vec3 const& a, vec3 const& b) { return vec3{impl::cross(a.v, b.v)}; }

inline vec2 operator+(vec2 const& a, vec2 const& b) { return vec2{impl::addf(a.v, b.v)}; }
inline vec2 operator-(vec2 const& a, vec2 const& b) { return vec2{impl::subf(a.v, b.v)}; }
inline vec2 operator*(vec2 const& a, vec2 const& b) { return vec2{impl::mulf(a.v, b.v)}; }
inline vec2 operator*(vec2 const& a, float b) { return vec2{impl::mulf(a.v, b)}; }
inline float len(vec2 const& a) { return impl::len4(a.v); }
inline float inv_len(vec2 const& a) { return impl::inv_len4(a.v); }
inline vec2 normalize(vec2 const& a) { return vec2{impl::normalized_4f(a.v)}; }

inline vec3 operator+(vec3 const& a, vec3 const& b) { return vec3{impl::addf(a.v, b.v)}; }
inline vec3 operator-(vec3 const& a, vec3 const& b) { return vec3{impl::subf(a.v, b.v)}; }
inline vec3 operator*(vec3 const& a, vec3 const& b) { return vec3{impl::mulf(a.v, b.v)}; }
inline vec3 operator*(vec3 const& a, float b) { return vec3{impl::mulf(a.v, b)}; }
inline float len(vec3 const& a) { return impl::len4(a.v); }
inline float inv_len(vec3 const& a) { return impl::inv_len4(a.v); }
inline vec3 normalize(vec3 const& a) { return vec3{impl::normalized_4f(a.v)}; }

inline vec4 operator+(vec4 const& a, vec4 const& b) { return vec4{impl::addf(a.v, b.v)}; }
inline vec4 operator-(vec4 const& a, vec4 const& b) { return vec4{impl::subf(a.v, b.v)}; }
inline vec4 operator*(vec4 const& a, vec4 const& b) { return vec4{impl::mulf(a.v, b.v)}; }
inline vec4 operator*(vec4 const& a, float b) { return vec4{impl::mulf(a.v, b)}; }
inline float len(vec4 const& a) { return impl::len4(a.v); }
inline vec4 normalize(vec4 const& a) { return vec4{impl::normalized_4f(a.v)}; }
inline float inv_len(vec4 const& a) { return impl::inv_len4(a.v); }
inline float inv_sqlen(vec4 const& a) { return impl::inv_lensq4(a.v); }

inline quat operator+(quat const& a, quat const& b) { return quat{impl::addf(a.v, b.v)}; }
inline quat operator-(quat const& a, quat const& b) { return quat{impl::subf(a.v, b.v)}; }
inline quat operator*(quat const& a, float b) { return quat{impl::mulf(a.v, b)}; }
inline float len(quat const& a) { return impl::len4(a.v); }
inline float inv_len(quat const& a) { return impl::inv_len4(a.v); }
inline float inv_sqlen(quat const& a) { return impl::inv_lensq4(a.v); }
inline quat conj(quat const& a) { return quat{impl::conj_quat(a.v)}; }
inline quat inverse(quat const& a) { return quat{impl::inverse_quat(a.v)}; }
inline quat mul(quat const& a, quat const& b) { return quat{impl::mul_quat(a.v, b.v)}; }
inline quat operator*(quat const& a, quat const& b) { return quat{impl::mul_quat(a.v, b.v)}; }
inline quat normalize(quat const& a) { return quat{impl::normalized_4f(a.v)}; }
inline quat from_axis_angle(vec3 const& v, float a) { return quat{impl::quat_from_aa(v.v, a)}; }
inline quat from_axis_angle_norm(vec3 const& v, float a) { return quat{impl::quat_fromaa_normalized0(v.v, a)}; }
inline vec3 axis(quat const& q) { return vec3 {impl::axis_quat(q.v)}; }
inline vec3 rot(vec3 const& v, quat const& q) { return vec3{impl::rot_vec3_quat(v.v, q.v)}; }

inline matrix mul(matrix const& a, matrix const& b) { return matrix{ impl::mul(a.v, b.v) }; }
inline matrix operator*(matrix const& a, matrix const& b) { return matrix{ impl::mul(a.v, b.v) }; }
inline matrix transpose(matrix const& a) { return matrix{ impl::transpose(a.v) }; }
inline matrix load_transpose(float const* a) { return matrix{ impl::load_transpose(a) }; }
inline void store_transpose(float* p, matrix const& a) { return impl::store_transpose(p, a.v); }
inline vec4 mul(matrix const& m, vec4 const& v) { return vec4{ impl::mul4(m.v, v.v) }; }

inline vec3 xform_pos(matrix const& m, vec3 const& v) { return vec3{ impl::to_vec3( impl::mul4(m.v, impl::assign_4f_3f_1f(v.v,1.0f)) ) }; }
inline vec4 xform_pos4(matrix const& m, vec3 const& v) { return vec4{ impl::mul4(m.v, impl::assign_4f_3f_1f(v.v,1.0f)) }; }
inline vec3 xform_dir(matrix const& m, vec3 const& v) { return vec3{ impl::mul3(m.v, v.v) }; }

inline matrix33 mul(matrix33 const& a, matrix33 const& b) { return matrix33{ impl::mul33(a.v, b.v) }; }
//inline matrix33 inverse(matrix33 const& a) { return matrix33{ impl::inverse33(a.v) }; }

inline matrix inverse_tr(matrix const& a) { return matrix{ impl::inverse_tr(a.v) }; }
inline matrix inverse_trs(matrix const& a) { return matrix{ impl::inverse_trs(a.v) }; }
//inline matrix inverse_full(matrix a) { return matrix{ simd::inverse_trs(a.v) }; }

} // stir::neon

#endif // STIR_MATH_NEON_HPP_
