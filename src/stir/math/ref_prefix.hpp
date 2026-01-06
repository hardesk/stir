#ifndef STIR_MATH_REF_PREFIX_HPP_
#define STIR_MATH_REF_PREFIX_HPP_

#include <memory.h>

namespace stir::ref::impl {

struct U4 { int val[4]; };

struct F4 { float val[4]; };
struct F12 { F4 val[3]; };
struct F16 { F4 val[4]; };

struct D4 { float val[4]; };
struct D16 { F4 val[4]; };

template<unsigned X=0, unsigned Y=1, unsigned Z=2, unsigned W=3>
struct swizz {
	static F4 get4(F4 const& a) { return (F4){a.val[X], a.val[Y], a.val[Z], a.val[W]}; }
	static F4 get3(F4 const& a, float w) { return (F4){a.val[X], a.val[Y], a.val[Z], w}; }
	static F4 get2(F4 const& a, float z, float w) { return (F4){a.val[X], a.val[Y], z, w}; }
	static F4 get1(F4 const& a, float y, float z, float w) { return (F4){a.val[X], y, z, w}; }
};

inline F4 assign_4f_2f(float x, float y) { return F4{x,y,0,0}; }

inline F4 assign_4f_1f(float x) { return F4{x, 0,0,0}; }
inline F4 assign_4f_1f(float x, float w) { return F4{x,0,0,w}; }
inline F4 assign_4f_3f(float x, float y, float z) { return F4{x,y,z,0}; }
inline F4 assign_4f_4f(float x, float y, float z, float w) { return F4{x,y,z,w}; }
inline F4 assign_4f_3f_1f(F4 v, float w) { return F4{v.val[0], v.val[1], v.val[2], w}; }

inline F4 load_3f(float const* p) { return F4{p[0], p[1], p[2], 0}; }
inline void store_3f(float* p, F4 const& v) { p[0] = v.val[0]; p[1] = v.val[1]; p[2] = v.val[2]; }
inline F4 load_4f(float const* p) { return F4{p[0], p[1], p[2], p[3]}; }
inline void store_4f(float* p, F4 const& v) { p[0] = v.val[0]; p[1] = v.val[1]; p[2] = v.val[2]; p[3] = v.val[3]; }
inline float elem_4f(F4 const& a, size_t i) { return a.val[i]; }
inline float elem_12f(F12 const& a, size_t i, size_t j) { return a.val[j].val[i]; }
inline float elem_16f(F16 const& a, size_t i, size_t j) { return a.val[j].val[i]; }
//inline F4 to_vec3(F4 const& a) { return 

} // stir::ref::impl

#endif // STIR_MATH_REF_PREFIX_HPP_
