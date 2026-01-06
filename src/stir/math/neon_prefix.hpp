#ifndef STIR_MATH_NEON_PREFIX_HPP_
#define STIR_MATH_NEON_PREFIX_HPP_

#include <arm_neon.h>
#include <cstddef>

namespace stir::neon::impl {

typedef uint32x4_t U4;

typedef float32x4_t F4;
typedef float32x4x4_t F16;
typedef float32x4x3_t F12;

typedef float32x4_t D4;
typedef float32x4x4_t D16;

template<unsigned X=0, unsigned Y=1, unsigned Z=2, unsigned W=3>
struct swizz {
    static float32x4_t get4(float32x4_t a) { return (float32x4_t){a[X], a[Y], a[Z], a[W]}; }
    static float32x4_t get3(float32x4_t a, float w) { return (float32x4_t){a[X], a[Y], a[Z], w}; }
    static float32x4_t get2(float32x4_t a, float z, float w) { return (float32x4_t){a[X], a[Y], z, w}; }
    static float32x4_t get1(float32x4_t a, float y, float z, float w) { return (float32x4_t){a[X], y, z, w}; }
};

inline float32x4_t assign_4f_2f(float x, float y);

inline float32x4_t assign_4f_1f(float x);
inline float32x4_t assign_4f_1f(float x, float w);
inline float32x4_t assign_4f_3f(float x, float y, float z);
inline float32x4_t assign_4f_4f(float x, float y, float z, float w);
inline float32x4_t assign_4f_3f_1f(float32x4_t v, float w);

inline float32x4_t load_3f(float const* p);
inline void store_3f(float* p, float32x4_t v);
inline float32x4_t load_4f(float const* p);
inline void store_4f(float* p, float32x4_t v);
inline float elem_4f(float32x4_t a, size_t i);
inline float elem_12f(float32x4x3_t a, size_t i, size_t j);
inline float elem_16f(float32x4x4_t a, size_t i, size_t j);

inline float32x4_t to_vec3(float32x4_t a);
inline bool eq_3f(float32x4_t a, float32x4_t b);
inline bool eq_4f(float32x4_t a, float32x4_t b);
inline bool eq_16f(float32x4x4_t a, float32x4x4_t b);

} // stir::neon::impl

#endif // STIR_MATH_NEON_PREFIX_HPP_
