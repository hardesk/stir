#ifndef STIR_MATH_SSE_PREFIX_HPP_
#define STIR_MATH_SSE_PREFIX_HPP_

#include <xmmintrin.h> // sse
#include <emmintrin.h> // sse2
#include <pmmintrin.h> // sse3
#include <smmintrin.h> // sse4.1
#include <nmmintrin.h> // sse4.2
#include <cstddef>

namespace stir::sse::impl {

#if defined(_MSC_VER) && defined(_M_X64)
#define STIR_SSE_CALL __vectorcall
#else
#define STIR_SSE_CALL 
#endif

typedef __m128 F4;
struct F16  { F4 val[4]; };
struct F12  { F4 val[3]; };

typedef __m128 D4;
struct D16  { D4 val[4]; };

template<unsigned X=0, unsigned Y=1, unsigned Z=2, unsigned W=3>
struct swizz {
    static __m128 STIR_SSE_CALL get4(__m128 a) { return _mm_shuffle_ps(a, a, _MM_SHUFFLE(W, Z, Y, X)); }
    static __m128 STIR_SSE_CALL get3(__m128 a, float w) {
        __m128 r = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, Z, Y, X));
        r = _mm_insert_ps(r, _mm_set_ss(w), 0x30);
        return r;
    }
    static __m128 STIR_SSE_CALL get2(__m128 a, float z, float w) {
        __m128 r = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 2, Y, X));
        r = _mm_insert_ps(r, _mm_set_ss(w), 0x30);
        r = _mm_insert_ps(r, _mm_set_ss(z), 0x20);
        return r;
    }
    static __m128 STIR_SSE_CALL get1(__m128 a, float y, float z, float w) {
        __m128 r = _mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 2, 1, X));
        r = _mm_insert_ps(r, _mm_set_ss(w), 0x30);
        r = _mm_insert_ps(r, _mm_set_ss(z), 0x20);
        r = _mm_insert_ps(r, _mm_set_ss(y), 0x10);
        return r;
    }
};

inline F4 STIR_SSE_CALL assign_4f_2f(float x, float y) { return _mm_set_ps(0.0f, 0.0f, y, x); }

inline F4 STIR_SSE_CALL assign_4f_1f(float x) { return _mm_set1_ps(x); }
inline F4 STIR_SSE_CALL assign_4f_1f(float x, float w) { return _mm_set_ps(w, 0.0f, 0.0f, x); }
inline F4 STIR_SSE_CALL assign_4f_3f(float x, float y, float z) { return _mm_set_ps(0.0f, z, y, x); }
inline F4 STIR_SSE_CALL assign_4f_4f(float x, float y, float z, float w) { return _mm_set_ps(w, z, y, x); }
inline F4 STIR_SSE_CALL assign_4f_3f_1f(F4 v, float w) { return _mm_insert_ps(v, _mm_set_ss(w), 0x30); }

inline F4 STIR_SSE_CALL load_3f(float const* p) { 
    __m128 xy = _mm_castpd_ps(_mm_load_sd((double const*)(void const*)p));
    __m128 z = _mm_load_ss(p+2);
    __m128 r = _mm_insert_ps(xy, z, 0x24); // 0x24 -> move z[0] into xy[2] and zero out xy['0b100']
    return r;
}
inline void STIR_SSE_CALL store_3f(float* p, F4 v) { 
    _mm_storel_pi((__m64*)p, v); // store x,y
    _mm_store_ss(p + 2, _mm_shuffle_ps(v, v,_MM_SHUFFLE(2, 2, 2, 2))); // store z
}
inline F4 STIR_SSE_CALL load_4f(float const* p) { return _mm_loadu_ps(p); }
inline void STIR_SSE_CALL store_4f(float* p, F4 v) { _mm_storeu_ps(p, v); }
inline float STIR_SSE_CALL elem_4f(F4 a, size_t i) { 
    switch(i) {
        default:
        case 0: return _mm_cvtss_f32(a);
        case 1: return _mm_cvtss_f32(_mm_shuffle_ps(a, a, _MM_SHUFFLE(3,2,1,1)));
        case 2: return _mm_cvtss_f32(_mm_shuffle_ps(a, a, _MM_SHUFFLE(3,2,2,2)));
        case 3: return _mm_cvtss_f32(_mm_shuffle_ps(a, a, _MM_SHUFFLE(3,2,2,3)));
    }
}
inline float STIR_SSE_CALL elem_12f(F12 a, size_t i, size_t j) { return elem_4f(a.val[j], i); }
inline float STIR_SSE_CALL elem_16f(F16 a, size_t i, size_t j) { return elem_4f(a.val[j], i); }

inline F4 STIR_SSE_CALL to_vec3(F4 a) { return _mm_insert_ps(a, _mm_set_ss(0.0f), 0x30); }
inline bool STIR_SSE_CALL eq_2f(F4 a, F4 b) {
    __m128 cmp = _mm_cmpeq_ps(a, b);
    return (_mm_movemask_ps(cmp) & 0x3) == 0x3;
}
inline bool STIR_SSE_CALL eq_3f(F4 a, F4 b) {
    __m128 cmp = _mm_cmpeq_ps(a, b);
    return (_mm_movemask_ps(cmp) & 0x7) == 0x7;
}
inline bool STIR_SSE_CALL eq_4f(F4 a, F4 b) {
    __m128 cmp = _mm_cmpeq_ps(a, b);
    return (_mm_movemask_ps(cmp) & 0xF) == 0xF;
}
inline bool STIR_SSE_CALL eq_16f(F16 a, F16 b) {
    __m128 cmp0 = _mm_cmpeq_ps(a.val[0], b.val[0]);
    __m128 cmp1 = _mm_cmpeq_ps(a.val[1], b.val[1]);
    __m128 cmp2 = _mm_cmpeq_ps(a.val[2], b.val[2]);
    __m128 cmp3 = _mm_cmpeq_ps(a.val[3], b.val[3]);
    return (_mm_movemask_ps(cmp0) & 0xF) == 0xF &&
           (_mm_movemask_ps(cmp1) & 0xF) == 0xF &&
           (_mm_movemask_ps(cmp2) & 0xF) == 0xF &&
           (_mm_movemask_ps(cmp3) & 0xF) == 0xF;
}

} // stir::sse::impl

#endif // STIR_MATH_SSE_PREFIX_HPP_
