#ifndef STIR_MATH_HPP_
#define STIR_MATH_HPP_

#include <cmath>
#include <cstring>
#include <type_traits>
#include <initializer_list>


// define types
#include "math_def.hpp"

// define "implementation" functions
#if STIR_MATH_MODE == STIR_MATH_MIPP
#include "math_mipp.hpp"
#elif STIR_MATH_MODE == STIR_MATH_XSIMD
#include "math_xsimd.hpp"
#else
// in case ref is backend is selected, include it here so 'common' sees the impl functions
#include "math_ref.hpp"
#endif

// define common functions that are implemented in terms of "implementation"
#include "math_common.hpp"

// always (unless specifically disabled) include the reference implementation
#include "math_ref.hpp"

// Vector is interpreted as 1x4 (a row) when treated as a matrix.
// Thus vector/matrix multiplication happens as v * M

namespace stir
{


}

#if 0

namespace stir
{

struct alignas(16) Matrix1
{
	/*template<typename... T>
	Matrix1(T... args)
		:	f{args...}
	{}*/
	float f[16];

	void set(int i, float a, float b, float c, float d) { f[i*4+0] = a; f[i*4+1] = b; f[i*4+2] = c; f[i*4+3] = d; }
	float operator()(int i, int j) const { return f[i*4+j]; }

	Matrix1 const& operator+=(Matrix1 const& a)
	{
		for (int i=0; i<16; ++i)
			f[i] += a.f[i];
		return *this;
	}
};

inline Vector4 mul_raw(Vector4 const& v, Matrix const& a)
{
	BF4 vv(v.simd_data());
	BF4 v0 = BF4(vv[0]);
	BF4 v1 = BF4(vv[1]);
	BF4 v2 = BF4(vv[2]);
	BF4 v3 = BF4(vv[3]);

	BF4 a0, a1, a2, a3;
	a0.load_aligned(a.el[0]);
	a1.load_aligned(a.el[0]);
	a2.load_aligned(a.el[0]);
	a3.load_aligned(a.el[0]);

	return Vector4(v0*a0 + v1*a1 + v2*a2 + v3*a3);
}

inline Vector4 mul(Vector4 const& v, Matrix const& a)
{
	BF4 vv(v.simd_data());
	BF4 v0 = BF4(vv[0]);
	BF4 v1 = BF4(vv[1]);
	BF4 v2 = BF4(vv[2]);
	BF4 v3 = BF4(vv[3]);

	return Vector4(v0*a.row[0] + v1*a.row[1] + v2*a.row[2] + v3*a.row[3]);
}

inline Vector4 mul(Vector3 const& v, Matrix const& a)
{
#if 1
	BF4 v0 = BF4(v.x);
	BF4 v1 = BF4(v.y);
	BF4 v2 = BF4(v.z);
#else
	BF4 vv(v.as_vec4().simd_data());
	BF4 v0 = BF4(vv[0]);
	BF4 v1 = BF4(vv[1]);
	BF4 v2 = BF4(vv[2]);
#endif

	return Vector4(v0*a.row[0] + v1*a.row[1] + v2*a.row[2] + a.row[3]);
}

inline Vector3 mul3(Vector3 const& v, Matrix const& a)
{
	BF4 v0 = BF4(v.x);
	BF4 v1 = BF4(v.y);
	BF4 v2 = BF4(v.z);

	BF4 r = v0*a.row[0] + v1*a.row[1] + v2*a.row[2];

	return Vector3(r[0], r[1], r[2]);
}

inline Matrix mul_sse(Matrix const& a, Matrix const& b)
{
	Matrix r;

	__m128i b0 = _mm_load_ps(b.el[0]);
	__m128i b1 = _mm_load_ps(b.el[1]);
	__m128i b2 = _mm_load_ps(b.el[2]);
	__m128i b3 = _mm_load_ps(b.el[3]);

	for (int i=0; i<4; ++i)
	{
		__m128i aa = _mm_load_ps(a.el[i]);
		__m128i a0 = _mm_shuffle_ps(aa, aa, 0x00);
		__m128i a1 = _mm_shuffle_ps(aa, aa, 0x55);
		__m128i a2 = _mm_shuffle_ps(aa, aa, 0xaa);
		__m128i a3 = _mm_shuffle_ps(aa, aa, 0xff);

		__m128i rr = _mm_add_ps(
				_mm_add_ps(_mm_mul_ps(a0, b0), _mm_mul_ps(a1, b1)),
				_mm_add_ps(_mm_mul_ps(a2, b2), _mm_mul_ps(a3, b3))); 
		_mm_store_ps(r.el[i], rr);
	}

	return r;
}

inline Matrix mul_xsimd_old(Matrix const& a, Matrix const& b)
{
	Matrix r;
	
	using R = xsimd::batch<float, 4>; 

	R b0(b.el[0]), b1(b.el[1]), b2(b.el[2]), b3(b.el[3]);

	for (int i=0; i<4; ++i)
	{
		R aa(a.el[i]);
		R a0 = R(aa[0]);
		R a1 = R(aa[1]);
		R a2 = R(aa[2]);
		R a3 = R(aa[3]);

		R row = a0*b0 + a1*b1 + a2*b2 + a3*b3;
		row.store_aligned(r.el[i]);
	}

	return r;
}

template<class Tag>
void mul_xsimd_ptr(float const* a, float const* b, float* dest)
{
	using R = xsimd::batch<float, 4>; 

	R b0, b1, b2, b3;
	xsimd::load_simd(b+ 0, b0, Tag());
	xsimd::load_simd(b+ 4, b1, Tag());
	xsimd::load_simd(b+ 8, b2, Tag());
	xsimd::load_simd(b+12, b3, Tag());

	for (int i=0; i<4; ++i)
	{
		R aa;
		xsimd::load_simd(a + i*4, aa, Tag());
		R a0 = R(aa[0]);
		R a1 = R(aa[1]);
		R a2 = R(aa[2]);
		R a3 = R(aa[3]);

		R row = a0*b0 + a1*b1 + a2*b2 + a3*b3;
		xsimd::store_simd(dest + i*4, row, Tag());
	}

	//dest[0] = 0xdeadbabe;
}

inline Matrix mul_xsimd(Matrix const& a, Matrix const& b)
{
	Matrix m;
	mul_xsimd_ptr<xsimd::aligned_mode>(a.el[0], b.el[0], m.el[0]);
	return m;
}


inline Matrix mul(Matrix const& a, Matrix const& b)
{
	return mul_xsimd(a, b);
}

inline xsimd::batch<float, 4> dos(xsimd::batch<float, 4> a)
{
	return xsimd::batch<float, 4>(a[2], a[1], a[1], a[3]);
}

}


#endif // 0


#undef Ax
#undef Bx
#undef Xx
#undef Tx

#endif
