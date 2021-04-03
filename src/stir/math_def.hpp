#ifndef STIR_MATH_DEF_HPP_
#define STIR_MATH_DEF_HPP_

#include <cstring>
#include <type_traits>
#include <initializer_list>

#include "math_config.hpp"

#if STIR_MATH_MODE == STIR_MATH_MIPP
#include <mipp.h>
namespace stir {
	using S = mipp::Reg<float>;
	constexpr int Sn = mipp::N<float>();
	using M = mipp::Msk<Sn>;
}
#elif STIR_MATH_MODE == STIR_MATH_XSIMD
#include "math_xsimd.hpp"
namespace stir {
	constexpr int Sn = 4;
	using S = xsimd::batch<float, Sn>;
	//using M = xsimd::batch_bool<Sn>;
}
#else
namespace stir {
	using S = float*;
}
#endif


namespace stir
{

#define STIR_USE_RSQRT 0 // need at least another step https://stackoverflow.com/questions/31555260/fast-vectorized-rsqrt-and-reciprocal-with-sse-avx-depending-on-precision/31559382#31559382 / https://stackoverflow.com/questions/1528727/why-is-sse-scalar-sqrtx-slower-than-rsqrtx-x
#define STIR_USE_RSQRT_2STEP 1

struct vec2;
struct vec3;
struct vec4;
struct quat;
struct matrix;
template<class T> T pi = T(3.1415926535897932385);

#define Ax (&a.x)
#define Bx (&b.x)
#define Tx (&this->x)
#define Xx(a) (&a.x)

namespace impl
{
inline S n4(S s);
inline float l4(S a);
inline float sql4(S a);
}

struct vec2
{
	vec2() {}
	vec2(S s);
	vec2(float x, float y) : x(x), y(y) {}

	vec3 as_vec3(float z = 0);
	vec4 as_vec4(float z = 0, float w = 1.0f);

	void normalize();
	float& operator[](int i) { return Tx[i]; }

	float x, y;
};
static_assert(std::is_standard_layout<vec2>::value == true);

struct vec3
{
	vec3() {}
	vec3(S s);
	vec3(float ax, float ay, float az) : x(ax), y(ay), z(az) {}

	vec4 as_vec4(float w = 1.0f) const;
	//void assign3(S a) { x = a[0]; y = a[1]; z = a[2]; }

	float operator[](int i) const { return Tx[i]; }
	float& operator[](int i) { return Tx[i]; }
	
	//bool operator==(vec3 const& a) const;

	void normalize();

	float x, y, z;
};
static_assert(std::is_standard_layout<vec3>::value == true);

struct vec4
{
	vec4() {}
	vec4(S a);
	vec4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}

	//bool operator==(vec4 const& a) const;
	//float length() const { return impl::l4(S(A)); }
	//float sq_length() const { return impl::sql4(S(A)); }

	void normalize();
	//vec4 normalized() const;

	float operator[](int i) const { return Tx[i]; }
	float& operator[](int i) { return Tx[i]; }

	float x, y, z, w;
};
static_assert(std::is_standard_layout<vec4>::value == true);

struct quat // w is the real component
{
	quat() {}
	quat(S a);
	quat(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}

	//quat conjugate() const;
	//matrix rot_matrix() const;
	static quat identity() { return quat(0,0,0,1.0f); }

	float operator[](int i) const { return Tx[i]; }
	float& operator[](int i) { return Tx[i]; }

	void normalize();
	//quat normalized() const;

	//float length() const { return impl::l4(S(A)); }
	//float sq_length() const { return impl::sql4(S(A)); }

	//S as_simd() const { return S(A); }

	float x, y, z, w;
	
	//static quat from_axis_angle(vec3 const& v, float a);
};
static_assert(std::is_standard_layout<quat>::value == true);

#define R(r) (x+r*4)
struct matrix
{
	static constexpr size_t N = 16;
	float x[N];

	constexpr matrix() {}
	constexpr matrix( float a0, float a1, float a2, float a3,
			float b0, float b1, float b2, float b3,
			float c0, float c1, float c2, float c3,
			float d0, float d1, float d2, float d3)
	{
		x[ 0]=a0; x[ 1]=a1; x[ 2]=a2; x[ 3]=a3;
		x[ 4]=b0; x[ 5]=b1; x[ 6]=b2; x[ 7]=b3;
		x[ 8]=c0; x[ 9]=c1; x[10]=c2; x[11]=c3;
		x[12]=d0; x[13]=d1; x[14]=d2; x[15]=d3;
	}
	/*matrix(S x0, S x1, S x2, S x3)
	{
		x0.store(x+0);
		x1.store(x+4);
		x2.store(x+8);
		x3.store(x+12);
	}*/
	matrix(std::initializer_list<float> l)
	{
		memcpy(x, l.begin(), sizeof(float)*(l.end()-l.begin()));
	}
	
	void set(int row_, float a, float b, float c, float d) { float*p=row(row_); p[0]=a; p[1]=b; p[2]=c; p[3]=d; }

	float* row(size_t i) { return x + 4*i; }
	float const* row(size_t i) const { return x + 4*i; }

	float operator()(int i, int j) const { return x[4*i+j]; }

	matrix const& operator+=(matrix const& a);
	matrix const& operator-=(matrix const& a);
	matrix const& operator*=(matrix const& a);
	matrix const& operator/=(matrix const& a);

	matrix const& operator+=(float k);
	matrix const& operator-=(float k);
	matrix const& operator*=(float k);
	matrix const& operator/=(float k);

//	matrix operator+(float k);
//	matrix operator-(float k);
//	matrix operator*(float k);
//	matrix operator/(float k);

	constexpr static matrix identity()
	{
		return matrix(
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1);
   	}
};


}

#endif // STIR_MATH_DEF_HPP_
