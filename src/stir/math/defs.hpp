#ifndef STIR_MATH_DEF_HPP_
#define STIR_MATH_DEF_HPP_

#include <cstring>
#include <type_traits>
#include <initializer_list>

// #define Ax (&a.x)
// #define Bx (&b.x)
// #define Tx (&this->x)
// #define Xx(a) (&a.x)

namespace stir
{

#define STIR_USE_RSQRT 0 // need at least another step https://stackoverflow.com/questions/31555260/fast-vectorized-rsqrt-and-reciprocal-with-sse-avx-depending-on-precision/31559382#31559382 / https://stackoverflow.com/questions/1528727/why-is-sse-scalar-sqrtx-slower-than-rsqrtx-x
#define STIR_USE_RSQRT_2STEP 1

struct vec2;
struct vec3;
struct vec4;
struct quat;
struct matrix;

template<class T> constexpr T pi_t = T(3.1415926535897932385);
inline constexpr auto pi = pi_t<float>;

enum cw {};
enum ccw {};


struct vec2
{
	enum { N = 2 };

	vec2() {}
	vec2(simd::impl::F4 a) : v(a) {}
	vec2(float x, float y) : v(simd::impl::assign_4f_2f(x, y)) {}

	float operator[](int i) const { return simd::impl::elem_4f(v, i); }

	inline void normalize();

	simd::impl::F4 v;
};
//static_assert(std::is_standard_layout<vec2>::value == true);

struct vec3
{
	enum { N = 3 };

	vec3() {}
	vec3(simd::impl::F4 a) : v(a) {}
	vec3(float x, float y, float z) : v(simd::impl::assign_4f_3f(x, y, z)) {}
	explicit vec3(vec2 const& v2, float z = 0) : v(simd::impl::swizz<0, 1>::get2(v2.v, z, 0)) {}
	explicit vec3(float x) : v(simd::impl::assign_4f_1f(x)) {}

	void loadu(float const* p) { v = simd::impl::load_3f(p); }

	void set(float x, float y, float z) { v = simd::impl::assign_4f_3f(x, y, z); }
	float operator[](size_t i) const { return simd::impl::elem_4f(v, i); }

	inline void normalize();
	
	simd::impl::F4 v;
};
//static_assert(std::is_standard_layout<vec3>::value == true);

struct vec4
{
	enum { N = 4 };

	vec4() {}
	vec4(simd::impl::F4 a) : v(a) {}
	//explicit vec4(vec3 v, float w = 1.0f) : v(simd::impl::assign_4f_3f_1f(v.v, w)) {}
	explicit vec4(vec3 const& v2, float w = 1.0f) : v(simd::impl::swizz<0, 1, 2>::get3(v2.v, w)) {}
	vec4(float x, float y, float z, float w) : v(simd::impl::assign_4f_4f(x,y,z,w)) {}

	float operator[](size_t i) const { return simd::impl::elem_4f(v, i); }
	
	inline void normalize();

	simd::impl::F4 v;
};
//static_assert(std::is_standard_layout<vec4>::value == true);

// w is the real (scalar) component, vector component is (x,y,z)
struct quat 
{
	enum { N = 4 };
	quat() {}
	quat(simd::impl::F4 a) : v(a) {}
	quat(float x, float y, float z, float w) : v(simd::impl::assign_4f_4f(x,y,z,w)) {}

	static quat identity() { return quat(0,0,0,1.0f); }

	float operator[](size_t i) const { return simd::impl::elem_4f(v, i); }
	vec3 v3() const { return vec3{ simd::impl::swizz<0, 1, 2>::get3(v, 0) }; }
	inline vec3 axis() const;
	float scalar() const { return simd::impl::elem_4f(v, 3); }

	inline void normalize();
	simd::impl::F4 v;
};
//static_assert(std::is_standard_layout<quat>::value == true);

struct matrix
{
	enum { N = 16 };
	simd::impl::F16 v;

	matrix() {}
	matrix(simd::impl::F16 a) : v(a) {}
	matrix( float a0, float a1, float a2, float a3,
			float b0, float b1, float b2, float b3,
			float c0, float c1, float c2, float c3,
			float d0, float d1, float d2, float d3)
	{
		v.val[0] = simd::impl::assign_4f_4f(a0, a1, a2, a3);
		v.val[1] = simd::impl::assign_4f_4f(b0, b1, b2, b3);
		v.val[2] = simd::impl::assign_4f_4f(c0, c1, c2, c3);
		v.val[3] = simd::impl::assign_4f_4f(d0, d1, d2, d3);
	}
	matrix(float const* p) {
		v.val[0] = simd::impl::load_4f(p + 0);
		v.val[1] = simd::impl::load_4f(p + 4);
		v.val[2] = simd::impl::load_4f(p + 8);
		v.val[3] = simd::impl::load_4f(p +12);
	}

	matrix(simd::impl::F4 c0, simd::impl::F4 c1, simd::impl::F4 c2, simd::impl::F4 c3)
	:	v( {c0, c1, c2, c3 } )
	{ }

	matrix(vec4 c0, vec4 c1, vec4 c2, vec4 c3)
	:	v( {c0.v, c1.v, c2.v, c3.v } )
	{ }

	// matrix(std::initializer_list<float> l)
	// {
	// 	memcpy(x, l.begin(), sizeof(float)*(l.end()-l.begin()));
	// }

	void set(size_t col, float x, float y, float z, float w) { v.val[col] = simd::impl::assign_4f_4f(x, y, z, w); }
	void set(size_t col, simd::impl::F4 a) { v.val[col] = a; }
	void set(size_t col, vec4 a) { v.val[col] = a.v; }

	// simd::impl::F4 col(size_t col) const { return v.val[col]; }
	vec4 col(size_t col) const { return vec4{ v.val[col] }; }
	float el(size_t i) const { return simd::impl::elem_16f(v, i>>2, i&3); }

	// float* row(size_t i) { return x + 4*i; }
	// float const* row(size_t i) const { return x + 4*i; }

	// i is row/y, j is column/x
	float operator()(int i, int j) const { return simd::impl::elem_16f(v, i, j); }
	// float operator[](int i, int j) const { return v.val[j][i]; }

	matrix const& operator+=(matrix const& a);
	matrix const& operator-=(matrix const& a);
	matrix const& operator*=(matrix const& a);
	matrix const& operator/=(matrix const& a);

	matrix const& operator+=(float k);
	matrix const& operator-=(float k);
	matrix const& operator*=(float k);
	matrix const& operator/=(float k);

	static matrix identity()
	{
		return matrix(
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1);
   	}
};

struct matrix33 {
	enum { N = 12 };

	simd::impl::F12 v;

	matrix33() {}
	matrix33(simd::impl::F12 a) : v(a) {}

	matrix33(	float a0, float a1, float a2,
				float b0, float b1, float b2,
				float c0, float c1, float c2)
	{
		v.val[0] = simd::impl::assign_4f_4f(a0, a1, a2, 0);
		v.val[1] = simd::impl::assign_4f_4f(b0, b1, b2, 0);
		v.val[2] = simd::impl::assign_4f_4f(c0, c1, c2, 0);
	}
	matrix33(float const* p) {
		v.val[0] = simd::impl::load_3f(p + 0);
		v.val[1] = simd::impl::load_3f(p + 3);
		v.val[2] = simd::impl::load_3f(p + 6);
	}

	float operator()(int i, int j) const { return simd::impl::elem_12f(v, i, j); }
};

} // stir

#endif // STIR_MATH_DEF_HPP_
