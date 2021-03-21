#ifndef STIR_MATH_HPP_
#define STIR_MATH_HPP_

#include <xsimd/xsimd.hpp>
#include <fmt/format.h>


#if defined(__x86_64)
#define STIR_MATH_X64 1
#else
#define STIR_MATH_X64 0
#endif

namespace stir
{

using BF4 = xsimd::batch<float, 4>; 

union Vector2;
union Vector3;
union Vector4;
union Quaternion;
union Matrix;

template<class T>
T pi = T(3.1415926535897932385);


// Generated code quality-wise, it does not matter whether class holds a xsimd::batch or straight float array.
// It's important do to loads correctly.

union Vector2
{
	Vector2() {}
	Vector2(float ax, float ay) : x(ax), y(ay) {}

	struct { float x, y; };
};

// Vector is interpreted as 1x4 (a row) when treated as a matrix. Thus vector/matrix multiplication
// happens as v * M
union Vector3
{
	struct { float x, y, z; };

	Vector3() {}
	Vector3(float ax, float ay, float az) : x(ax), y(ay), z(az) {}

	Vector4 as_vec4(float w = 1.0f) const;
	void store3(BF4 a) {
		el[0] = a[0];
		el[1] = a[1];
		el[2] = a[2];
	}

	float operator[](int i) const { return el[i]; }
	float& operator[](int i) { return el[i]; }

	void normalize();
	Vector3 normalized() const;

private:
	float el[3];
};

union Vector4
{
	Vector4() {}
#if 0
	Vector4(xsimd::batch<float, 4> b) : v(b) {}
	Vector4(float x, float y, float z, float w) : v(x, y, z, w) {}
	Vector4 operator+(Vector4 const& a) const {
		//return Vector4( v + a.v );
		BF4 s1, s2;
		s1.load_unaligned(el);
		s2.load_unaligned(a.el);
		return Vector4(s1 + s2);
	}

	bool operator==(Vector4 const& a) const {
	   	xsimd::batch_bool<float, 4> r = v == a.v;
		xsimd::batch<int32_t,4> rf(r);

		int sum = xsimd::hadd(rf);
		return sum == -4;
	}

	BF4 simd_data() const { return v; }
	xsimd::batch<float, 4> v;
#else
	Vector4(xsimd::batch<float, 4> b)
	{
		b.store_aligned(el);
	}
	Vector4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}

	Vector4 operator+(Vector4 const& a) const {
		BF4 s1, s2;
		s1.load_unaligned(el);
		s2.load_unaligned(a.el);
		return Vector4(s1 + s2);
	}

	bool operator==(Vector4 const& a) const {
		BF4 ba(el), bb(a.el);
	   	xsimd::batch_bool<float, 4> r = ba == bb;
		xsimd::batch<int32_t,4> rf(r);

		int sum = xsimd::hadd(rf);
		return sum == -4;
	}

	void normalize() {
		BF4 aa(el);
		BF4 aax = aa * aa;
		float sq = sqrtf(xsimd::hadd(aax));
		if (sq >= 1e-6f)
			aa /= sq;
		
		aa.store_aligned(el);
	}

	Vector4 normalized() const {
		BF4 aa(el);
		BF4 aax = aa * aa;
		float sq = sqrtf(xsimd::hadd(aax));
		if (sq >= 1e-6f)
			aa /= sq;
		return aa;
	}


	BF4 simd_data() const { return BF4(el); }
#endif

	float operator[](int i) const { return el[i]; }
	//float& operator[](int i) { return v[i]; }

	struct { float x, y, z, w; };

private:
	alignas(16) float el[4];
};

inline Vector4 Vector3::as_vec4(float w) const { return Vector4(x,y,z,w); }
inline void Vector3::normalize() {
	BF4 aa(el[0], el[1], el[2], 0);
	BF4 aax = aa * aa;
	float sq = sqrtf(xsimd::hadd(aax));
	if (sq >= 1e-6f)
		aa /= sq;
	store3(aa);
}

inline Vector3 Vector3::normalized() const {
	BF4 aa(el[0], el[1], el[2], 0);
	BF4 aax = aa * aa;
	float sq = sqrtf(xsimd::hadd(aax));
	if (sq >= 1e-6f)
		aa /= sq;
	Vector3 v3;
	v3.store3(aa);
	return v3;
}


union Quaternion
{
	Quaternion() {}
	Quaternion(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
	Quaternion(BF4 a) : v(a) {}

	Quaternion conj() const { 
		BF4 q(el);
		BF4 nq(-q);
		xsimd::batch_bool<float, 4> sel(xsimd::batch_bool<int, 4>(0,0,0,-1));
		return xsimd::select(sel, nq, q);
   	}
	Matrix rot_matrix() const;
	static Quaternion identity() { return Quaternion(0,0,0,1.0f); }

	float operator[](int i) const { return el[i]; }
	float& operator[](int i) { return el[i]; }

	void normalize() {
		BF4 aa(el);
		BF4 aax = aa * aa;
		float sq = sqrtf(xsimd::hadd(aax));
		if (sq >= 1e-6f)
			aa /= sq;
		aa.store_aligned(el);
	}

	float length() const { BF4 aa(el); return sqrtf(xsimd::hadd(aa * aa)); }
	float sq_length() const { BF4 aa(el); return xsimd::hadd(aa*aa); }

	Quaternion normalized() const {
		BF4 aa(el);
		BF4 aax = aa * aa;
		float sq = sqrtf(xsimd::hadd(aax));
		if (sq >= 1e-6f)
			aa /= sq;
		return aa;
	}

	bool operator==(Quaternion const& a) const {
		BF4 aa(el), bb(a.el);
	   	xsimd::batch_bool<float, 4> r = aa == bb;
		xsimd::batch<int32_t,4> rf(r);
		int sum = xsimd::hadd(rf);
		return sum == -4;
	}

	Quaternion operator+(Quaternion a) const { return Quaternion(v + a.v); }
	Quaternion operator-(Quaternion a) const { return Quaternion(v - a.v); }

	xsimd::batch<float, 4> v;
	struct { float x, y, z, w; };
	
	static Quaternion from_axis_angle(Vector3 const& v, float a);

private:
	float el[4];
};

inline Quaternion Quaternion::from_axis_angle(Vector3 const& v, float a)
{
	float a2 = a/2.0f;
#if 1
	float sa = sin(a2), ca = cos(a2);
	return Quaternion (v.x*sa, v.y*sa, v.z*sa, ca);
#else

	float sa = sin(a2);
	BF4 sa1(sa, sa, sa, 1);
	BF4 vv = v.as_vec4(ca);
	return Quaternion(vv * sa);
#endif
}

inline Quaternion mul_ref(Quaternion const& a, Quaternion const& b)
{
    float x = a.v[3] * b.v[0] + a.v[0] * b.v[3] + a.v[1] * b.v[2] - a.v[2] * b.v[1];
    float y = a.v[3] * b.v[1] - a.v[0] * b.v[2] + a.v[1] * b.v[3] + a.v[2] * b.v[0];
    float z = a.v[3] * b.v[2] + a.v[0] * b.v[1] - a.v[1] * b.v[0] + a.v[2] * b.v[3];
	float w = a.v[3] * b.v[3] - a.v[0] * b.v[0] - a.v[1] * b.v[1] - a.v[2] * b.v[2];

	return Quaternion(x, y, z, w);
}

// https://stackoverflow.com/questions/18542894/how-to-multiply-two-quaternions-with-minimal-instructions
// http://momchil-velikov.blogspot.com/2013/10/fast-sse-quternion-multiplication.html

inline Quaternion mul(Quaternion a, Quaternion b)
{
#if 0
	return mul_ref(a, b);
#else

//#define _mm_shufpsd(r,i) _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(aa), i))
#define _mm_shufpsd(r,i) _mm_shuffle_ps(r, r, i)

#if STIR_MATH_X64

	// note: _MM_SHUFFLE is little-endian last, so the order is reversed
	__m128 aa(a.v);
	__m128 a3 = _mm_shufpsd(aa, 0xff), a0 = _mm_shufpsd(aa, 0x00), a1 = _mm_shufpsd(aa, 0x55), a2 = _mm_shufpsd(aa, 0xaa);
	
	__m128 bb = b.v;
	__m128 bs0 = _mm_shufpsd(bb, _MM_SHUFFLE(2,3,0,1)); 
	BF4 bs01( b.v[1], b.v[0], b.v[3], b.v[2]);
	__m128 bs1 = _mm_shufpsd(bb, _MM_SHUFFLE(1,0,3,2));
	__m128 bs2 = _mm_shufpsd(bb, _MM_SHUFFLE(2,0,3,1));
	__m128 bs3 = _mm_shufpsd(bb, _MM_SHUFFLE(3,1,0,2));
#else
	BF4 aa(a.v);
	BF4 a3(aa[3]), a0(aa[0]), a1(aa[1]), a2(aa[2]);

	BF4 bb(b.v);

	BF4 bs0( bb[0], bb[1], bb[2], bb[3]);
	BF4 bs1( bb[3],-bb[2], bb[1],-bb[0]);
	BF4 bs2( bb[2], bb[3],-bb[0],-bb[1]);
	BF4 bs3(-bb[1], bb[0], bb[3],-bb[2]);

	BF4 q0 = a3 * bs0;
	BF4 q1 = a0 * bs1;
	BF4 q2 = a1 * bs2;
	BF4 q3 = a2 * bs3;
#endif

#if STIR_MATH_X64

	// this is the original order, now let's rearrange columns so we get as much natural addsub as possible
	// the order for MM_SHUFFLE below is reversed
	// 0  1  2  3
	// 3 -2  1 -0 
	// 2  3 -0 -1
	//-1  0  3 -2
	
	// 1  0  3  2 (order for bs0)
	//-2  3 -0  1 (order for bs1)
	
	// 3  1  2  0 (new order for bs0/temp) prev pos = (2 0 3 1)
	//-1  3 -0  2 (order for bs2)
	
	// 3  1  0  2 (new order for bs0/temp) prev pos = (0 1 3 2)
	//-2  0 -1  3 (order for bs3)
	
	// reshufle into corect positions: (2 1 3 0)
	__m128 q0 = _mm_mul_ps(a3, bs0);
	__m128 q1 = _mm_mul_ps(a0, bs1);
	__m128 q2 = _mm_mul_ps(a1, bs2);
	__m128 q3 = _mm_mul_ps(a2, bs3);
	
	__m128 tq1 = _mm_shufpsd(_mm_addsub_ps(q0, q1), _MM_SHUFFLE(1,3,0,2));
	__m128 tq2 = _mm_shufpsd(_mm_addsub_ps(tq1, q2), _MM_SHUFFLE(2,3,1,0));
	__m128 tq3 = _mm_shufpsd(_mm_addsub_ps(tq2, q3), _MM_SHUFFLE(0,3,1,2));

	return Quaternion(tq3);
#else
	return Quaternion(q0 + q1 + q2 + q3);
#endif
#endif
}

inline Quaternion operator*(Quaternion const& a, Quaternion const& b) { return mul(a, b); }

inline Vector3 rot(Vector3 const& v, Quaternion const& q)
{
	Quaternion qv(v.x, v.y, v.z, 0);
	Quaternion m = q * qv * q.conj();
	return Vector3(m.x, m.y, m.z);
}


union Matrix
{
	BF4 row[4];
	float el[4][4];

	Matrix() {}
	Matrix( float a1, float a2, float a3, float a4,
			float b1, float b2, float b3, float b4,
			float c1, float c2, float c3, float c4,
			float d1, float d2, float d3, float d4)
	{
		el[0][0]=a1; el[0][1]=a2; el[0][2]=a3; el[0][3]=a4;
		el[1][0]=b1; el[1][1]=b2; el[1][2]=b3; el[1][3]=b4;
		el[2][0]=c1; el[2][1]=c2; el[2][2]=c3; el[2][3]=c4;
		el[3][0]=d1; el[3][1]=d2; el[3][2]=d3; el[3][3]=d4;
	}
	
	void set(int i, float a, float b, float c, float d) { row[i] = BF4(a,b,c,d); }

	BF4 load_row(size_t i) const { return BF(el[i]); }
	void store_row(size_t i, BF4 row) const { return row.store(el[i]); }

	float operator()(int i, int j) const { return el[i][j]; }

	Matrix const& operator+=(Matrix const& a)
	{
		for(int i=0; i<4; ++i)
		{
			BF4 r = load_row(i) + a.load_row(i);
			store_row(i, r);
		}
		return *this;
	}

	Matrix operator*(float f)
	{
		Matrix r;
		BF4 ff(f);
		for (int i=0; i<4; ++i) {
			BF4 row = load_row(i) * ff;
			r.store_row(i, row);
		}

		return r;
	}

	Matrix& operator*=(float m)
	{
		for (int i=0; i<4; ++i)
			row[i] *= m;
		return *this;
	}

	Matrix operator/(float m)
	{
		Matrix r;
		BF4 ff(f);
		for (int i=0; i<4; ++i) {
			BF4 row = load_row(i) / ff;
			r.store_row(i, row);
		}
		return r;
	}

	Matrix& operator/=(float m)
	{
		for (int i=0; i<4; ++i) {
			BF4 row = load_row(i) * ff;
			store_row(i, row);
		}
		return *this;
	}

	static Matrix identity()
	{
		Matrix m;
		m.set(0, 1, 0, 0, 0);
		m.set(1, 0, 1, 0, 0);
		m.set(2, 0, 0, 1, 0);
		m.set(3, 0, 0, 0, 1);
		return m;
   	}
};

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

inline Matrix mul_ref(Matrix const& a, Matrix const& b)
{
	Matrix r;
	for (int i=0; i<4; ++i)
		for(int j=0; j<4; ++j)
			r.el[i][j] = a.el[i][0]*b.el[0][j] + a.el[i][1]*b.el[1][j] + a.el[i][2]*b.el[2][j] + a.el[i][3]*b.el[3][j];
	return r;
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

inline Matrix Quaternion::rot_matrix() const
{
#if 1
	// http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/jay.htm
	Matrix m1{
		  w,  z, -y, x,
		 -z,  w,  x, y,
		  y, -x,  w, z,
		 -x, -y, -z, w
	};

	Matrix m2{
		 w,  z, -y, -x,
		-z,  w,  x, -y,
		 y, -x,  w, -z,
		 x,  y,  z,  w
	};

	return mul(m1, m2);
#else
	float xx = x * x;
    float xy = x * y;
    float xz = x * z;
    float xw = x * w;

    float yy = y * y;
    float yz = y * z;
    float yw = y * w;

    float zz = z * z;
    float zw = z * w;

	return Matrix{
		{ 1 - 2 * ( yy + zz ),	2 * ( xy - zw ),		2 * ( xz + yw ),	0 },
		{ 2 * ( xy + zw ), 		1 - 2 * ( xx + zz ), 	2 * ( yz - xw ),	0 },
		{ 2 * ( xz - yw ),		2 * ( yz + xw ), 		1 - 2 * ( xx + yy ),0 },
		{ 0, 					0,						0,					0 }
	};
#endif
}

inline xsimd::batch<float, 4> dos(xsimd::batch<float, 4> a)
{
	return xsimd::batch<float, 4>(a[2], a[1], a[1], a[3]);
}

}

#include <fmt/ostream.h>

template <>
struct fmt::formatter<stir::Vector2> {
  template <typename ParseContext>
  constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const stir::Vector2 &a, FormatContext &ctx) {
    return format_to(ctx.out(), "{{ {} {} }}", a.x, a.y);
  }
};

template <>
struct fmt::formatter<stir::Vector3> {
  template <typename ParseContext>
  constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const stir::Vector3 &a, FormatContext &ctx) {
    return format_to(ctx.out(), "{{ {} {} {} }}", a.x, a.y, a.z);
  }
};

template <>
struct fmt::formatter<stir::Vector4> {
  template <typename ParseContext>
  constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const stir::Vector4 &a, FormatContext &ctx) {
    return format_to(ctx.out(), "{{ {} {} {} {} }}", a.x, a.y, a.z, a.w);
  }
};

template <>
struct fmt::formatter<stir::Quaternion> {
  template <typename ParseContext>
  constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const stir::Quaternion &a, FormatContext &ctx) {
    return format_to(ctx.out(), "{{ {} {} {} {} }}", a.x, a.y, a.z, a.w);
  }
};

namespace stir {
	inline std::ostream& operator<<(std::ostream& os, stir::Vector2 const& a) { fmt::print(os, "{}", a); return os; }
	inline std::ostream& operator<<(std::ostream& os, stir::Vector3 const& a) { fmt::print(os, "{}", a); return os; }
	inline std::ostream& operator<<(std::ostream& os, stir::Vector4 const& a) { fmt::print(os, "{}", a); return os; }
	inline std::ostream& operator<<(std::ostream& os, stir::Quaternion const& a) { fmt::print(os, "{}", a); return os; }
}




#endif
