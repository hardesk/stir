#include "math_def.hpp"

#include <mipp.h>

/*
// extension to mipp
namespace mipp
{
	template<class S>
	struct const_shuff;

	template<unsigned A, unsigned B, unsigned C, unsigned D>
	struct const_shuff<float, A, B, C, D>
	{
		static reg doit(reg a) { return _mm_shuffle_epi32(a, _MM_SHUFFLE(A, B, C, D)); }
	};

	template<unsigned A, unsigned B>
	struct const_shuff<double, A, B>
	{
		static reg doit(reg a) { return _mm_shuffle_pd(a, A|(B<<1)); }
	};


	// arm
	template<unsigned A, unsigned B, unsigned C, unsigned D>
	struct const_shuff<float, A, B, C, D>
	{
		static reg doit(reg a) { return _mm_shuffle_epi32(a, _MM_SHUFFLE(A, B, C, D)); }
	};

	template<>
	struct const_shuff<float, 0, 0, 0, 0>
	{
		static reg doit(reg a) { return vdupq_lane_f32( vget_low_f32(a), 0); }
	};

	
}*/

namespace stir
{

#if STIR_MATH_MODE == STIR_MATH_MIPP
inline
#endif
namespace simd_mipp {

namespace impl 
{
	inline S n4(S s) {
		S ss = s * s;
		S ha(ss.hadd());
#if STIR_USE_RSQRT
		S rs = ha.rsqrt();
#if STIR_USE_RSQRT_2STEP
		S three(3.0f);
		S half(0.5f);
		rs = half * rs * (three - ha * rs * rs);
#endif
		s *= rs;

#else
		S sqrt = ha.sqrt();
		s /= sqrt;
#endif
		return s;
	}

	inline float l4(S a) {
		S aa = a*a;
		return sqrtf(aa.hadd());
	}
	inline float sql4(S a) {
		S aa=a*a;
		return aa.hadd();
	}
}

inline float length(vec2 const& a) { return impl::l4(S{a.x, a.y, 0, 0}); }
inline float length(vec3 const& a) { return impl::l4(S{a.x, a.y, a.z, 0}); }
inline float length(vec4 const& a) { return impl::l4(S(Ax)); }
inline float length(quat const& a) { return impl::l4(S(Ax)); }

inline float sq_length(vec2 const& a) { return impl::sql4(S{a.x, a.y, 0, 0}); }
inline float sq_length(vec3 const& a) { return impl::sql4(S{a.x, a.y, a.z, 0}); }
inline float sq_length(vec4 const& a) { return impl::sql4(S(Ax)); }
inline float sq_length(quat const& a) { return impl::sql4(S(Ax)); }

inline vec2 normalize(vec2 const& a) { return vec2(impl::n4(S{a.x, a.y, 0, 0})); }
inline vec3 normalize(vec3 const& a) { return vec3(impl::n4(S{a.x, a.y, a.z, 0})); }
inline vec4 normalize(vec4 const& a) { return impl::n4(S(Ax)); }
inline quat normalize(quat const& a) { return quat(impl::n4(S(Ax))); }

inline S as_simd(vec3 const& a, float w = 0.0f) { return S({a.x, a.y, a.z, w}); }
inline S as_simd(vec4 const& a) { return S({a.x, a.y, a.z, a.w}); }
inline S as_simd(quat const& a) { return S({a.x, a.y, a.z, a.w}); }

inline bool operator==(vec3 const& a, vec3 const& b) {
	S s1=as_simd(a), s2=as_simd(b);
	M r = s1 == s2;
	mipp::Reg<int> s = r.toReg<int>();
	int sum = s.hadd();
	return sum == -4;
}


inline vec4 operator+(vec4 const& a, vec4 const& b) {
	S s1(Ax), s2(Bx);
	return vec4(s1 + s2);
}

inline bool operator==(vec4 const& a, vec4 const& b) {
	S s1(Ax), s2(Bx);
	M r = s1 == s2;
	mipp::Reg<int> s = r.toReg<int>();
	int sum = s.hadd();
	return sum == -4;
}
inline bool operator==(quat const& a, quat const& b) {
	S s1(Ax), s2(Bx);
	M r = s1 == s2;
	mipp::Reg<int> s = r.toReg<int>();
	int sum = s.hadd();
	return sum == -4;
}
inline quat operator+(quat const& a, quat const& b) { S s1(Ax), s2(Bx); return quat(s1+s2); }
inline quat operator-(quat const& a, quat const& b) { S s1(Ax), s2(Bx); return quat(s1-s2); }

inline quat conj(quat const& a)
{
	S q(Ax);
	M m{false,false,false,true};
	return q.neg(m);
}

inline quat from_axis_angle(vec3 const& axis, float angle)
{
#if 0
	return ref::from_axis_angle(axis, angle);
#else
	float a2=angle/2;
	float sa = sin(a2);
	float ca = cos(a2);
	S sa1{sa, sa, sa, ca};
	S vv = as_simd(axis,1.0f);
	return quat(vv * sa);
#endif
}

// https://stackoverflow.com/questions/18542894/how-to-multiply-two-quaternions-with-minimal-instructions
// http://momchil-velikov.blogspot.com/2013/10/fast-sse-quternion-multiplication.html

inline quat mul(quat const& a, quat const& b)
{
#if 0
	return mul_ref(a, b);
#else

//#define _mm_shufpsd(r,i) _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(aa), i))
#define _mm_shufpsd(r,i) _mm_shuffle_ps(r, r, i)

#if STIR_MATH_X64
	// note: _MM_SHUFFLE index 0 is right-most (least significant little endian), so x:0 y:1 z:2 w:3
	__m128 aa = as_simd(a).r;
	__m128 a3 = _mm_shufpsd(aa, 0xff), a0 = _mm_shufpsd(aa, 0x00), a1 = _mm_shufpsd(aa, 0x55), a2 = _mm_shufpsd(aa, 0xaa);
	
	__m128 bb = as_simd(b).r;
	__m128 bs0 = _mm_shufpsd(bb, _MM_SHUFFLE(2,3,0,1)); 
	__m128 bs1 = _mm_shufpsd(bb, _MM_SHUFFLE(1,0,3,2));
	__m128 bs2 = _mm_shufpsd(bb, _MM_SHUFFLE(2,0,3,1));
	__m128 bs3 = _mm_shufpsd(bb, _MM_SHUFFLE(3,1,0,2));
#else
	S aa(Ax);
	S a3(aa[3]), a0(aa[0]), a1(aa[1]), a2(aa[2]);

	S bb(Bx);

	S bs0{ bb[0], bb[1], bb[2], bb[3]};
	S bs1{ bb[3],-bb[2], bb[1],-bb[0]};
	S bs2{ bb[2], bb[3],-bb[0],-bb[1]};
	S bs3{-bb[1], bb[0], bb[3],-bb[2]};

	S q0 = a3 * bs0;
	S q1 = a0 * bs1;
	S q2 = a1 * bs2;
	S q3 = a2 * bs3;
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

	return quat(tq3);
#else
	return quat(q0 + q1 + q2 + q3);
#endif
#endif
}

#define STIR_MATRIX_UN_OP(op) for(int i=0; i<4; ++i) { S q=S(x+i*4) op S(z.x+i*4); q.store(x+i*4); } return *this
#define STIR_MATRIX_UN_S_OP(op) S kk(k); for(int i=0; i<4; ++i) { S q=S(x+i*4) op kk; q.store(x+i*4); } return *this
#define STIR_MATRIX_BIN_OP(op) matrix r; for(int i=0; i<4; ++i) { S q=S(a.x+i*4) op S(b.x+i*4); q.store(r.x+i*4); } return r
#define STIR_MATRIX_BIN_S_OP(op) matrix r; S kk(k); for(int i=0; i<4; ++i) { S q=S(a.x+i*4) op kk; q.store(r.x+i*4); } return r

#define STIR_MATRIX_BIN_S_OP(op) matrix r; S kk(k); for(int i=0; i<4; ++i) { S q=S(a.x+i*4) op kk; q.store(r.x+i*4); } return r

inline matrix operator-(matrix const& a, matrix const& b) { STIR_MATRIX_BIN_OP(-); }
inline matrix operator+(matrix const& a, matrix const& b) { STIR_MATRIX_BIN_OP(+); }
inline matrix operator*(matrix const& a, matrix const& b) { STIR_MATRIX_BIN_OP(*); }
inline matrix operator/(matrix const& a, matrix const& b) { STIR_MATRIX_BIN_OP(/); }

inline matrix operator+(matrix const& a, float k) { STIR_MATRIX_BIN_S_OP(+); }
inline matrix operator-(matrix const& a, float k) { STIR_MATRIX_BIN_S_OP(-); }
inline matrix operator*(matrix const& a, float k) { STIR_MATRIX_BIN_S_OP(*); }
inline matrix operator/(matrix const& a, float k) { STIR_MATRIX_BIN_S_OP(/); }

/*inline matrix operator*(matrix const& a, float k) {
	S kk(k);
	S x1(a.x+0);
	S x2(a.x+4);
	S x3(a.x+8);
	S x4(a.x+12);
	return matrix(x1*kk,x2*kk,x3*kk,x4*kk);
}*/

} // simd_mipp

inline vec2::vec2(S a) { x=a[0]; y=a[1]; }
inline vec3::vec3(S a)
{
	x=a[0]; y=a[1]; z=a[2];
}

#if STIR_MATH_MODE == STIR_MATH_MIPP
inline void vec2::normalize() { S s = simd_mipp::impl::n4(S{x,y,0,0}); x=s[0]; y=s[1]; }
inline void vec3::normalize() { S s = simd_mipp::impl::n4(S{x,y,z,0}); x=s[0]; y=s[1]; z=s[2]; }
inline void vec4::normalize() { simd_mipp::impl::n4(S(Tx)).storeu(Tx); }
inline void quat::normalize() { simd_mipp::impl::n4(S(Tx)).storeu(Tx); }
#endif

inline vec4::vec4(S a)
{
	a.store(Tx);
}

inline vec4 vec3::as_vec4(float w) const { return vec4(x,y,z,w); }


inline quat::quat(S a) { a.storeu(Tx); }

inline matrix const& matrix::operator+=(matrix const& z) { STIR_MATRIX_UN_OP(+); }
inline matrix const& matrix::operator-=(matrix const& z) { STIR_MATRIX_UN_OP(-); }
inline matrix const& matrix::operator*=(matrix const& z) { STIR_MATRIX_UN_OP(*); }
inline matrix const& matrix::operator/=(matrix const& z) { STIR_MATRIX_UN_OP(/); }

} 

#include "math_common.hpp"

