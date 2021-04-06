#include "math_def.hpp"

#include <mipp.h>

#define Ax (&a.x)
#define Bx (&b.x)
#define Tx (&this->x)
#define Xx(a) (&a.x)


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
inline vec4 normalize(vec4 const& a) { return vec4(impl::n4(S(Ax))); }
inline quat normalize(quat const& a) { return quat(impl::n4(S(Ax))); }

inline S as_simd(vec3 const& a, float w = 0.0f) { return S({a.x, a.y, a.z, w}); }
inline S as_simd(vec4 const& a) { return S({a.x, a.y, a.z, a.w}); }
inline S as_simd(quat const& a) { return S({a.x, a.y, a.z, a.w}); }
//inline S load_v3_0(quat const& a) { return S s(&a.x); a[3]=0; return s; }

inline vec2 operator+(vec2 const& a, vec2 const& b) { return vec2(a.x+b.x, a.y+b.y); }
inline vec3 operator+(vec3 const& a, vec3 const& b) { return vec3(as_simd(a)+as_simd(b)); }
inline vec4 operator+(vec4 const& a, vec4 const& b) { return vec4(as_simd(a)+as_simd(b)); }
inline quat operator+(quat const& a, quat const& b) { return quat(as_simd(a)+as_simd(b)); }
inline vec2 operator+(vec2 const& a, float b) { return vec2(a.x+b, a.y+b); }
inline vec3 operator+(vec3 const& a, float b) { return vec3(as_simd(a)+S(b)); }
inline vec4 operator+(vec4 const& a, float b) { return vec4(as_simd(a)+S(b)); }
inline quat operator+(quat const& a, float b) { return quat(as_simd(a)+S(b)); }

inline vec2 operator-(vec2 const& a, vec2 const& b) { return vec2(a.x-b.x, a.y-b.y); }
inline vec3 operator-(vec3 const& a, vec3 const& b) { return vec3(as_simd(a)-as_simd(b)); }
inline vec4 operator-(vec4 const& a, vec4 const& b) { return vec4(as_simd(a)-as_simd(b)); }
inline quat operator-(quat const& a, quat const& b) { return quat(as_simd(a)-as_simd(b)); }
inline vec2 operator-(vec2 const& a, float b) { return vec2(a.x-b, a.y-b); }
inline vec3 operator-(vec3 const& a, float b) { return vec3(as_simd(a)-S(b)); }
inline vec4 operator-(vec4 const& a, float b) { return vec4(as_simd(a)-S(b)); }
inline quat operator-(quat const& a, float b) { return quat(as_simd(a)-S(b)); }

inline vec2 operator*(vec2 const& a, float b) { return vec2(a.x*b, a.y*b); }
inline vec3 operator*(vec3 const& a, float b) { return vec3(as_simd(a)*S(b)); }
inline vec4 operator*(vec4 const& a, float b) { return vec4(as_simd(a)*S(b)); }
inline quat operator*(quat const& a, float b) { return quat(as_simd(a)*S(b)); }

inline vec2 operator/(vec2 const& a, float b) { return vec2(a.x/b, a.y/b); }
inline vec3 operator/(vec3 const& a, float b) { return vec3(as_simd(a)/S(b)); }
inline vec4 operator/(vec4 const& a, float b) { return vec4(as_simd(a)/S(b)); }
inline quat operator/(quat const& a, float b) { return quat(as_simd(a)/S(b)); }


inline bool operator==(vec3 const& a, vec3 const& b) {
	S s1=as_simd(a), s2=as_simd(b);
	M r = s1 == s2;
	mipp::Reg<int> s = r.toReg<int>();
	int sum = s.hadd();
	return sum == -4;
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
//inline quat operator+(quat const& a, quat const& b) { S s1(Ax), s2(Bx); return quat(s1+s2); }
//inline quat operator-(quat const& a, quat const& b) { S s1(Ax), s2(Bx); return quat(s1-s2); }

inline quat conj(quat const& a)
{
	S q(Ax);
	M m{false,false,false,true};
	return quat(q.neg(m));
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
	return quat(q0 + q1 + q2 + q3);
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

#undef Ax
#undef Bx
#undef Xx
#undef Tx

#include "math_common.hpp"

