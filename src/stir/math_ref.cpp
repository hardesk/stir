#include <cmath>
#include "math_ref.hpp"

namespace stir {

#if STIR_MATH_MODE == STIR_MATH_REF
inline
#endif
namespace ref
{

quat mul(quat const& a, quat const& b)
{
    float x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    float y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    float z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
	float w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;

	return quat(x, y, z, w);
}

matrix mul(matrix const& a, matrix const& b)
{
	matrix r;
	for (int i=0; i<4; ++i)
		for(int j=0; j<4; ++j)
			r.x[i*4+j] = a.x[i*4+0]*b.x[0*4+j] + a.x[i*4+1]*b.x[1*4+j] + a.x[i*4+2]*b.x[2*4+j] + a.x[i*4+3]*b.x[3*4+j];
	return r;
}

vec4 mul(vec4 const& v, matrix const& a)
{
	float v0 = v.x*a.x[0*4+0] + v.y*a.x[1*4+0] + v.z*a.x[2*4+0] + v.w*a.x[3*4+0];
	float v1 = v.x*a.x[0*4+1] + v.y*a.x[1*4+1] + v.z*a.x[2*4+1] + v.w*a.x[3*4+1];
	float v2 = v.x*a.x[0*4+2] + v.y*a.x[1*4+2] + v.z*a.x[2*4+2] + v.w*a.x[3*4+2];
	float v3 = v.x*a.x[0*4+3] + v.y*a.x[1*4+3] + v.z*a.x[2*4+3] + v.w*a.x[3*4+3];
	return vec4(v0, v1, v2, v3);
}

vec4 mul(vec3 const& v, matrix const& a)
{
	float v0 = v.x*a.x[0*4+0] + v.y*a.x[1*4+0] + v.z*a.x[2*4+0] + a.x[3*4+0];
	float v1 = v.x*a.x[0*4+1] + v.y*a.x[1*4+1] + v.z*a.x[2*4+1] + a.x[3*4+1];
	float v2 = v.x*a.x[0*4+2] + v.y*a.x[1*4+2] + v.z*a.x[2*4+2] + a.x[3*4+2];
	float v3 = v.x*a.x[0*4+3] + v.y*a.x[1*4+3] + v.z*a.x[2*4+3] + a.x[3*4+3];
	return vec4(v0, v1, v2, v3);
}

vec3 mul3(vec3 const& v, matrix const& a)
{
	float v0 = v.x*a.x[0*4+0] + v.y*a.x[1*4+0] + v.z*a.x[2*4+0];
	float v1 = v.x*a.x[0*4+1] + v.y*a.x[1*4+1] + v.z*a.x[2*4+1];
	float v2 = v.x*a.x[0*4+2] + v.y*a.x[1*4+2] + v.z*a.x[2*4+2];
	return vec3(v0, v1, v2);
}

matrix rot_matrix(quat const& q)
{
#if 1
	// http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/jay.htm
	matrix m1{
		  q.w,  q.z, -q.y, q.x,
		 -q.z,  q.w,  q.x, q.y,
		  q.y, -q.x,  q.w, q.z,
		 -q.x, -q.y, -q.z, q.w
	};

	matrix m2{
		 q.w,  q.z, -q.y, -q.x,
		-q.z,  q.w,  q.x, -q.y,
		 q.y, -q.x,  q.w, -q.z,
		 q.x,  q.y,  q.z,  q.w
	};

	return ref::mul(m1, m2);
#else
	float xx = q.x * q.x;
    float xy = q.x * q.y;
    float xz = q.x * q.z;
    float xw = q.x * q.w;

    float yy = q.y * q.y;
    float yz = q.y * q.z;
    float yw = q.y * q.w;

    float zz = q.z * q.z;
    float zw = q.z * q.w;

	return matrix{
		{ 1 - 2 * ( yy + zz ),	2 * ( xy - zw ),		2 * ( xz + yw ),	0 },
		{ 2 * ( xy + zw ), 		1 - 2 * ( xx + zz ), 	2 * ( yz - xw ),	0 },
		{ 2 * ( xz - yw ),		2 * ( yz + xw ), 		1 - 2 * ( xx + yy ),0 },
		{ 0, 					0,						0,					0 }
	};
#endif
}

quat exp(quat const& a)
{
	// exp(q)=e^w * (cos|v| + v/|v|*sin|v|)
	float ew=std::exp(a.w);
	float l=length(a.v());
	float cv=std::cosf(l);
	float sv=std::sinf(l);
	float m=ew/l;
	return quat(a.x*m*sv, a.y*m*sv, a.z*m*sv, ew*cv);
}

quat log(quat const& a)
{
	// ln(q)=ln|q|+v/|v|arccos(a/|q|)
	float lq=length(a);
	float lv=length(a.v());
	float ac=std::acos(a.w/lq);
	float lnq=std::log(lq);
	float m=1.0f/lv*ac;
	return quat(a.x*m, a.y*m, a.z*m, lnq);
}

matrix transpose(matrix const& m)
{
	matrix r;
	// aw az ay ax		dx cx bx ax
	// bw bz by bx -->	dy cy by ay
	// cw cz cy cx		dz cz bz az
	// dw dz dy dx		dw cw bw aw
	for(int i=0; i<4; ++i)
		for(int j=0; j<4; ++j)
			r.x[i*4+j]=m.x[j*4+i];
	return r;
}

} // ref

}

