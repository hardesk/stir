#include <cmath>
#include "math.hpp"

namespace stir {

// #if STIR_MATH_MODE == STIR_MATH_REF
// inline
// #endif
namespace ref
{

quat mul(quat const& a, quat const& b)
{
    float x = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
    float y = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
    float z = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
	float w = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];

	return quat(x, y, z, w);
}

matrix mul(matrix const& a, matrix const& b)
{
	matrix r;
	for (int i=0; i<4; ++i) {
		vec4 col = {
			a(0,0)*b(0,i) + a(0,1)*b(1,i) + a(0,2)*b(2,i) + a(0,3)*b(3,i),
			a(1,0)*b(0,i) + a(1,1)*b(1,i) + a(1,2)*b(2,i) + a(1,3)*b(3,i),
			a(2,0)*b(0,i) + a(2,1)*b(1,i) + a(2,2)*b(2,i) + a(2,3)*b(3,i),
			a(3,0)*b(0,i) + a(3,1)*b(1,i) + a(3,2)*b(2,i) + a(3,3)*b(3,i)
		};
		r.set(i, col);
	}
	return r;
}

vec4 mul(matrix const& m, vec4 const& v)
{
	float v0 = m(0,0)*v[0] + m(0,1)*v[1] + m(0,2)*v[2] + m(0,3)*v[3];
	float v1 = m(1,0)*v[0] + m(1,1)*v[1] + m(1,2)*v[2] + m(1,3)*v[3];
	float v2 = m(2,0)*v[0] + m(2,1)*v[1] + m(2,2)*v[2] + m(2,3)*v[3];
	float v3 = m(3,0)*v[0] + m(3,1)*v[1] + m(3,2)*v[2] + m(3,3)*v[3];
	return vec4(v0, v1, v2, v3);
}

vec3 xform_pos(matrix const& m, vec3 const& v)
{
	float v0 = m(0,0)*v[0] + m(0,1)*v[1] + m(0,2)*v[2] + m(0,3);
	float v1 = m(1,0)*v[0] + m(1,1)*v[1] + m(1,2)*v[2] + m(1,3);
	float v2 = m(2,0)*v[0] + m(2,1)*v[1] + m(2,2)*v[2] + m(2,3);
	return vec3(v0, v1, v2);
}

vec4 xform_pos4(matrix const& m, vec3 const& v)
{
	float v0 = m(0,0)*v[0] + m(0,1)*v[1] + m(0,2)*v[2] + m(0,3);
	float v1 = m(1,0)*v[0] + m(1,1)*v[1] + m(1,2)*v[2] + m(1,3);
	float v2 = m(2,0)*v[0] + m(2,1)*v[1] + m(2,2)*v[2] + m(2,3);
	float v3 = m(3,0)*v[0] + m(3,1)*v[1] + m(3,2)*v[2] + m(3,3);
	return vec4(v0, v1, v2, v3);
}

vec3 xform_dir(matrix const& m, vec3 const& v)
{
	float v0 = m(0,0)*v[0] + m(0,1)*v[1] + m(0,2)*v[2];
	float v1 = m(1,0)*v[0] + m(1,1)*v[1] + m(1,2)*v[2];
	float v2 = m(2,0)*v[0] + m(2,1)*v[1] + m(2,2)*v[2];
	return vec3(v0, v1, v2);
}

vec3 mul(matrix33 const& m, vec3 const& v)
{
	float v0 = m(0,0)*v[0] + m(0,1)*v[1] + m(0,2)*v[2];
	float v1 = m(1,0)*v[0] + m(1,1)*v[1] + m(1,2)*v[2];
	float v2 = m(2,0)*v[0] + m(2,1)*v[1] + m(2,2)*v[2];
	return vec3(v0, v1, v2);
}


matrix rot_matrix(quat const& q)
{
#if 1
	// http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/jay.htm
	matrix m1{
		  q[3],  q[2], -q[1], q[0],
		 -q[2],  q[3],  q[0], q[1],
		  q[1], -q[0],  q[3], q[2],
		 -q[0], -q[1], -q[2], q[3]
	};

	matrix m2{
		 q[3],  q[2], -q[1], -q[0],
		-q[2],  q[3],  q[0], -q[1],
		 q[1], -q[0],  q[3], -q[2],
		 q[0],  q[1],  q[2],  q[3]
	};

	return ref::mul(m1, m2);
#else
	float xx = q[0] * q[0];
    float xy = q[0] * q[1];
    float xz = q[0] * q[2];
    float xw = q[0] * q[3];

    float yy = q[1] * q[1];
    float yz = q[1] * q[2];
    float yw = q[1] * q[3];

    float zz = q[2] * q[2];
    float zw = q[2] * q[3];

	return matrix{
		{ 1 - 2 * ( yy + zz ),	2 * ( xy - zw ),		2 * ( xz + yw ),	0 },
		{ 2 * ( xy + zw ), 		1 - 2 * ( xx + zz ), 	2 * ( yz - xw ),	0 },
		{ 2 * ( xz - yw ),		2 * ( yz + xw ), 		1 - 2 * ( xx + yy ),0 },
		{ 0, 					0,						0,					1 }
	};
#endif
}

quat exp(quat const& a)
{
	// exp(q)=e^w * (cos|v| + v/|v|*sin|v|)
	float ew=std::exp(a.scalar());
	float l=ref::len(a.v3());
	float cv=std::cosf(l);
	float sv=std::sinf(l);
	float m=ew/l;
	return quat(a[0]*m*sv, a[1]*m*sv, a[2]*m*sv, ew*cv);
}

quat log(quat const& a)
{
	// ln(q)=ln|q|+v/|v|arccos(a/|q|)
	float lq=ref::len(a);
	float lv=ref::len(a.v3());
	float ac=std::acos(a[3]/lq);
	float lnq=std::log(lq);
	float m=1.0f/lv*ac;
	return quat(a[0]*m, a[1]*m, a[2]*m, lnq);
}

bool operator==(matrix const& a, matrix const& b)
{
    #if 0
	for(int i=0; i<matrix::N; ++i)
		if(a.el(i) != b.el(i)) return false;
    #else
	for(int i=0; i<4; ++i)
		if(a.col(i) != b.col(i)) return false;
    #endif
	return true;
}

matrix transpose(matrix const& m)
{
	matrix r;
	// aw az ay ax		dx cx bx ax
	// bw bz by bx -->	dy cy by ay
	// cw cz cy cx		dz cz bz az
	// dw dz dy dx		dw cw bw aw
	for(int i=0; i<4; ++i)
		r.set(i, m(i, 0), m(i, 1), m(i, 2), m(i, 3));
	return r;
}

// matrix inverse_cofactor(matrix const& m)
// {
// 	auto Cij=[](matrix const& m, int i, int j) -> float {

// 		float mm[3*4];
// 		for(int q=0, s=0; s<4; ++s) {
// 			if (s==i) continue;
// 			for(int p=0, t=0; t<4; ++t) {
// 				if (t==j) continue;
// 				mm[q*4+p]=m.x[s*4+t];
// 				++p;
// 			}
// 			++q;
// 		}
// #define x(q,r) (mm[(q)*4+r])
// 		//	00 01 02
// 		//	10 11 12
// 		//	20 21 22
// //#define c(q,r) (mm[q*4+(r)%3])
// //	a[i]=x(0,0+i)*x(1,1+i)*x(2,2+i);
// //	float a[3],b[3];
// //	for(int i=0;i<3;++i)
// //	{
// //		a[i]=x(0,0+i)*x(1,1+i)*x(2,2+i);
// //		b[i]=x(2,0+i)*x(1,1+i)*x(0,2+i);
// //	}

// 		float a=x(0,0)*x(1,1)*x(2,2);
// 		float b=x(0,1)*x(1,2)*x(2,0);
// 		float c=x(0,2)*x(1,0)*x(2,1);

// 		float d=x(2,0)*x(1,1)*x(0,2);
// 		float e=x(2,1)*x(1,2)*x(0,0);
// 		float f=x(2,2)*x(1,0)*x(0,1);

// 		float s=a+b+c-d-e-f;
// 		s=((i+j)&1) ? -s : s;
// 		return s;
// 	};

// 	// A-1 = 1/detA * transpose(C) ; C-cofactor matrix
// 	// C=-1^(i+j)*Mij; Mij=det(Axxij)
	
// 	float C[4*4];
// 	for(int i=0; i<4; ++i)
// 		for(int j=0; j<4; ++j)
// 			C[i*4+j]=Cij(m, i, j);
	
// 	float detA=m.x[0]*C[0] + m.x[1]*C[1] + m.x[2]*C[2] + m.x[3]*C[3];
// 	float rdetA=1.0f/detA;

// 	matrix mm;
// 	for(int i=0; i<4; ++i)
// 		for(int j=0; j<4; ++j)
// 			mm.x[i*4+j]=rdetA*C[j*4+i];
// 	return mm;
// }

// matrix inverse(matrix const& m)
// {
// 	struct M2 {
// 		float a,b,c,d;
// 		M2 i() const { float det=1.0f/(a*d-b*c); return M2{d*det, -b*det, -c*det, a*det}; }

// 		M2 operator*(M2 const& m) const { return M2{a*m.a+b*m.c, a*m.b+b*m.d, c*m.a+d*m.c, c*m.b+d*m.d}; }
// 		M2 operator+(M2 const& m) const { return M2{a+m.a, b+m.b, c+m.c, d+m.d}; }
// 		M2 operator-(M2 const& m) const { return M2{a-m.a, b-m.b, c-m.c, d-m.d}; }
// 		M2 operator-() const { return M2{-a, -b, -c, -d}; }
// 	};

// #define S(x0,y0) m.x[(x0+0)*4+(y0+0)], m.x[(x0+0)*4+(y0+1)], m.x[(x0+1)*4+(y0+0)], m.x[(x0+1)*4+(y0+1)]

// 	M2 a11{S(0,0)}, a12{S(0,2)}, a21{S(2,0)}, a22{S(2,2)};

// 	M2 a11i=a11.i();
// 	M2 a22i=a22.i();
	
// 	M2 u11=(a11-a12*a22i*a21).i();
// 	M2 u12=-a11i*a12*(a22-a21*a11i*a12).i();
// 	M2 u21=-a22i*a21*(a11-a12*a22i*a21).i();
// 	M2 u22=(a22-a21*a11i*a12).i();

// 	return matrix(u11.a, u11.b, u12.a, u12.b,
// 				  u11.c, u11.d, u12.c, u12.d,
// 				  u21.a, u21.b, u22.a, u22.b,
// 				  u21.c, u21.d, u22.c, u22.d);
// }

} // ref

}

