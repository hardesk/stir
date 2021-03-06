#include <stir/math.hpp>
#include <stir/math_sse.hpp>
#include <stir/math_fmt.hpp>


#include "doctest/doctest.h"


int operator<=>(stir::quat const& q, doctest::Approx const& a) {
	return 0;
}

using namespace stir;
using namespace doctest;

/*
TEST_CASE("Matrix")
{
	stir::Matrix m = stir::Matrix::identity();
	
	SUBCASE("identity is identity")
	{
		CHECK(m(0,0)==1);
		CHECK(m(0,1)==0);
		CHECK(m(0,2)==0);
		CHECK(m(0,3)==0);

		CHECK(m(1,0)==0);
		CHECK(m(1,1)==1);
	}
}

Matrix mul_m1(Matrix const& a, Matrix const& b)
{
	return stir::mul_sse(a, b);
}


Matrix mul_m2(Matrix const& a, Matrix const& b)
{
	return stir::mul_xsimd(a, b);
}

Matrix mul_m3(Matrix const& a, Matrix const& b)
{
	return stir::mul_xsimd_old(a, b);
}


TEST_CASE("Matrix mul")
{
	Matrix m1, m2;
}

Quaternion conj_1(Quaternion q)
{
	return q.conj();
}

*/
quat __attribute__((noinline)) mul_q(quat const& a, quat const& b)
{
	return stir::mul(a, b);
}

quat __attribute__((noinline)) mul_q_sse(quat const& a, quat const& b)
{
	return stir::sse::mul(a, b);
}

quat __attribute__((noinline)) norm_q(quat& a)
{
	quat qq=normalize(a);
	return qq * qq;
}

TEST_CASE("quat")
{
	quat q1234(1,2,3,4);
	quat q1(1,0,0,0);
	quat q11(1,1,1,1);
	quat q_i = quat::identity();
	quat q = from_axis_angle(vec3(0.5f,2.0f,1.0f), 3.1415f/2);
	quat q_c = quat(1.0f, 2.0f, 3.0f, 4.0f);
	quat q_n = quat(0.1825742f, 0.3651484f, 0.5477226f, 0.7302967f);

	SUBCASE("mul")
	{
		quat q1(1,2,3,4);

		quat qref = ref::mul(q, q1);
		quat qm = mul(q, q1);
		CHECK(qref == qm);

		CHECK(mul_q(q, q1) == qref);

		SUBCASE("sse")
		{
			quat qm_sse=stir::sse::mul(q, q1);
			CHECK(qref == qm_sse);
			//CHECK(mul_q_sse(q, q1) == qref);
		}
	}
	
	SUBCASE("length")
	{
		CHECK(length(q1) == 1.0f);
		CHECK(length(q11) == 2.0f);
		CHECK(length(q_n) == Approx(1.0));
	}

	SUBCASE("normalization")
	{
		CHECK(normalize(q1) == q1);
		CHECK(normalize(q11) == quat(0.5f, 0.5f, 0.5f, 0.5f));

		CHECK(length(q_n - q_n) == Approx(0));
		CHECK(length(normalize(q_n) - q_n) == Approx(0));
		CHECK(length(normalize(q_c) - q_n) == Approx(0));

		quat a = q_c;
		a.normalize();
		CHECK(length(a - q_n) == Approx(0));
	}

	SUBCASE("exp log") {
		CHECK(ref::exp(q1234)==quat(-8.240026f, -16.480053f,  -24.72008f,  -45.0598f));
		CHECK(ref::log(q1234)==quat( 0.20099117f, 0.40198234f,  0.6029735f,  1.7005987f));
	}
	norm_q(q);
	//printf("%d", &q);
}

/*
Vector4 add_2(Vector4 const& a, Vector4 const& b)
{
	return a + b;
}

bool mul_2(Vector4 const& a)
{
	Matrix mi = Matrix::identity();
	Vector4 b = mul(a, mi);
	Vector4 c = add_2(a, b);
	return c == Vector4(2,4,6,8);
}


TEST_CASE("mul")
{
	Matrix mi = Matrix::identity();
	Vector4 v = {1,2,3,4};

	Vector4 v1 = mul(v, mi);
	CHECK(v == Vector4(1,2,3,4));
}
*/

TEST_CASE("vec2")
{
	vec2 v(1,2);

}

TEST_CASE("vec3")
{
	vec3 v(0,3,4);
	SUBCASE("eq") {
		vec3 v1(0,3,4);
		CHECK(v==v1);
	}
	SUBCASE("not eq") {
		vec3 v1(1,3,4);
		CHECK(v!=v1);
	}
	SUBCASE("normalization") {
		vec3 nn(0, 0.6f, 0.8f);
		v.normalize();
		CHECK(v==nn);
		//CHECK(v.normalized().as_simd()==nn.as_simd());
		CHECK(normalize(v)==nn);
	}
	SUBCASE("add") {
		vec3 x=v+vec3(1.0f,-1.0f,3.0f);
		CHECK(x==vec3(1,2,7));
		//x+=v;
		//CHECK(x==vec3(1,5,11));
	}
	SUBCASE("sub") {
		vec3 x=v-vec3(2.0f,1.0f,-4.0f);
		CHECK(x==vec3(-2,2,8));
		//x-=v;
		//CHECK(x==vec3(-2,-1,-4));
	}
}

TEST_CASE("vec4")
{
	vec4 v(0,3,4, 0);
	SUBCASE("normalization") {
		vec4 nn(0, 0.6f, 0.8f, 0);
		v.normalize();
		CHECK(v==nn);
		//CHECK(v.normalized().as_simd()==nn.as_simd());
		CHECK(normalize(v)==nn);
	}
}




