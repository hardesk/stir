#include <stir/math/math.hpp>
#include <stir/math/ref.hpp>
#include <stir/math/format.hpp>
//#include <stir/math_sse.hpp>
//#include <stir/math_fmt.hpp>

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

// int operator<=>(stir::quat const& q, doctest::Approx const& a) {
// 	return 0;
// }

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
*/

struct f4_t {
    static f4_t decompose(simd::impl::F4 a) {
        return f4_t{
            simd::impl::elem_4f(a, 0),
            simd::impl::elem_4f(a, 1),
            simd::impl::elem_4f(a, 2),
            simd::impl::elem_4f(a, 3)};
    }
    float x,y,z,w;
};

template<class T>
struct Approx1 {
    T m_value;
    Approx1(T a) : m_value(a) {}
    bool operator==(T const& a) {
        auto [x,y,z,w] = f4_t{ a[0], a[1], a[2], a[3] };//f4_t::decompose(a.v);
        doctest::Approx xx[4] = {m_value[0], m_value[1], m_value[2], m_value[3]};
        return x == xx[0] && y == xx[1] && z == xx[2] && w == xx[3];
    }
};

struct ApproxQuat : Approx1<quat> {
    ApproxQuat(quat q) : Approx1<quat>(q) {}
    //bool compare_component();
};

struct ApproxV3 : Approx1<vec3> {
    ApproxV3(vec3 v) : Approx1<vec3>(v) {}
    //bool compare_component();
};

template<class T>
String toString(const Approx1<T>& in) {
    return "Approx( " + doctest::toString(in.m_value) + " )";
}
String toString(const ApproxQuat& in) {
    // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
    return String("ApproxQ( ") + doctest::toString(in.m_value) + " )";
}
String toString(const ApproxV3& in) {
    // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
    return String("ApproxV3( ") + doctest::toString(in.m_value) + " )";
}

TEST_CASE("quat")
{
    quat q1234(1,2,3,4);
    quat q0_5(0.5f, 0.5f, 0.5f, 0.5f);
    quat q1(1,0,0,0);
    quat q11(1,1,1,1);
    quat q_i = quat::identity();
    quat q = from_axis_angle(vec3(0.5f,2.0f,1.0f), pi/2);
    quat q_axisangle = quat(0.154303f, 0.617213f, 0.308607f, 0.707106f); // calculated with external calculator
    vec3 naxis = vec3(0.218218f, 0.872872f, 0.436436f); // precalculated vector axis
    quat q_c = quat(1.0f, 2.0f, 3.0f, 4.0f);
    quat q_n = quat(0.1825742f, 0.3651484f, 0.5477226f, 0.7302967f);

    SUBCASE("axis") {
        CHECK(q == ApproxQuat(q_axisangle));
        CHECK(q.axis() == ApproxV3(naxis));
        CHECK(q.scalar() == Approx(0.7071068f));
    }

    SUBCASE("mul")
    {
        CHECK(q1234 == mul(q1234, q_i));

        quat qref = ref::mul(q, q1234);
        quat qm = mul(q, q1234);
        CHECK(qm == ApproxQuat(qref));

        CHECK(mul(q, q1234) == ApproxQuat(qref));

        // SUBCASE("sse")
        // {
        // 	quat qm_sse=stir::sse::mul(q, q1);
        // 	CHECK(qref == qm_sse);
        // 	//CHECK(mul_q_sse(q, q1) == qref);
        // }
    }
    
    SUBCASE("length")
    {
        CHECK(ref::length(q11) == 2.0f);

        CHECK(q11 == quat(1,1,1,1));
        CHECK(length(q11) == Approx(2.0f));
        CHECK(length(q1234) == Approx(5.47722f));
        CHECK(length(q1) == Approx(1.0f));
        CHECK(length(q_n) == Approx(1.0));
    }

    SUBCASE("normalization")
    {
        CHECK(normalize(q1) == ApproxQuat(q1));
        CHECK(normalize(q11) == ApproxQuat(q0_5));
        CHECK(ref::normalize(q11) == ApproxQuat(q0_5));//quat(0.5,0.5,0.5,0.5));
        CHECK(q0_5 == ref::normalize(q11));//quat(0.5,0.5,0.5,0.5));

        CHECK(length(q_n - q_n) == Approx(0));
        CHECK(length(normalize(q_n) - q_n) == Approx(0));
        CHECK(length(normalize(q_c) - q_n) == Approx(0));

        quat a = q_c;
        a.normalize();
        CHECK(length(a - q_n) == Approx(0));
    }

    SUBCASE("rot vec3")
    {
        // identity quaternion rotates vector unchanged
        quat q_i = quat::identity();
        vec3 v(1.0f, 0.0f, 0.0f);
        vec3 v_rot = rot(q_i, v);
        CHECK(v_rot == ApproxV3(v));

        // 90 degree rotation around z-axis
        quat q_z90 = from_axis_angle(vec3(0.0f, 0.0f, 1.0f), stir::pi / 2.0f);
        vec3 x_axis(1.0f, 0.0f, 0.0f);
        vec3 x_rotated = rot(q_z90, x_axis);
        // Should rotate x-axis toward y-axis
        CHECK(x_rotated == ApproxV3(vec3(0.0f, 1.0f, 0.0f)));
 
        // 180 degree rotation around x-axis
        quat q_x180 = from_axis_angle(vec3(1.0f, 0.0f, 0.0f), 3.14159f);
        vec3 y_axis(0.0f, 1.0f, 0.0f);
        vec3 y_rotated = rot(q_x180, y_axis);
        // Should flip y-axis to negative y
        CHECK(y_rotated[0] == Approx(0.0f));
        CHECK(y_rotated[1] == Approx(-1.0f));
        CHECK(y_rotated[2] == Approx(0.0f));
        CHECK(y_rotated[3] == Approx(0.0f));
    } 

    // SUBCASE("exp log") {
    // 	CHECK(ref::exp(q1234)==quat(-8.240026f, -16.480053f,  -24.72008f,  -45.0598f));
    // 	CHECK(ref::log(q1234)==quat( 0.20099117f, 0.40198234f,  0.6029735f,  1.7005987f));
    // }
    //norm_q(q);
    //printf("%d", &q);
}


TEST_CASE("vec2")
{
    vec2 v(1,2);
}


// stir::matrix __attribute__((noinline)) mytestM_V_(matrix a, matrix b) { return mul(a, mul(a, b)); }
// stir::matrix __attribute__((noinline)) mytestM_V1(matrix a, matrix b) { return mul1(a, mul1(a, b)); }
// stir::matrix __attribute__((noinline)) mytestM_R_(matrix const& a, matrix const& b) { return mul(a, mul(a, b)); }
// stir::matrix __attribute__((noinline)) mytestM_R1(matrix const& a, matrix const& b) { return mul1(a, mul1(a, b)); }

// float32x4_t __attribute__((noinline)) mytestSS_1(float32x4_t a) { return simd::swizz<0,0,0,0>::get4(a); }
// float32x4_t __attribute__((noinline)) mytestSS_2(float32x4_t a) { return simd::swizz<0,0,2,2>::get4(a); }
// float32x4_t __attribute__((noinline)) mytestSS_3(float32x4_t a) { return simd::swizz<1,2,3,0>::get4(a); }

TEST_CASE("vec3")
{
    vec3 v(1,3,4);
    SUBCASE("eq") {
        vec3 v1(1,3,4);
        CHECK(v==v1);
    }
    // vec3 q = v.xxx() + vec3(3,3,3);
    // printf("%f %f %f\n", q[0], q[1], q[2]);
    // SUBCASE("not eq") {
    // 	vec3 v1(1,3,4);
    // 	CHECK(v!=v1);
    // }
    // SUBCASE("normalization") {
    // 	vec3 nn(0, 0.6f, 0.8f);
    // 	v.normalize();
    // 	CHECK(v==nn);
    // 	//CHECK(v.normalized().as_simd()==nn.as_simd());
    // 	CHECK(normalize(v)==nn);
    // }
    // SUBCASE("add") {
    // 	vec3 x=v+vec3(1.0f,-1.0f,3.0f);
    // 	CHECK(x==vec3(1,2,7));
    // 	//x+=v;
    // 	//CHECK(x==vec3(1,5,11));
    // }
    // SUBCASE("sub") {
    // 	vec3 x=v-vec3(2.0f,1.0f,-4.0f);
    // 	CHECK(x==vec3(-2,2,8));
    // 	//x-=v;
    // 	//CHECK(x==vec3(-2,-1,-4));
    // }
}

/*TEST_CASE("vec4")
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
*/

TEST_CASE("matrix")
{
    matrix i = matrix::identity();

    matrix ii = mul(i, i);
    CHECK(ii == i);

    matrix ti = transpose(i);
    CHECK(ti == i);

    vec3 v1(1.0f);
    vec3 v11 = xform_pos(ti, v1);
    CHECK(v11 == v1);

    // SUBCASE("inverse (ref)") {
    // 	matrix ii=ref::inverse_cofactor(i);
    // 	CHECK(i==ii);

    // 	matrix q(1,0,2,-5,
    // 			 2,5,0,3,
    // 			 -3,4,1,3,
    // 			 0,-2,7,1);
    // 	matrix iq(
    // 		-0.01085776330076004341,0.18458197611292073832,-0.21389793702497285558,0.033659066232356134638,
    // 		0.11183496199782844734,0.098805646036916395212,0.10314875135722041259,-0.04668838219326818676,
    // 		0.05754614549402823019,0.021715526601520086857,0.033659066232356134636,0.12160694896851248643,
    // 		-0.17915309446254071661,0.045602605863192182412,-0.029315960912052117264,0.055374592833876221501
    // 	);

    // 	matrix iqq=ref::inverse(q);
    // 	CHECK(iq==iqq);

    // 	matrix iqq2=sse::inverse(q);
    // 	CHECK(iq==iqq2);

    // 	matrix iqq3=simd_mipp::inverse(q);
    // 	CHECK(iq==iqq3);
    // }
}
