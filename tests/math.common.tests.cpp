#include <stir/math/math.hpp>
#include <stir/math/format.hpp>
#include "doctest.h"

using namespace stir;

TEST_CASE("common functions: operators, dot/cross, normalize, quat, matrix") {
    SUBCASE("equality operators for vectors and quat/matrix") {
        vec2 a2(1.0f, 2.0f), b2(1.0f, 2.0f);
        CHECK(a2 == b2);

        vec3 a3(1.0f, 2.0f, 3.0f), b3(1.0f, 2.0f, 3.0f);
        CHECK(a3 == b3);

        vec4 a4(1.0f, 2.0f, 3.0f, 4.0f), b4(1.0f, 2.0f, 3.0f, 4.0f);
        CHECK(a4 == b4);

        quat q1(1,2,3,4), q2(1,2,3,4);
        CHECK(q1 == q2);

        matrix I = matrix::identity();
        matrix J = matrix::identity();
        CHECK(I == J);
    }

    SUBCASE("dot and cross products") {
        vec3 u(1.0f, 0.0f, 0.0f);
        vec3 v(0.0f, 1.0f, 0.0f);
        CHECK(dot(u, v) == doctest::Approx(0.0f));
        vec3 w = cross(u, v);
        printf("cross(u,v) = (%f, %f, %f)\n", w[0], w[1], w[2]);
        CHECK(w[0] == doctest::Approx(0.0f));
        CHECK(w[1] == doctest::Approx(0.0f));
        CHECK(w[2] == doctest::Approx(1.0f));
    }

    SUBCASE("vector arithmetic and lengths") {
        vec3 a(1.0f, 2.0f, 3.0f);
        vec3 b(4.0f, -1.0f, 0.5f);
        vec3 add = a + b;
        CHECK(add[0] == doctest::Approx(5.0f));
        CHECK(add[1] == doctest::Approx(1.0f));
        CHECK(add[2] == doctest::Approx(3.5f));

        vec3 sub = a - b;
        CHECK(sub[0] == doctest::Approx(-3.0f));

        vec3 muls = a * 2.0f;
        CHECK(muls[0] == doctest::Approx(2.0f));
        CHECK(length(a) == doctest::Approx(std::sqrt(14.0f)));

        vec3 an = normalize(a);
        CHECK(length(an) == doctest::Approx(1.0f));
    }

    SUBCASE("quat operations: conj, inverse, multiply") {
        quat q(1.0f, 2.0f, 3.0f, 4.0f);
        quat qc = conj(q);
        // conj should flip vector part sign
        CHECK(qc[0] == doctest::Approx(-1.0f));
        CHECK(qc.scalar() == doctest::Approx(4.0f));
        CHECK(qc[3] == doctest::Approx(4.0f));

        // quat invq = inverse(q);
        // // q * inverse(q) should produce (approximately) identity quaternion scaled
        // quat prod = mul(q, invq);
        // // Normalizing prod to compare with identity
        // quat pnorm = normalize(prod);
        quat ide = quat::identity();
        // CHECK(pnorm == ApproxQuat(ide));

        quat qx = mul(q, ide);
        CHECK(qx == q);
    }

    SUBCASE("matrix operations: mul, transpose, mul with vec3/vec4") {
        matrix I = matrix::identity();
        // create a simple translation-like matrix using columns
        vec4 c0(1,0,0,0);
        vec4 c1(0,1,0,0);
        vec4 c2(0,0,1,0);
        vec4 c3(10,20,30,1);
        matrix T(c0, c1, c2, c3);

        matrix TT = mul(I, T);
        CHECK(TT == T);

        matrix Tr = transpose(T);
        CHECK(Tr(0,3) == doctest::Approx(T(3,0)));

        vec3 p(1.0f, 2.0f, 3.0f);
        vec3 pout = xform_pos(T, p);
        // Because T has translation in column 3, xform_pos(T,p) should add translation
        CHECK(pout[0] == doctest::Approx(1.0f + 10.0f));
        CHECK(pout[1] == doctest::Approx(2.0f + 20.0f));
        CHECK(pout[2] == doctest::Approx(3.0f + 30.0f));

        // directions should not be affected by translation
        vec3 dir(1.0f, 0.0f, 0.0f);
        vec3 d_out = xform_dir(T, dir);
        CHECK(d_out[0] == doctest::Approx(dir[0]));
        CHECK(d_out[1] == doctest::Approx(dir[1]));
        CHECK(d_out[2] == doctest::Approx(dir[2]));

        vec4 pvout = xform_pos4(T, p);
        CHECK(pvout[0] == doctest::Approx(1.0f + 10.0f));
    }
}
