#include <stir/math/math.hpp>
#include "doctest.h"

using namespace stir;

TEST_CASE("defs: basic types and constructors") {
    SUBCASE("vec2 constructor and index") {
        vec2 v(1.0f, 2.0f);
        CHECK(v[0] == doctest::Approx(1.0f));
        CHECK(v[1] == doctest::Approx(2.0f));
    }

    SUBCASE("vec3 constructor and index") {
        vec3 v(1.0f, 2.0f, 3.0f);
        CHECK(v[0] == doctest::Approx(1.0f));
        CHECK(v[1] == doctest::Approx(2.0f));
        CHECK(v[2] == doctest::Approx(3.0f));
    }

    SUBCASE("vec4 constructor and normalized") {
        vec4 v(1.0f, 2.0f, 3.0f, 4.0f);
        CHECK(v[0] == doctest::Approx(1.0f));
        CHECK(v[1] == doctest::Approx(2.0f));
        CHECK(v[2] == doctest::Approx(3.0f));
        CHECK(v[3] == doctest::Approx(4.0f));

        vec4 vcopy = v;
        vcopy.normalize();
        // normalization should change values; check length is > 0
        CHECK(len(vcopy) > 0.0f);
    }

    SUBCASE("quat basic accessors") {
        quat q(1.0f, 2.0f, 3.0f, 4.0f);
        CHECK(q[0] == doctest::Approx(1.0f));
        CHECK(q[1] == doctest::Approx(2.0f));
        CHECK(q[2] == doctest::Approx(3.0f));
        CHECK(q[3] == doctest::Approx(4.0f));
        CHECK(q.scalar() == doctest::Approx(4.0f));

        vec3 v3 = q.v3();
        CHECK(v3[0] == doctest::Approx(1.0f));
        CHECK(v3[1] == doctest::Approx(2.0f));
        CHECK(v3[2] == doctest::Approx(3.0f));
    }

    SUBCASE("matrix identity and element access") {
        matrix I = matrix::identity();
        CHECK(I(0,0) == doctest::Approx(1.0f));
        CHECK(I(0,1) == doctest::Approx(0.0f));
        CHECK(I(1,1) == doctest::Approx(1.0f));

        float arr[16];
        for (int i = 0; i < 16; ++i) arr[i] = float(i);
        matrix M(arr);
        // check a couple of elements: M constructed from array uses load by rows
        CHECK(M(0,0) == doctest::Approx(arr[0]));
        CHECK(M(3,3) == doctest::Approx(arr[15]));

        // columns / col access
        vec4 c0(0.0f, 1.0f, 2.0f, 3.0f);
        vec4 c1(4.0f, 5.0f, 6.0f, 7.0f);
        vec4 c2(8.0f, 9.0f, 10.0f, 11.0f);
        vec4 c3(12.0f, 13.0f, 14.0f, 15.0f);
        matrix MC(c0, c1, c2, c3);
        vec4 rc2 = MC.col(2);
        CHECK(rc2[0] == doctest::Approx(8.0f));
        CHECK(rc2[3] == doctest::Approx(11.0f));
    }

    SUBCASE("matrix33 access") {
        matrix33 m33(1,2,3, 4,5,6, 7,8,9);
        CHECK(m33(0,0) == doctest::Approx(1.0f));
        CHECK(m33(2,2) == doctest::Approx(9.0f));
    }
}
