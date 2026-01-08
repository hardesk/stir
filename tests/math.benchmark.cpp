#include <nanobench.h>
#include <doctest.h>

#include <stir/math/math.hpp>
//#include <stir/math_mipp.hpp>
#include <stir/math/ref.hpp>

#include <glm/glm.hpp>
#include <glm/ext/quaternion_float.hpp>

namespace bench = ankerl::nanobench;
using namespace stir;

bench::Rng r;
float f11() { return r.uniform01()*2-1; }
#define rF f11()
matrix rM() { return matrix(rF,rF,rF,rF, rF,rF,rF,rF, rF,rF,rF,rF, rF,rF,rF,rF); }

TEST_CASE("math vec benchmark")
{
	ankerl::nanobench::Bench b;
    b.title("vec3 math")
        .warmup(10)
		.performanceCounters(true)
		.minEpochIterations(50000);

	stir::vec3 v3{rF,rF,rF};
	stir::vec3 v33{rF,rF,rF};
	b.run("vec3::normalize", [&] { vec3 v = normalize(v3); bench::doNotOptimizeAway(v); });
	b.run("vec3 normalize (ref)", [&] { vec3 v=ref::normalize(v3); bench::doNotOptimizeAway(v); });
	//b.run("vec3 normalize (mipp)", [&] { stir::vec3 v = simd_mipp::normalize(v3); bench::doNotOptimizeAway(v); });

	b.run("vec3 cross", [&] { stir::vec3 v=cross(v3, v33); bench::doNotOptimizeAway(v); });
	b.run("vec3 cross (ref)", [&] { stir::vec3 v=ref::cross(v3, v33); bench::doNotOptimizeAway(v); });

	stir::vec4 v4{rF, rF, rF, rF};
	b.run("vec4::normalize", [&] { vec4 v=normalize(v4); bench::doNotOptimizeAway(v); });
	b.run("vec4 normalize (ref)", [&] { vec4 v=ref::normalize(v4); bench::doNotOptimizeAway(v); });
	// b.run("vec4 normalize (neon)", [&] { vec4 v=simd_mipp::normalize(v4); bench::doNotOptimizeAway(v); });
}

/*TEST_CASE("math matrix benchmark")
{
	matrix m{0,1,2,3, 4,5,6,7, 8,9,0,1, 2,3,4,5};
	bench::Bench().minEpochIterations(100000).run("matrix plus float", [&] {
		matrix m1=m*5.0f;
		bench::doNotOptimizeAway(m1);
	});
}*/


TEST_CASE("math quat benchmark")
{
	quat q1(rF,rF,rF,rF);
	quat q2(rF,rF,rF,rF);
    vec3 v3(rF,rF,rF);
    quat qn = normalize(q1);

    ankerl::nanobench::Bench b;
    b.title("quat math")
        .warmup(10)
		.performanceCounters(true)
		.minEpochIterations(50000);

	b.run("quat mul (ref)", [&] { quat q=ref::mul(q1, q2); bench::doNotOptimizeAway(q); });
	b.run("quat mul", [&] { quat q=mul(q1, q2); bench::doNotOptimizeAway(q); });

	b.run("quat rot (ref)", [&] { vec3 v=ref::rot(qn, v3); bench::doNotOptimizeAway(v); });
	b.run("quat rot", [&] { vec3 v=rot(qn, v3); bench::doNotOptimizeAway(v); });

	// bench::Bench().run("quat mul (xsimd)", [&] {
	// 	quat q1=simd_xsimd::mul(q, q2);
	// 	bench::doNotOptimizeAway(q1);
	// });

	// bench::Bench().run("quat mul (sse)", [&] {
	// 	quat q1=sse::mul(q, q2);
	// 	bench::doNotOptimizeAway(q1);
	// });

	// vec3 v(rF,rF,rF);
	// bench::Bench().run("vec3 rot quat", [&] {
	// 	vec3 vr=rot(v, q);
	// 	bench::doNotOptimizeAway(vr);
	// });
	// bench::Bench().run("vec3 rot quat (ref)", [&] {
	// 	vec3 vr=ref::rot(v, q);
	// 	bench::doNotOptimizeAway(vr);
	// });
	// bench::Bench().run("vec3 rot quat (mipp)", [&] {
	// 	vec3 vr=simd_mipp::rot(v, q);
	// 	bench::doNotOptimizeAway(vr);
	// });
}

glm::mat4 toglm(matrix m) {
    glm::mat4 r;
    for (int i=0; i<4; ++i)
        for (int j=0; j<4; ++j)
            r[i][j] = m(i,j);
    return r;
}

glm::vec3 toglm(vec3 v) { return glm::vec3(v[0], v[1], v[2]); }
glm::vec4 toglm(vec4 v) { return glm::vec4(v[0], v[1], v[2], v[3]); }
glm::quat toglm(quat q) { return glm::quat(q[3], q[0], q[1], q[2]); }

TEST_CASE("math matrix benchmark")
{
	matrix m1(1,2,3,4, 5,6,7,8, 9,0,1,2, 3,4,5,6);
	matrix m2(1,1,2,2, 3,3,4,4, 5,5,6,6, 7,7,8,8);
	matrix m(rF,rF,rF,rF, rF,rF,rF,rF, rF,rF,rF,rF,rF,rF,rF,rF);
	matrix mX(rF,rF,rF,rF, rF,rF,rF,rF, rF,rF,rF,rF,rF,rF,rF,rF);
	vec4 v4(rF,rF,rF,rF);
	vec4 v42(rF,rF,rF,rF);
	vec3 v3(rF,rF,rF);
	quat qa = from_axis_angle(vec3(rF,rF,rF),rF);

    glm::mat4  glm_m1 = toglm(m1);
    glm::mat4  glm_m2 = toglm(m2);
    glm::mat4  glm_m = toglm(m);
    glm::mat4  glm_mX = toglm(m);
    glm::quat  glm_qa = toglm(qa);
    glm::vec4  glm_v4 = toglm(v4);
    glm::vec4  glm_v42 = toglm(v4);
    glm::vec3  glm_v3 = toglm(v3);

	ankerl::nanobench::Bench b;
    b.title("matrix math")
        .warmup(10);
    b.performanceCounters(true);
	b.minEpochIterations(50000);

	b.run("matrix mulC",			[&] { matrix m=mul(m1, m2); bench::doNotOptimizeAway(m); });
	b.run("matrix mulC (ref)",	[&] { matrix m=ref::mul(m1, m2); bench::doNotOptimizeAway(m); });
	b.run("matrix mulC (glm)",	[&] { glm::mat4 m=glm_m1*glm_m2; bench::doNotOptimizeAway(m); });
	// b.run("matrix mul (sse)",	[&] { matrix m=sse::mul(m1, m2); bench::doNotOptimizeAway(m); });
	// b.run("matrix mul (mipp)",	[&] { matrix m=simd_mipp::mul(m1, m2); bench::doNotOptimizeAway(m); });

	b.run("matrix mul",			[&] { matrix mm=mul(m, mX); bench::doNotOptimizeAway(mm); });
	b.run("matrix mul (ref)",	[&] { matrix mm=ref::mul(m, mX); bench::doNotOptimizeAway(mm); });
	b.run("matrix mul (glm)",	[&] { glm::mat4 mm=glm_m*glm_mX; bench::doNotOptimizeAway(mm); });

	b.run("matrix transpose",			[&] { matrix mx=transpose(m); bench::doNotOptimizeAway(mx); });
	b.run("matrix transpose (ref)",		[&] { matrix mx=ref::transpose(m); bench::doNotOptimizeAway(mx); });
	b.run("matrix transpose (glm)",	    [&] { glm::mat4 mx=glm::transpose(glm_m); bench::doNotOptimizeAway(mx); });
	// b.run("matrix transpose (sse)",		[&] { matrix mx=sse::transpose(m); bench::doNotOptimizeAway(mx); });
	// b.run("matrix transpose (xsimd)",	[&] { matrix mx=simd_xsimd::transpose(m); bench::doNotOptimizeAway(mx); });
	// b.run("matrix transpose (mipp)",	[&] { matrix mx=simd_mipp::transpose(m); bench::doNotOptimizeAway(mx); });

	b.run("some math (glm)",	[&] {
		glm::vec4 vx = glm_m * glm_v4 - glm_v42*3.0f;
		glm::vec4 vy = glm::vec4(glm_qa * glm_v3 + glm_v3, 2) + glm_v4*7.0f;
		float x = glm::dot(vx, vy);
		bench::doNotOptimizeAway(x); });

	b.run("some math",	[&] {
		vec4 vx = mul(m, v4) - v42*3.0f;
		vec4 vy = vec4(rot(qa, v3) + v3, 2) + v4*7.0f;
		float x = dot(vx, vy);
		bench::doNotOptimizeAway(x); });



	// matrix q(1,0,2,-5,
	// 		 2,5,0,3,
	// 		 -3,4,1,3,
	// 		 0,-2,7,1);
	// b.run("matrix inverse",			[&] { matrix mx=inverse(m); bench::doNotOptimizeAway(mx); });
	// b.run("matrix inverse (ref)",	[&] { matrix mx=ref::inverse(m); bench::doNotOptimizeAway(mx); });
	// b.run("matrix inverse (sse)",	[&] { matrix mx=sse::inverse(m); bench::doNotOptimizeAway(mx); });
}
