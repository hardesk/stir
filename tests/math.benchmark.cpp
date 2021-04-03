#include <nanobench.h>
#include <doctest/doctest.h>

#include <stir/math.hpp>

namespace bench = ankerl::nanobench;
using namespace stir;

bench::Rng r;
float f11() { return r.uniform01()*2-1; }
#define rF f11()

TEST_CASE("math vec benchmark")
{
	stir::vec3 v3{rF,rF,rF};
	bench::Bench().run("vec3::normalize", [&] {
		v3.normalize();
		bench::doNotOptimizeAway(v3);
	});
	bench::Bench().run("vec3 normalize ref", [&] {
		vec3 v=ref::normalize(v3);
		bench::doNotOptimizeAway(v);
	});
	bench::Bench().run("vec3 normalize", [&] {
		stir::vec3 v = normalize(v3);
		bench::doNotOptimizeAway(v);
	});

	stir::vec4 v4{rF, rF, rF, rF};
	bench::Bench().run("vec4::normalize", [&] {
		v4.normalize();
		bench::doNotOptimizeAway(v4);
	});
	bench::Bench().run("vec4 normalize ref", [&] {
		vec4 v=ref::normalize(v4);
		bench::doNotOptimizeAway(v);
	});
	bench::Bench().run("vec4 normalize", [&] {
		vec4 v=normalize(v4);
		bench::doNotOptimizeAway(v);
	});
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
	quat q(rF,rF,rF,rF);
	quat q2(rF,rF,rF,rF);
	bench::Bench().run("quat mul", [&] {
		quat q1=mul(q, q2);
		bench::doNotOptimizeAway(q1);
	});

	bench::Bench().run("quat mul ref", [&] {
		quat q1=ref::mul(q, q2);
		bench::doNotOptimizeAway(q1);
	});

	vec3 v(rF,rF,rF);
	bench::Bench().run("vec3 rot quat", [&] {
		vec3 vr=rot(v, q);
		bench::doNotOptimizeAway(vr);
	});
}
