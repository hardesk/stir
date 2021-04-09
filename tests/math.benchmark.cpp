#include <nanobench.h>
#include <doctest/doctest.h>

#include <stir/math.hpp>
#include <stir/math_mipp.hpp>
#include <stir/math_sse.hpp>
#include <stir/math_xsimd.hpp>

namespace bench = ankerl::nanobench;
using namespace stir;

bench::Rng r;
float f11() { return r.uniform01()*2-1; }
#define rF f11()
matrix rM() { return matrix(rF,rF,rF,rF, rF,rF,rF,rF, rF,rF,rF,rF, rF,rF,rF,rF); }


quat __attribute__((noinline)) do_math_quat_sse(quat const& a)
{
	quat q=sse::mul(a,quat(2,1,3,0.2f));
	q = sse::mul(a,a) + q / 4.f;
	return q;
}

matrix __attribute__((noinline)) do_math1_matrix(matrix const& a)
{
	matrix mm=rM();
	return simd_mipp::mul(mm, simd_mipp::transpose(a));
}

matrix __attribute__((noinline)) do_math1_matrix_mipp(matrix const& a)
{
	matrix mm=matrix::identity();
	return simd_mipp::mul(mm, a);
}

matrix __attribute__((noinline)) do_math1_matrix_sse(matrix const& a)
{
	matrix mm=matrix::identity();
	return sse::mul(mm, a);
}

matrix __attribute__((noinline)) do_math1_matrix_xsimd(matrix const& a)
{
	matrix mm=matrix::identity();
	return simd_xsimd::mul(mm, a);
}


matrix __attribute__((noinline)) do_math1_matrix_transpose_ref(matrix const& a)
{
	return ref::transpose(a);
}

matrix __attribute__((noinline)) do_math1_matrix_transpose_mipp(matrix const& a)
{
	return simd_mipp::transpose(a);
}

matrix __attribute__((noinline)) do_math1_matrix_transpose_sse(matrix const& a)
{
	return sse::transpose(a);
}

matrix __attribute__((noinline)) do_math1_matrix_transpose_xsimd(matrix const& a)
{
	return simd_xsimd::transpose(a);
}


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

	bench::Bench().run("quat mul ref", [&] {
		quat q1=ref::mul(q, q2);
		bench::doNotOptimizeAway(q1);
	});

	bench::Bench().run("quat mul", [&] {
		quat q1=mul(q, q2);
		bench::doNotOptimizeAway(q1);
	});

	bench::Bench().run("quat mul (xsimd)", [&] {
		quat q1=simd_xsimd::mul(q, q2);
		bench::doNotOptimizeAway(q1);
	});

	bench::Bench().run("quat mul (sse)", [&] {
		quat q1=sse::mul(q, q2);
		bench::doNotOptimizeAway(q1);
	});

	vec3 v(rF,rF,rF);
	bench::Bench().run("vec3 rot quat", [&] {
		vec3 vr=rot(v, q);
		bench::doNotOptimizeAway(vr);
	});
}

TEST_CASE("math matrix benchmark")
{
	matrix m1(1,2,3,4, 5,6,7,8, 9,0,1,2, 3,4,5,6);
	matrix m2(1,1,2,2, 3,3,4,4, 5,5,6,6, 7,7,8,8);
	matrix m(rF,rF,rF,rF, rF,rF,rF,rF, rF,rF,rF,rF,rF,rF,rF,rF);

	bench::Bench().run("matrix mul", [&] {
		matrix m=mul(m1, m2);
		bench::doNotOptimizeAway(m);
	});

	bench::Bench().run("matrix mul (ref)", [&] {
		matrix m=ref::mul(m1, m2);
		bench::doNotOptimizeAway(m);
	});

	bench::Bench().run("matrix mul (sse)", [&] {
		matrix m=sse::mul(m1, m2);
		bench::doNotOptimizeAway(m);
	});

	bench::Bench().run("matrix transpose",			[&] { matrix m=transpose(m); bench::doNotOptimizeAway(m); });
	bench::Bench().run("matrix transpose (ref)",	[&] { matrix m=ref::transpose(m); bench::doNotOptimizeAway(m); });
	bench::Bench().run("matrix transpose (sse)",	[&] { matrix m=sse::transpose(m); bench::doNotOptimizeAway(m); });
	bench::Bench().run("matrix transpose (xsimd)",	[&] { matrix m=simd_xsimd::transpose(m); bench::doNotOptimizeAway(m); });
	bench::Bench().run("matrix transpose (mipp)",	[&] { matrix m=simd_mipp::transpose(m); bench::doNotOptimizeAway(m); });



}
