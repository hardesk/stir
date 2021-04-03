#include <iostream>
#include <random>
#include <vector>
#include <fmt/format.h>
#include <fmt/ostream.h>
#include <lua/lua.h>
#include "stir/math.hpp"

//#include <spdlog/spdlog.h>


#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest/doctest.h"

std::pair<bool, int> run_tests(int argc, char** argv)
{
	doctest::Context context;

    // defaults
//    context.addFilter("test-case-exclude", "*math*"); // exclude test cases with "math" in their name
    context.setOption("abort-after", 5);              // stop test execution after 5 failed assertions
    context.setOption("order-by", "name");            // sort the test cases by their name

    context.applyCommandLine(argc, argv);

    // overrides
    context.setOption("no-breaks", true);             // don't break in the debugger when assertions fail

    int res = context.run(); // run
	return std::make_pair(context.shouldExit(), res);
}

#if 0
TEST_CASE("main") { printf("hello from <main.cpp>\n"); }


TEST_CASE("matrix")
{
	stir::Matrix m2 = stir::Matrix::identity() * 2;
	stir::Matrix m1 = stir::Matrix::identity();

	stir::Matrix ref = stir::mul_ref(m1, m2);
	stir::Matrix impl = stir::mul(m1, m2);

	for (int i=0; i<16; ++i)
		CHECK(ref(i/4, i%4) == impl(i/4, i%4));

}


template <>
struct fmt::formatter<stir::Matrix> {
  template <typename ParseContext>
  constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const stir::Matrix &m, FormatContext &ctx) {
    return format_to(ctx.out(), "{{ {} {} {} {}  {} {} {} {}  {} {} {} {}  {} {} {} {} }}",
			m.el[0][0], m.el[0][1], m.el[0][2], m.el[0][3], 
			m.el[1][0], m.el[1][1], m.el[1][2], m.el[1][3], 
			m.el[2][0], m.el[2][1], m.el[2][2], m.el[2][3], 
			m.el[3][0], m.el[3][1], m.el[3][2], m.el[3][3]);
  }
};


template <>
struct fmt::formatter<stir::Matrix1> {
  template <typename ParseContext>
  constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const stir::Matrix1 &m, FormatContext &ctx) {
    return format_to(ctx.out(), "{{ {} {} {} {}  {} {} {} {}  {} {} {} {}  {} {} {} {} }}",
			m.f[ 0], m.f[ 1], m.f[ 2], m.f[ 3], 
			m.f[ 4], m.f[ 5], m.f[ 6], m.f[ 7], 
			m.f[ 8], m.f[ 9], m.f[10], m.f[11], 
			m.f[12], m.f[13], m.f[14], m.f[15]);
  }
};


template<class M>
__attribute__((noinline)) void do_test(char const* name, M a, M const& b)
{
	doctest::detail::Timer s;
	s.start();
	for (int i=0; i<10000; ++i)
	{
		M r = stir::mul(a, b);
		a = r;
	}
	int e = s.getElapsedMicroseconds();
	fmt::print("{}: {} -- time:{}\n", name, a, e); 
}


__attribute__((noinline)) void do_shuffle(char const* name, xsimd::batch<float, 4> a)
{
	xsimd::batch<float, 4> r = stir::dos(a);
	fmt::print("v: {} {} {} {}\n", r[0], r[1], r[2], r[3]);
}

template<class M>
__attribute__((noinline)) int do_test_loop(std::vector<M> const& a, std::vector<M> const& b, M& r)
{
	doctest::detail::Timer s;
	s.start();
	
	for (int i=0; i<a.size(); ++i)
		r += stir::mul(a[i], b[i]);
	return s.getElapsedMicroseconds();
}

template<class M>
__attribute__((noinline)) void do_test(int LOOPCNT, char const* name, std::vector<M> const& a, std::vector<M> const& b)
{
	M r;
	long long sum = 0;
	for (int i=0; i<LOOPCNT; ++i)
		sum += do_test_loop(a, b, r);
	fmt::print("{}: {} -- time avg of {}:{}\n", name, r, LOOPCNT, (double)sum/LOOPCNT); 
}

__attribute__((noinline)) void do_rot1(char const* type, int CNT, stir::Quaternion const& q)
{
	doctest::detail::Timer s;
	s.start();
	stir::Vector3 v(0,0,1);
	for (int i=0; i<CNT; ++i)
		v = stir::rot(v, q);

	fmt::print("{} --> {} {} {}, time:{}\n", type, v.x, v.y, v.z, s.getElapsedMicroseconds());
}

__attribute__((noinline)) void do_rot2(char const* type, int CNT, stir::Quaternion const& q)
{
	doctest::detail::Timer s;
	s.start();
	stir::Vector3 v(0,0,1);
	for (int i=0; i<CNT; ++i)
	{
		stir::Matrix m = q.rot_matrix();
		v = stir::mul3(v, m);
	}

	fmt::print("{} --> {} {} {}, time:{}\n", type, v.x, v.y, v.z, s.getElapsedMicroseconds());
}

std::random_device rd;
std::uniform_real_distribution<float> dist(0.1f, 0.35f);

void makem(stir::Matrix& m1a, stir::Matrix& m1b, stir::Matrix1& m2a, stir::Matrix1& m2b)
{
	for (int i=0; i<4; ++i)
	{
		float a1 = dist(rd), b1 = dist(rd), c1 = dist(rd), d1 = dist(rd);
		float a2 = dist(rd), b2 = dist(rd), c2 = dist(rd), d2 = dist(rd);
		m1a.set(i, a1, b1, c1, d1);
		m1b.set(i, a2, b2, c2, d2);
		m2a.set(i, a1, b1, c1, d1);
		m2b.set(i, a2, b2, c2, d2);
	}
}
#endif

int main(int argc, char** argv)
{
	auto [shouldExit, res] = run_tests(argc, argv);
/*
	stir::Matrix mi1{1.0f,0,0,0, 0,1.0f,0,0, 0,0,1.0f,0, 0,0,0,1.0f};
	stir::Matrix1 mi2{1.0f,0,0,0, 0,1.0f,0,0, 0,0,1.0f,0, 0,0,0,1.0f};
	
	constexpr int CNT = 5000;

	std::vector<stir::Matrix> v1a, v1b;
	std::vector<stir::Matrix1> v2a, v2b;

	v1a.resize(CNT);
	v1b.resize(CNT);
	v2a.resize(CNT);
	v2b.resize(CNT);

	for (int i=0; i<CNT; ++i)
		makem(v1a[i], v1b[i], v2a[i], v2b[i]);

//	stir::Matrix m1{{{2.0f,0,0,0}, {0,1.0f,0,0}, {0,0,1.0f,0}, {0,0,0,1.0f}}};

	int LOOPCNT=20000;
//	if (argc >= 2 && strcmp(argv[1], "1") == 0)
//		do_test(LOOPCNT, "stir::Matrix1", v2a, v2b);
//	else
	if (argc >= 2 && strcmp(argv[1], "2") == 0)
		do_test(LOOPCNT, "stir::Matrix", v1a, v1b);


	do_shuffle("stir::dos", v1a[0].row[1]);

	stir::Quaternion q = stir::Quaternion::from_axis_angle(stir::Vector3(1,0,0), stir::pi<float>/2.0f);
	do_rot1("quat", CNT, q);
	do_rot2("matrix3", CNT, q);


	stir::Vector4 v4 {0,1,2,3};
	fmt::print("{} {} {} {}\n", v4.x, v4.y, v4.z, v4.w);
	*/

	return 0;


}
