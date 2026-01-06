#ifndef STIR_MATH_FORMAT_HPP_
#define STIR_MATH_FORMAT_HPP_

#include <fmt/format.h>
#include <fmt/ostream.h>
#include "math.hpp"

template <>
struct fmt::formatter<stir::vec2> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const stir::vec2 &a, FormatContext &ctx) const {
    return format_to(ctx.out(), "{{ {} {} }}", a[0], a[1]);
  }
};

template <>
struct fmt::formatter<stir::vec3> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const stir::vec3 &a, FormatContext &ctx) const {
    return format_to(ctx.out(), "{{ {} {} {} }}", a[0], a[1], a[2]);
  }
};

template <>
struct fmt::formatter<stir::vec4> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const stir::vec4 &a, FormatContext &ctx) const {
    return format_to(ctx.out(), "{{ {} {} {} {} }}", a[0], a[1], a[2], a[3]);
  }
};

template <>
struct fmt::formatter<stir::quat> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const stir::quat &a, FormatContext &ctx) const {
    return format_to(ctx.out(), "{{ {} {} {} {} }}", a[0], a[1], a[2], a[3]);
  }
};

template <>
struct fmt::formatter<stir::matrix> {
  constexpr auto parse(format_parse_context &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const stir::matrix &a, FormatContext &ctx) const {
    return format_to(ctx.out(), "{{ {} {} {} {}  {} {} {} {}  {} {} {} {}  {} {} {} {} }}",
			a(0, 0), a(0, 1), a(0, 2), a(0, 3),
			a(1, 0), a(1, 1), a(1, 2), a(1, 3),
			a(2, 0), a(2, 1), a(2, 2), a(2, 3),
			a(3, 0), a(3, 1), a(3, 2), a(3, 3));
  }
};

namespace stir {
	inline std::ostream& operator<<(std::ostream& os, stir::vec2 const& a) { fmt::print(os, "{}", a); return os; }
	inline std::ostream& operator<<(std::ostream& os, stir::vec3 const& a) { fmt::print(os, "{}", a); return os; }
	inline std::ostream& operator<<(std::ostream& os, stir::vec4 const& a) { fmt::print(os, "{}", a); return os; }
	inline std::ostream& operator<<(std::ostream& os, stir::quat const& a) { fmt::print(os, "{}", a); return os; }
	inline std::ostream& operator<<(std::ostream& os, stir::matrix const& a) { fmt::print(os, "{}", a); return os; }
}

#endif // STIR_MATH_FORMAT_HPP_
