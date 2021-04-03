#include <fmt/format.h>
#include <fmt/ostream.h>

template <>
struct fmt::formatter<stir::vec2> {
  template <typename ParseContext>
  constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const stir::vec2 &a, FormatContext &ctx) {
    return format_to(ctx.out(), "{{ {} {} }}", a.x, a.y);
  }
};

template <>
struct fmt::formatter<stir::vec3> {
  template <typename ParseContext>
  constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const stir::vec3 &a, FormatContext &ctx) {
    return format_to(ctx.out(), "{{ {} {} {} }}", a.x, a.y, a.z);
  }
};

template <>
struct fmt::formatter<stir::vec4> {
  template <typename ParseContext>
  constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const stir::vec4 &a, FormatContext &ctx) {
    return format_to(ctx.out(), "{{ {} {} {} {} }}", a.x, a.y, a.z, a.w);
  }
};

template <>
struct fmt::formatter<stir::quat> {
  template <typename ParseContext>
  constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const stir::quat &a, FormatContext &ctx) {
    return format_to(ctx.out(), "{{ {} {} {} {} }}", a.x, a.y, a.z, a.w);
  }
};

namespace stir {
	inline std::ostream& operator<<(std::ostream& os, stir::vec2 const& a) { fmt::print(os, "{}", a); return os; }
	inline std::ostream& operator<<(std::ostream& os, stir::vec3 const& a) { fmt::print(os, "{}", a); return os; }
	inline std::ostream& operator<<(std::ostream& os, stir::vec4 const& a) { fmt::print(os, "{}", a); return os; }
	inline std::ostream& operator<<(std::ostream& os, stir::quat const& a) { fmt::print(os, "{}", a); return os; }
}


