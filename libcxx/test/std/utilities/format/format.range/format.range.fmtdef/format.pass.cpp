//===----------------------------------------------------------------------===//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

// UNSUPPORTED: c++03, c++11, c++14, c++17, c++20

// UNSUPPORTED: GCC-ALWAYS_INLINE-FIXME

// <format>

// template<ranges::input_range R, class charT>
//   struct range-default-formatter<range_format::sequence, R, charT>

// template<class FormatContext>
//   typename FormatContext::iterator
//     format(maybe-const-r& elems, FormatContext& ctx) const;

#include <array>
#include <cassert>
#include <concepts>
#include <format>
#include <iterator>

#include "assert_macros.h"
#include "format.functions.common.h"
#include "test_format_context.h"
#include "test_macros.h"
#include "make_string.h"

#define SV(S) MAKE_STRING_VIEW(CharT, S)

template <class StringViewT>
void test_format(StringViewT expected, std::array<int, 2> arg) {
  using CharT      = typename StringViewT::value_type;
  using String     = std::basic_string<CharT>;
  using OutIt      = std::back_insert_iterator<String>;
  using FormatCtxT = std::basic_format_context<OutIt, CharT>;

  std::formatter<std::array<int, 2>, CharT> formatter;

  String result;
  OutIt out             = std::back_inserter(result);
  FormatCtxT format_ctx = test_format_context_create<OutIt, CharT>(out, std::make_format_args<FormatCtxT>(arg));
  formatter.format(arg, format_ctx);
  assert(result == expected);
}

template <class CharT>
void test_assure_parse_is_called(std::basic_string_view<CharT> fmt) {
  using String     = std::basic_string<CharT>;
  using OutIt      = std::back_insert_iterator<String>;
  using FormatCtxT = std::basic_format_context<OutIt, CharT>;
  std::array<parse_call_validator, 2> arg;

  String result;
  OutIt out             = std::back_inserter(result);
  FormatCtxT format_ctx = test_format_context_create<OutIt, CharT>(out, std::make_format_args<FormatCtxT>(arg));

  std::formatter<decltype(arg), CharT> formatter;
  std::basic_format_parse_context<CharT> ctx{fmt};

  formatter.parse(ctx);
  formatter.format(arg, format_ctx);
}

template <class CharT>
void test_assure_parse_is_called() {
  using String     = std::basic_string<CharT>;
  using OutIt      = std::back_insert_iterator<String>;
  using FormatCtxT = std::basic_format_context<OutIt, CharT>;
  std::array<parse_call_validator, 2> arg;

  String result;
  OutIt out             = std::back_inserter(result);
  FormatCtxT format_ctx = test_format_context_create<OutIt, CharT>(out, std::make_format_args<FormatCtxT>(arg));

  { // parse not called
    [[maybe_unused]] const std::formatter<decltype(arg), CharT> formatter;
    TEST_THROWS_TYPE(parse_call_validator::parse_function_not_called, formatter.format(arg, format_ctx));
  }

  test_assure_parse_is_called(SV("5"));
  test_assure_parse_is_called(SV("n"));
  test_assure_parse_is_called(SV("5n"));
}

template <class CharT>
void test_fmt() {
  test_format(SV("[1, 42]"), std::array<int, 2>{{1, 42}});
  test_format(SV("[0, 99]"), std::array<int, 2>{{0, 99}});

  test_assure_parse_is_called<CharT>();
}

void test() {
  test_fmt<char>();
#ifndef TEST_HAS_NO_WIDE_CHARACTERS
  test_fmt<wchar_t>();
#endif
}

int main(int, char**) {
  test();

  return 0;
}
