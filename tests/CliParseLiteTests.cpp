#include "cli/CliParse.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace fs = std::filesystem;

static int g_failures = 0;

#define EXPECT_TRUE(cond)                                                                                            \
  do {                                                                                                               \
    if (!(cond)) {                                                                                                   \
      ++g_failures;                                                                                                  \
      std::cerr << __FILE__ << ":" << __LINE__ << " EXPECT_TRUE failed: " << #cond << "\n";                    \
    }                                                                                                                \
  } while (0)

#define EXPECT_FALSE(cond) EXPECT_TRUE(!(cond))

#define EXPECT_EQ(a, b)                                                                                              \
  do {                                                                                                               \
    const auto _a = (a);                                                                                             \
    const auto _b = (b);                                                                                             \
    if (!(_a == _b)) {                                                                                               \
      ++g_failures;                                                                                                  \
      std::cerr << __FILE__ << ":" << __LINE__ << " EXPECT_EQ failed: " << #a << " == " << #b << "\n";   \
    }                                                                                                                \
  } while (0)

#define EXPECT_NE(a, b)                                                                                              \
  do {                                                                                                               \
    const auto _a = (a);                                                                                             \
    const auto _b = (b);                                                                                             \
    if ((_a == _b)) {                                                                                                \
      ++g_failures;                                                                                                  \
      std::cerr << __FILE__ << ":" << __LINE__ << " EXPECT_NE failed: " << #a << " != " << #b << "\n";   \
    }                                                                                                                \
  } while (0)

#define ASSERT_TRUE(cond)                                                                                            \
  do {                                                                                                               \
    if (!(cond)) {                                                                                                   \
      ++g_failures;                                                                                                  \
      std::cerr << __FILE__ << ":" << __LINE__ << " ASSERT_TRUE failed: " << #cond << "\n";                    \
      return;                                                                                                        \
    }                                                                                                                \
  } while (0)

static fs::path MakeTempPath(const std::string& prefix)
{
  static std::uint64_t counter = 0;
  ++counter;

  std::error_code ec;
  fs::path root = fs::temp_directory_path(ec);
  if (ec || root.empty()) {
    root = fs::current_path(ec);
    if (ec || root.empty()) {
      root = fs::path(".");
    }
  }

  const auto stamp = static_cast<std::uint64_t>(
    std::chrono::high_resolution_clock::now().time_since_epoch().count());

  return root / (prefix + "_" + std::to_string(stamp) + "_" + std::to_string(counter));
}

static void TestParseI32()
{
  using namespace isocity::cli;

  int v = 0;
  EXPECT_TRUE(ParseI32("0", &v));
  EXPECT_EQ(v, 0);
  EXPECT_TRUE(ParseI32("-1", &v));
  EXPECT_EQ(v, -1);
  EXPECT_TRUE(ParseI32("+7", &v));
  EXPECT_EQ(v, 7);

  // Leading/trailing junk should fail.
  EXPECT_FALSE(ParseI32("1.0", &v));
  EXPECT_FALSE(ParseI32("1 ", &v));
  EXPECT_FALSE(ParseI32(" 1", &v));
  EXPECT_FALSE(ParseI32("1a", &v));
  EXPECT_FALSE(ParseI32("", &v));
  EXPECT_FALSE(ParseI32("+", &v));

  // Overflow should fail.
  EXPECT_FALSE(ParseI32("2147483648", &v));
  EXPECT_FALSE(ParseI32("-2147483649", &v));
}

static void TestParseU64()
{
  using namespace isocity::cli;

  std::uint64_t v = 0;
  EXPECT_TRUE(ParseU64("0", &v));
  EXPECT_EQ(v, 0u);
  EXPECT_TRUE(ParseU64("42", &v));
  EXPECT_EQ(v, 42u);
  EXPECT_TRUE(ParseU64("+7", &v));
  EXPECT_EQ(v, 7u);

  EXPECT_TRUE(ParseU64("0x10", &v));
  EXPECT_EQ(v, 16u);
  EXPECT_TRUE(ParseU64("0Xff", &v));
  EXPECT_EQ(v, 255u);
  EXPECT_TRUE(ParseU64("+0x10", &v));
  EXPECT_EQ(v, 16u);

  EXPECT_TRUE(ParseU64("18446744073709551615", &v));
  EXPECT_EQ(v, std::numeric_limits<std::uint64_t>::max());

  EXPECT_TRUE(ParseU64("0xffffffffffffffff", &v));
  EXPECT_EQ(v, std::numeric_limits<std::uint64_t>::max());

  EXPECT_FALSE(ParseU64("", &v));
  EXPECT_FALSE(ParseU64("-1", &v));
  EXPECT_FALSE(ParseU64("0x", &v));
  EXPECT_FALSE(ParseU64("0xg", &v));
  EXPECT_FALSE(ParseU64("18446744073709551616", &v));
  EXPECT_FALSE(ParseU64(" 1", &v));
}

static void TestParseFloats()
{
  using namespace isocity::cli;

  double d = 0.0;
  float f = 0.0f;

  EXPECT_TRUE(ParseF64("0", &d));
  EXPECT_EQ(d, 0.0);
  EXPECT_TRUE(ParseF64("3.5", &d));
  EXPECT_EQ(d, 3.5);
  EXPECT_TRUE(ParseF64("-1e-3", &d));
  EXPECT_EQ(d, -1e-3);

  EXPECT_FALSE(ParseF64("nan", &d));
  EXPECT_FALSE(ParseF64("inf", &d));
  EXPECT_FALSE(ParseF64("1e309", &d));
  EXPECT_FALSE(ParseF64("1 ", &d));

  EXPECT_TRUE(ParseF32("3.5", &f));
  EXPECT_EQ(f, 3.5f);

  // Out-of-range for float should be rejected even if double parses.
  EXPECT_FALSE(ParseF32("1e40", &f));
}

static void TestParseBool01()
{
  using namespace isocity::cli;

  bool b = false;

  EXPECT_TRUE(ParseBool01("0", &b));
  EXPECT_FALSE(b);
  EXPECT_TRUE(ParseBool01("1", &b));
  EXPECT_TRUE(b);

  EXPECT_TRUE(ParseBool01("true", &b));
  EXPECT_TRUE(b);
  EXPECT_TRUE(ParseBool01("FALSE", &b));
  EXPECT_FALSE(b);
  EXPECT_TRUE(ParseBool01("TrUe", &b));
  EXPECT_TRUE(b);

  EXPECT_TRUE(ParseBool01("on", &b));
  EXPECT_TRUE(b);
  EXPECT_TRUE(ParseBool01("OFF", &b));
  EXPECT_FALSE(b);

  EXPECT_TRUE(ParseBool01("yes", &b));
  EXPECT_TRUE(b);
  EXPECT_TRUE(ParseBool01("No", &b));
  EXPECT_FALSE(b);

  EXPECT_FALSE(ParseBool01("", &b));
  EXPECT_FALSE(ParseBool01("maybe", &b));
  EXPECT_FALSE(ParseBool01("2", &b));
}

static void TestParseWxH()
{
  using namespace isocity::cli;

  int w = 0;
  int h = 0;

  EXPECT_TRUE(ParseWxH("16x8", &w, &h));
  EXPECT_EQ(w, 16);
  EXPECT_EQ(h, 8);

  EXPECT_TRUE(ParseWxH("+32X64", &w, &h));
  EXPECT_EQ(w, 32);
  EXPECT_EQ(h, 64);

  EXPECT_FALSE(ParseWxH("16", &w, &h));
  EXPECT_FALSE(ParseWxH("16x", &w, &h));
  EXPECT_FALSE(ParseWxH("x8", &w, &h));
  EXPECT_FALSE(ParseWxH("0x8", &w, &h));
  EXPECT_FALSE(ParseWxH("16x0", &w, &h));
}

static void TestTriples()
{
  using namespace isocity::cli;

  float a = 0.0f;
  float b = 0.0f;
  float c = 0.0f;

  EXPECT_TRUE(ParseF32Triple("1,2,3", &a, &b, &c));
  EXPECT_EQ(a, 1.0f);
  EXPECT_EQ(b, 2.0f);
  EXPECT_EQ(c, 3.0f);

  EXPECT_TRUE(ParseF32Triple("1x2x3", &a, &b, &c));
  EXPECT_EQ(a, 1.0f);
  EXPECT_EQ(b, 2.0f);
  EXPECT_EQ(c, 3.0f);

  EXPECT_FALSE(ParseF32Triple("1,2", &a, &b, &c));
  EXPECT_FALSE(ParseF32Triple("", &a, &b, &c));

  std::uint8_t ua = 0;
  std::uint8_t ub = 0;
  std::uint8_t uc = 0;

  EXPECT_TRUE(ParseU8Triple("0,128,255", &ua, &ub, &uc));
  EXPECT_EQ(ua, static_cast<std::uint8_t>(0));
  EXPECT_EQ(ub, static_cast<std::uint8_t>(128));
  EXPECT_EQ(uc, static_cast<std::uint8_t>(255));

  // Clamp + round.
  EXPECT_TRUE(ParseU8Triple("-1,256,3.6", &ua, &ub, &uc));
  EXPECT_EQ(ua, static_cast<std::uint8_t>(0));
  EXPECT_EQ(ub, static_cast<std::uint8_t>(255));
  EXPECT_EQ(uc, static_cast<std::uint8_t>(4));
}

static void TestHexU64()
{
  using namespace isocity::cli;

  EXPECT_EQ(HexU64(0u), std::string("0x0000000000000000"));
  EXPECT_EQ(HexU64(1u), std::string("0x0000000000000001"));

  const std::string s = HexU64(0xabcdefu);
  EXPECT_TRUE(s.rfind("0x", 0) == 0);
  EXPECT_EQ(s.size(), std::string("0x0000000000000000").size());
}

static void TestSplitCommaList()
{
  using namespace isocity::cli;

  {
    const auto v = SplitCommaList("a,b,c");
    ASSERT_TRUE(v.size() == 3);
    EXPECT_EQ(v[0], "a");
    EXPECT_EQ(v[1], "b");
    EXPECT_EQ(v[2], "c");
  }

  {
    const auto v = SplitCommaList("a, b, ,c, ");
    ASSERT_TRUE(v.size() == 3);
    EXPECT_EQ(v[0], "a");
    EXPECT_EQ(v[1], "b");
    EXPECT_EQ(v[2], "c");
  }

  {
    const auto v = SplitCommaList("");
    EXPECT_TRUE(v.empty());
  }
}

static void TestEnsureDirs()
{
  using namespace isocity::cli;

  std::error_code ec;
  const fs::path base = MakeTempPath("procisocity_cli_parse_dirs");

  EXPECT_FALSE(EnsureDir(fs::path{}));
  EXPECT_TRUE(EnsureDir(base / "a" / "b"));
  EXPECT_TRUE(fs::exists(base / "a" / "b"));

  EXPECT_FALSE(EnsureParentDir(fs::path{}));

  const fs::path file = base / "c" / "d" / "out.txt";
  EXPECT_TRUE(EnsureParentDir(file));
  EXPECT_TRUE(fs::exists(base / "c" / "d"));

  fs::remove_all(base, ec);
}

int main()
{
  TestParseI32();
  TestParseU64();
  TestParseFloats();
  TestParseBool01();
  TestParseWxH();
  TestTriples();
  TestHexU64();
  TestSplitCommaList();
  TestEnsureDirs();

  if (g_failures == 0) {
    std::cout << "proc_isocity_cli_parse_tests: OK\n";
    return 0;
  }

  std::cerr << "proc_isocity_cli_parse_tests: FAILED (" << g_failures << ")\n";
  return 1;
}
