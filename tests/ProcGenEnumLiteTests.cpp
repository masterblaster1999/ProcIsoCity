#include "isocity/ProcGen.hpp"

#include <cstdint>
#include <iostream>
#include <string>

static int g_failures = 0;

#define EXPECT_TRUE(cond)                                                                                            \
  do {                                                                                                               \
    if (!(cond)) {                                                                                                   \
      ++g_failures;                                                                                                  \
      std::cerr << __FILE__ << ":" << __LINE__ << " EXPECT_TRUE failed: " << #cond << "\n";                          \
    }                                                                                                                \
  } while (0)

#define EXPECT_FALSE(cond) EXPECT_TRUE(!(cond))

#define EXPECT_EQ(a, b)                                                                                              \
  do {                                                                                                               \
    const auto _a = (a);                                                                                             \
    const auto _b = (b);                                                                                             \
    if (!(_a == _b)) {                                                                                               \
      ++g_failures;                                                                                                  \
      std::cerr << __FILE__ << ":" << __LINE__ << " EXPECT_EQ failed: " << #a << " == " << #b << "\n";              \
    }                                                                                                                \
  } while (0)

static void TestTerrainPresetRoundtripAndAliases()
{
  using namespace isocity;

  for (std::uint8_t i = 0; i < ProcGenTerrainPresetCountU8(); ++i) {
    const auto p = static_cast<ProcGenTerrainPreset>(i);
    const char* name = ToString(p);
    EXPECT_TRUE(name != nullptr);
    ProcGenTerrainPreset out{};
    EXPECT_TRUE(ParseProcGenTerrainPreset(name, out));
    EXPECT_EQ(static_cast<int>(out), static_cast<int>(p));
  }

  ProcGenTerrainPreset out{};
  EXPECT_TRUE(ParseProcGenTerrainPreset("AtOlL", out));
  EXPECT_EQ(out, ProcGenTerrainPreset::Atoll);
  EXPECT_TRUE(ParseProcGenTerrainPreset("reef", out));
  EXPECT_EQ(out, ProcGenTerrainPreset::Atoll);

  EXPECT_TRUE(ParseProcGenTerrainPreset("PeNiNsUlA", out));
  EXPECT_EQ(out, ProcGenTerrainPreset::Peninsula);
  EXPECT_TRUE(ParseProcGenTerrainPreset("cape", out));
  EXPECT_EQ(out, ProcGenTerrainPreset::Peninsula);
  EXPECT_TRUE(ParseProcGenTerrainPreset("promontory", out));
  EXPECT_EQ(out, ProcGenTerrainPreset::Peninsula);

  EXPECT_FALSE(ParseProcGenTerrainPreset("", out));
}

static void TestDistrictingRoundtrip()
{
  using namespace isocity;

  for (std::uint8_t i = 0; i < ProcGenDistrictingModeCountU8(); ++i) {
    const auto m = static_cast<ProcGenDistrictingMode>(i);
    const char* name = ToString(m);
    EXPECT_TRUE(name != nullptr);
    ProcGenDistrictingMode out{};
    EXPECT_TRUE(ParseProcGenDistrictingMode(name, out));
    EXPECT_EQ(static_cast<int>(out), static_cast<int>(m));
  }
}

static void TestRoadLayoutRoundtrip()
{
  using namespace isocity;

  for (std::uint8_t i = 0; i < ProcGenRoadLayoutCountU8(); ++i) {
    const auto r = static_cast<ProcGenRoadLayout>(i);
    const char* name = ToString(r);
    EXPECT_TRUE(name != nullptr);
    ProcGenRoadLayout out{};
    EXPECT_TRUE(ParseProcGenRoadLayout(name, out));
    EXPECT_EQ(static_cast<int>(out), static_cast<int>(r));
  }
}

int main()
{
  TestTerrainPresetRoundtripAndAliases();
  TestDistrictingRoundtrip();
  TestRoadLayoutRoundtrip();

  if (g_failures == 0) {
    std::cout << "proc_isocity_procgen_enum_tests: OK\n";
    return 0;
  }

  std::cerr << "proc_isocity_procgen_enum_tests: FAILED (" << g_failures << ")\n";
  return 1;
}
