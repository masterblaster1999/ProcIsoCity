#include "isocity/ZipWriter.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace fs = std::filesystem;

static int g_failures = 0;

#define EXPECT_TRUE(cond)                                                                                            \
  do {                                                                                                               \
    if (!(cond)) {                                                                                                   \
      ++g_failures;                                                                                                  \
      std::cerr << __FILE__ << ":" << __LINE__ << " EXPECT_TRUE failed: " << #cond << "\n";                         \
    }                                                                                                                \
  } while (0)

#define EXPECT_FALSE(cond) EXPECT_TRUE(!(cond))

#define EXPECT_EQ(a, b)                                                                                              \
  do {                                                                                                               \
    const auto _a = (a);                                                                                             \
    const auto _b = (b);                                                                                             \
    if (!(_a == _b)) {                                                                                               \
      ++g_failures;                                                                                                  \
      std::cerr << __FILE__ << ":" << __LINE__ << " EXPECT_EQ failed: " << #a << " == " << #b << "\n";             \
    }                                                                                                                \
  } while (0)

#define ASSERT_TRUE(cond)                                                                                            \
  do {                                                                                                               \
    if (!(cond)) {                                                                                                   \
      ++g_failures;                                                                                                  \
      std::cerr << __FILE__ << ":" << __LINE__ << " ASSERT_TRUE failed: " << #cond << "\n";                         \
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

static bool ReadFirst2Bytes(const fs::path& file, char outSig[2])
{
  std::ifstream ifs(file, std::ios::binary);
  if (!ifs) return false;
  ifs.read(outSig, 2);
  return ifs.gcount() == 2;
}

static void TestRejectsDuplicateEntryNames()
{
  using namespace isocity;

  fs::path zipFile = MakeTempPath("procisocity_zip_dupe");
  zipFile += ".zip";

  std::error_code ec;
  fs::remove(zipFile, ec);

  ZipWriter zw;
  std::string err;
  ASSERT_TRUE(zw.open(zipFile, err));
  EXPECT_TRUE(err.empty());

  EXPECT_TRUE(zw.addFileFromString("a.txt", "hello", err));
  EXPECT_FALSE(zw.addFileFromString("a.txt", "world", err));
  EXPECT_TRUE(err.find("duplicate") != std::string::npos);

  // Normalization should also collide.
  err.clear();
  EXPECT_TRUE(zw.addFileFromString("foo\\bar.txt", "x", err));
  EXPECT_FALSE(zw.addFileFromString("foo/bar.txt", "y", err));
  EXPECT_TRUE(err.find("duplicate") != std::string::npos);

  EXPECT_TRUE(zw.finalize(err));
  zw.close();

  // Signature sanity check (local file header starts with 'PK').
  char sig[2] = {0, 0};
  ASSERT_TRUE(ReadFirst2Bytes(zipFile, sig));
  EXPECT_EQ(sig[0], 'P');
  EXPECT_EQ(sig[1], 'K');

  fs::remove(zipFile, ec);
}

static void TestBlocksZipSlipSegments()
{
  using namespace isocity;

  fs::path zipFile = MakeTempPath("procisocity_zip_slip");
  zipFile += ".zip";

  std::error_code ec;
  fs::remove(zipFile, ec);

  ZipWriter zw;
  std::string err;
  ASSERT_TRUE(zw.open(zipFile, err));

  err.clear();
  EXPECT_FALSE(zw.addFileFromString("../evil.txt", "nope", err));
  EXPECT_TRUE(err.find("blocked") != std::string::npos);

  // Leading slashes are stripped.
  err.clear();
  EXPECT_TRUE(zw.addFileFromString("/ok.txt", "ok", err));

  EXPECT_TRUE(zw.finalize(err));
  zw.close();

  fs::remove(zipFile, ec);
}

static void TestAddFileFromPathDuplicate()
{
  using namespace isocity;

  fs::path tmp1 = MakeTempPath("procisocity_zip_src1");
  tmp1 += ".bin";
  fs::path tmp2 = MakeTempPath("procisocity_zip_src2");
  tmp2 += ".bin";

  {
    std::ofstream o1(tmp1, std::ios::binary | std::ios::trunc);
    std::ofstream o2(tmp2, std::ios::binary | std::ios::trunc);
    ASSERT_TRUE(static_cast<bool>(o1));
    ASSERT_TRUE(static_cast<bool>(o2));
    o1 << "one";
    o2 << "two";
  }

  fs::path zipFile = MakeTempPath("procisocity_zip_path_dupe");
  zipFile += ".zip";

  std::error_code ec;
  fs::remove(zipFile, ec);

  ZipWriter zw;
  std::string err;
  ASSERT_TRUE(zw.open(zipFile, err));

  EXPECT_TRUE(zw.addFileFromPath("data.bin", tmp1, err));
  EXPECT_FALSE(zw.addFileFromPath("data.bin", tmp2, err));
  EXPECT_TRUE(err.find("duplicate") != std::string::npos);

  EXPECT_TRUE(zw.finalize(err));
  zw.close();

  fs::remove(zipFile, ec);
  fs::remove(tmp1, ec);
  fs::remove(tmp2, ec);
}

int main()
{
  TestRejectsDuplicateEntryNames();
  TestBlocksZipSlipSegments();
  TestAddFileFromPathDuplicate();

  if (g_failures == 0) {
    std::cout << "proc_isocity_zip_tests: OK\n";
    return 0;
  }

  std::cerr << "proc_isocity_zip_tests: FAILED (" << g_failures << ")\n";
  return 1;
}
