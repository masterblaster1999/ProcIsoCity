#include "isocity/SupportBundle.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

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

static bool FileExists(const fs::path& p)
{
  std::error_code ec;
  return fs::exists(p, ec) && !ec;
}

static bool ReadFirst2Bytes(const fs::path& file, char outSig[2])
{
  std::ifstream ifs(file, std::ios::binary);
  if (!ifs) return false;
  ifs.read(outSig, 2);
  return ifs.gcount() == 2;
}

static bool ReadTextFile(const fs::path& p, std::string& out)
{
  out.clear();
  std::ifstream ifs(p, std::ios::in | std::ios::binary);
  if (!ifs) return false;
  std::ostringstream ss;
  ss << ifs.rdbuf();
  out = ss.str();
  return true;
}

static bool WriteTextFile(const fs::path& p, const std::string& content)
{
  std::error_code ec;
  fs::create_directories(p.parent_path(), ec);
  std::ofstream ofs(p, std::ios::binary | std::ios::trunc);
  if (!ofs) return false;
  ofs << content;
  return static_cast<bool>(ofs);
}

static std::uint16_t ReadLE16(const std::vector<std::uint8_t>& buf, std::size_t off)
{
  if (off + 1 >= buf.size()) return 0;
  return static_cast<std::uint16_t>(buf[off]) | (static_cast<std::uint16_t>(buf[off + 1]) << 8);
}

static std::uint32_t ReadLE32(const std::vector<std::uint8_t>& buf, std::size_t off)
{
  if (off + 3 >= buf.size()) return 0;
  return static_cast<std::uint32_t>(buf[off]) |
         (static_cast<std::uint32_t>(buf[off + 1]) << 8) |
         (static_cast<std::uint32_t>(buf[off + 2]) << 16) |
         (static_cast<std::uint32_t>(buf[off + 3]) << 24);
}

static bool ReadFileBytes(const fs::path& p, std::vector<std::uint8_t>& out)
{
  out.clear();
  std::ifstream ifs(p, std::ios::binary);
  if (!ifs) return false;
  ifs.seekg(0, std::ios::end);
  const std::streamoff sz = ifs.tellg();
  if (sz <= 0) return false;
  ifs.seekg(0, std::ios::beg);
  out.resize(static_cast<std::size_t>(sz));
  ifs.read(reinterpret_cast<char*>(out.data()), sz);
  return ifs.gcount() == sz;
}

static bool FindZipEocd(const std::vector<std::uint8_t>& buf, std::size_t& outOff)
{
  outOff = 0;
  if (buf.size() < 22) return false;

  // EOCD can be followed by an optional comment (max 65535 bytes).
  const std::size_t searchMax = std::min<std::size_t>(buf.size(), 22 + 65535);
  const std::size_t start = buf.size() - searchMax;

  for (std::size_t i = buf.size() - 22; i + 3 < buf.size() && i >= start; --i) {
    if (buf[i + 0] == 0x50 && buf[i + 1] == 0x4b && buf[i + 2] == 0x05 && buf[i + 3] == 0x06) {
      outOff = i;
      return true;
    }
    if (i == 0) break;
  }
  return false;
}

static bool ListZipEntries(const fs::path& zipFile, std::vector<std::string>& outNames)
{
  outNames.clear();
  std::vector<std::uint8_t> buf;
  if (!ReadFileBytes(zipFile, buf)) return false;

  std::size_t eocdOff = 0;
  if (!FindZipEocd(buf, eocdOff)) return false;

  const std::uint32_t cdSize = ReadLE32(buf, eocdOff + 12);
  const std::uint32_t cdOff = ReadLE32(buf, eocdOff + 16);
  if (cdOff >= buf.size()) return false;
  if (static_cast<std::size_t>(cdOff) + static_cast<std::size_t>(cdSize) > buf.size()) return false;

  std::size_t cur = static_cast<std::size_t>(cdOff);
  const std::size_t end = cur + static_cast<std::size_t>(cdSize);

  while (cur + 46 <= end) {
    const std::uint32_t sig = ReadLE32(buf, cur);
    if (sig != 0x02014b50u) return false; // central directory header

    const std::uint16_t nameLen = ReadLE16(buf, cur + 28);
    const std::uint16_t extraLen = ReadLE16(buf, cur + 30);
    const std::uint16_t commentLen = ReadLE16(buf, cur + 32);

    const std::size_t nameOff = cur + 46;
    const std::size_t next = nameOff + static_cast<std::size_t>(nameLen) +
                             static_cast<std::size_t>(extraLen) +
                             static_cast<std::size_t>(commentLen);
    if (next > end) return false;

    std::string name;
    name.reserve(nameLen);
    for (std::size_t i = 0; i < nameLen; ++i) {
      name.push_back(static_cast<char>(buf[nameOff + i]));
    }
    outNames.push_back(name);
    cur = next;
  }

  return true;
}

static bool Contains(const std::vector<std::string>& v, const std::string& s)
{
  for (const auto& x : v) {
    if (x == s) return true;
  }
  return false;
}

static void TestDirBundleSanitizesPrefix()
{
  using namespace isocity;

  fs::path base = MakeTempPath("procisocity_support_dir");
  std::error_code ec;
  fs::remove_all(base, ec);
  fs::create_directories(base, ec);
  ASSERT_TRUE(!ec);

  SupportBundleOptions opt;
  opt.baseDir = base;
  opt.dataDir = base;
  opt.namePrefix = "my:bad/prefix..\\test  ";
  opt.diagnosticsText = "hello";
  opt.includeManifest = true;

  SupportBundleResult res;
  std::string err;
  ASSERT_TRUE(CreateSupportBundle(opt, res, err));
  EXPECT_TRUE(err.empty());

  EXPECT_TRUE(FileExists(res.bundleDir));
  EXPECT_TRUE(FileExists(res.filesDir));

  const std::string dirName = res.bundleDir.filename().string();
  EXPECT_TRUE(dirName.rfind("my_bad_prefix_test_", 0) == 0);
  EXPECT_TRUE(dirName.find(':') == std::string::npos);
  EXPECT_TRUE(dirName.find('/') == std::string::npos);
  EXPECT_TRUE(dirName.find('\\') == std::string::npos);
  EXPECT_FALSE(!dirName.empty() && (dirName.back() == '.' || dirName.back() == ' '));

  EXPECT_TRUE(FileExists(res.bundleDir / "diagnostics.txt"));
  EXPECT_TRUE(FileExists(res.bundleDir / "manifest.txt"));

  fs::remove_all(base, ec);
}

static void TestDirBundlePreservesDuplicateBasenamesAndSortedManifest()
{
  using namespace isocity;

  fs::path base = MakeTempPath("procisocity_support_dups_dir");
  std::error_code ec;
  fs::remove_all(base, ec);
  fs::create_directories(base, ec);
  ASSERT_TRUE(!ec);

  const fs::path a = base / "a" / "dup.txt";
  const fs::path b = base / "b" / "dup.txt";
  ASSERT_TRUE(WriteTextFile(a, "A"));
  ASSERT_TRUE(WriteTextFile(b, "B"));

  SupportBundleOptions opt;
  opt.baseDir = base;
  opt.dataDir = base;
  opt.namePrefix = "dups";
  opt.diagnosticsText = "hello";
  opt.includeManifest = true;
  opt.extraFiles.push_back(a);
  opt.extraFiles.push_back(b);

  SupportBundleResult res;
  std::string err;
  ASSERT_TRUE(CreateSupportBundle(opt, res, err));
  EXPECT_TRUE(err.empty());

  const fs::path f1 = res.filesDir / "dup.txt";
  const fs::path f2 = res.filesDir / "dup__2.txt";
  EXPECT_TRUE(FileExists(f1));
  EXPECT_TRUE(FileExists(f2));

  std::string s1, s2;
  ASSERT_TRUE(ReadTextFile(f1, s1));
  ASSERT_TRUE(ReadTextFile(f2, s2));
  EXPECT_EQ(s1, "A");
  EXPECT_EQ(s2, "B");

  std::string man;
  ASSERT_TRUE(ReadTextFile(res.bundleDir / "manifest.txt", man));
  const std::size_t p1 = man.find("files/dup.txt");
  const std::size_t p2 = man.find("files/dup__2.txt");
  EXPECT_TRUE(p1 != std::string::npos);
  EXPECT_TRUE(p2 != std::string::npos);
  EXPECT_TRUE(p1 < p2);

  EXPECT_TRUE(man.find("files/dup.txt (1 bytes)") != std::string::npos);
  EXPECT_TRUE(man.find("files/dup__2.txt (1 bytes)") != std::string::npos);

  fs::remove_all(base, ec);
}

static void TestDirBundleExpandsExtraDirectoryAndTruncatesDeterministically()
{
  using namespace isocity;

  fs::path base = MakeTempPath("procisocity_support_dir_extra");
  std::error_code ec;
  fs::remove_all(base, ec);
  fs::create_directories(base, ec);
  ASSERT_TRUE(!ec);

  const fs::path extraDir = base / "extra_dir";
  ASSERT_TRUE(WriteTextFile(extraDir / "a.txt", "A"));
  ASSERT_TRUE(WriteTextFile(extraDir / "sub" / "b.txt", "B"));
  ASSERT_TRUE(WriteTextFile(extraDir / "sub2" / "c.txt", "C"));

  SupportBundleOptions opt;
  opt.baseDir = base;
  opt.dataDir = base;
  opt.namePrefix = "dir_extra";
  opt.diagnosticsText = "hello";
  opt.includeManifest = true;
  opt.extraFiles.push_back(extraDir);

  SupportBundleResult res;
  std::string err;
  ASSERT_TRUE(CreateSupportBundle(opt, res, err));
  EXPECT_TRUE(err.empty());
  EXPECT_TRUE(res.warnings.empty());

  EXPECT_TRUE(FileExists(res.filesDir / "a.txt"));
  EXPECT_TRUE(FileExists(res.filesDir / "b.txt"));
  EXPECT_TRUE(FileExists(res.filesDir / "c.txt"));

  std::string man;
  ASSERT_TRUE(ReadTextFile(res.bundleDir / "manifest.txt", man));
  const std::size_t pa = man.find("files/a.txt");
  const std::size_t pb = man.find("files/b.txt");
  const std::size_t pc = man.find("files/c.txt");
  EXPECT_TRUE(pa != std::string::npos);
  EXPECT_TRUE(pb != std::string::npos);
  EXPECT_TRUE(pc != std::string::npos);
  EXPECT_TRUE(pa < pb);
  EXPECT_TRUE(pb < pc);

  // Truncation selects lexicographically earliest files.
  SupportBundleOptions opt2 = opt;
  opt2.namePrefix = "dir_extra_trunc";
  opt2.extraDirMaxFiles = 2;

  SupportBundleResult res2;
  ASSERT_TRUE(CreateSupportBundle(opt2, res2, err));
  EXPECT_TRUE(err.empty());
  EXPECT_FALSE(res2.warnings.empty());

  EXPECT_TRUE(FileExists(res2.filesDir / "a.txt"));
  EXPECT_TRUE(FileExists(res2.filesDir / "b.txt"));
  EXPECT_FALSE(FileExists(res2.filesDir / "c.txt"));

  bool sawTrunc = false;
  for (const auto& w : res2.warnings) {
    if (w.find("truncated") != std::string::npos) sawTrunc = true;
  }
  EXPECT_TRUE(sawTrunc);

  fs::remove_all(base, ec);
}

static void TestZipBundleIncludesExpectedEntries()
{
  using namespace isocity;

  fs::path base = MakeTempPath("procisocity_support_zip");
  std::error_code ec;
  fs::remove_all(base, ec);
  fs::create_directories(base, ec);
  ASSERT_TRUE(!ec);

  const fs::path extra = base / "extra.txt";
  {
    std::ofstream ofs(extra, std::ios::binary | std::ios::trunc);
    ASSERT_TRUE(static_cast<bool>(ofs));
    ofs << "x";
  }

  SupportBundleOptions opt;
  opt.baseDir = base;
  opt.dataDir = base;
  opt.namePrefix = "zip:bad/prefix";
  opt.diagnosticsText = "hello";
  opt.includeManifest = true;
  opt.extraFiles.push_back(extra);

  SupportBundleArchiveResult res;
  std::string err;
  ASSERT_TRUE(CreateSupportBundleZip(opt, res, err));
  EXPECT_TRUE(err.empty());

  EXPECT_TRUE(FileExists(res.archivePath));
  EXPECT_EQ(res.archivePath.extension().string(), ".zip");

  const std::string stem = res.archivePath.stem().string();
  EXPECT_TRUE(stem.rfind("zip_bad_prefix_", 0) == 0);
  EXPECT_TRUE(stem.find(':') == std::string::npos);
  EXPECT_TRUE(stem.find('/') == std::string::npos);
  EXPECT_TRUE(stem.find('\\') == std::string::npos);

  // Signature sanity check (local file header starts with 'PK').
  char sig[2] = {0, 0};
  ASSERT_TRUE(ReadFirst2Bytes(res.archivePath, sig));
  EXPECT_EQ(sig[0], 'P');
  EXPECT_EQ(sig[1], 'K');

  std::vector<std::string> entries;
  ASSERT_TRUE(ListZipEntries(res.archivePath, entries));

  const std::string root = stem;
  EXPECT_TRUE(Contains(entries, root + "/diagnostics.txt"));
  EXPECT_TRUE(Contains(entries, root + "/manifest.txt"));
  EXPECT_TRUE(Contains(entries, root + "/files/extra.txt"));

  fs::remove_all(base, ec);
}

static void TestZipBundlePreservesDuplicateBasenames()
{
  using namespace isocity;

  fs::path base = MakeTempPath("procisocity_support_dups_zip");
  std::error_code ec;
  fs::remove_all(base, ec);
  fs::create_directories(base, ec);
  ASSERT_TRUE(!ec);

  const fs::path a = base / "a" / "dup.txt";
  const fs::path b = base / "b" / "dup.txt";
  ASSERT_TRUE(WriteTextFile(a, "A"));
  ASSERT_TRUE(WriteTextFile(b, "B"));

  SupportBundleOptions opt;
  opt.baseDir = base;
  opt.dataDir = base;
  opt.namePrefix = "dups";
  opt.diagnosticsText = "hello";
  opt.includeManifest = true;
  opt.extraFiles.push_back(a);
  opt.extraFiles.push_back(b);

  SupportBundleArchiveResult res;
  std::string err;
  ASSERT_TRUE(CreateSupportBundleZip(opt, res, err));
  EXPECT_TRUE(err.empty());
  EXPECT_TRUE(res.warnings.empty());

  std::vector<std::string> entries;
  ASSERT_TRUE(ListZipEntries(res.archivePath, entries));

  const std::string root = res.archivePath.stem().string();
  EXPECT_TRUE(Contains(entries, root + "/files/dup.txt"));
  EXPECT_TRUE(Contains(entries, root + "/files/dup__2.txt"));

  fs::remove_all(base, ec);
}

static void TestZipBundleExpandsExtraDirectoryAndTruncates()
{
  using namespace isocity;

  fs::path base = MakeTempPath("procisocity_support_zip_extra");
  std::error_code ec;
  fs::remove_all(base, ec);
  fs::create_directories(base, ec);
  ASSERT_TRUE(!ec);

  const fs::path extraDir = base / "extra_dir";
  ASSERT_TRUE(WriteTextFile(extraDir / "a.txt", "A"));
  ASSERT_TRUE(WriteTextFile(extraDir / "b.txt", "B"));
  ASSERT_TRUE(WriteTextFile(extraDir / "c.txt", "C"));

  SupportBundleOptions opt;
  opt.baseDir = base;
  opt.dataDir = base;
  opt.namePrefix = "zip_extra";
  opt.diagnosticsText = "hello";
  opt.includeManifest = true;
  opt.extraFiles.push_back(extraDir);
  opt.extraDirMaxFiles = 2;

  SupportBundleArchiveResult res;
  std::string err;
  ASSERT_TRUE(CreateSupportBundleZip(opt, res, err));
  EXPECT_TRUE(err.empty());

  std::vector<std::string> entries;
  ASSERT_TRUE(ListZipEntries(res.archivePath, entries));

  const std::string root = res.archivePath.stem().string();
  EXPECT_TRUE(Contains(entries, root + "/files/a.txt"));
  EXPECT_TRUE(Contains(entries, root + "/files/b.txt"));
  EXPECT_FALSE(Contains(entries, root + "/files/c.txt"));

  bool sawTrunc = false;
  for (const auto& w : res.warnings) {
    if (w.find("truncated") != std::string::npos) sawTrunc = true;
  }
  EXPECT_TRUE(sawTrunc);

  fs::remove_all(base, ec);
}

int main()
{
  TestDirBundleSanitizesPrefix();
  TestDirBundlePreservesDuplicateBasenamesAndSortedManifest();
  TestDirBundleExpandsExtraDirectoryAndTruncatesDeterministically();
  TestZipBundleIncludesExpectedEntries();
  TestZipBundlePreservesDuplicateBasenames();
  TestZipBundleExpandsExtraDirectoryAndTruncates();

  if (g_failures == 0) {
    std::cout << "proc_isocity_support_bundle_tests: OK\n";
    return 0;
  }

  std::cerr << "proc_isocity_support_bundle_tests: FAILED (" << g_failures << ")\n";
  return 1;
}
