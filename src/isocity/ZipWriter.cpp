#include "isocity/ZipWriter.hpp"

#include "isocity/Checksum.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace isocity {

namespace {

constexpr std::uint32_t kSigLocalHeader = 0x04034b50u;
constexpr std::uint32_t kSigCentralHeader = 0x02014b50u;
constexpr std::uint32_t kSigEndOfCentral = 0x06054b50u;
constexpr std::uint32_t kSigDataDescriptor = 0x08074b50u;

constexpr std::uint16_t kVersionMadeBy = 20;      // 2.0
constexpr std::uint16_t kVersionNeeded = 20;      // 2.0
constexpr std::uint16_t kMethodStore = 0;         // no compression
constexpr std::uint16_t kFlagDataDescriptor = 0x0008u;

static void WriteLe16(std::ofstream& ofs, std::uint16_t v)
{
  const unsigned char b[2] = {static_cast<unsigned char>(v & 0xFFu),
                              static_cast<unsigned char>((v >> 8) & 0xFFu)};
  ofs.write(reinterpret_cast<const char*>(b), 2);
}

static void WriteLe32(std::ofstream& ofs, std::uint32_t v)
{
  const unsigned char b[4] = {static_cast<unsigned char>(v & 0xFFu),
                              static_cast<unsigned char>((v >> 8) & 0xFFu),
                              static_cast<unsigned char>((v >> 16) & 0xFFu),
                              static_cast<unsigned char>((v >> 24) & 0xFFu)};
  ofs.write(reinterpret_cast<const char*>(b), 4);
}

static std::uint32_t TellpU32(std::ofstream& ofs)
{
  const std::streampos p = ofs.tellp();
  if (p < 0) return 0;
  const std::uint64_t u = static_cast<std::uint64_t>(p);
  if (u > std::numeric_limits<std::uint32_t>::max()) return std::numeric_limits<std::uint32_t>::max();
  return static_cast<std::uint32_t>(u);
}

static std::time_t FileTimeToTimeT(std::filesystem::file_time_type ft)
{
  // Convert filesystem clock -> system_clock.
  // This is the common (portable-ish) technique.
  using file_clock = std::filesystem::file_time_type::clock;
  const auto sctp = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
    ft - file_clock::now() + std::chrono::system_clock::now());
  return std::chrono::system_clock::to_time_t(sctp);
}

} // namespace

ZipWriter::~ZipWriter()
{
  close();
}

bool ZipWriter::open(const std::filesystem::path& path, std::string& outError, const ZipWriterOptions& opt)
{
  outError.clear();
  close();

  if (path.empty()) {
    outError = "ZipWriter path is empty";
    return false;
  }

  std::error_code ec;
  if (std::filesystem::exists(path, ec) && !ec) {
    if (!opt.overwrite) {
      outError = "ZipWriter target already exists: " + path.string();
      return false;
    }
    std::filesystem::remove(path, ec);
    // ignore removal errors; we'll fail on open below if it still exists.
  }

  // Ensure parent directory exists.
  const std::filesystem::path parent = path.parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent, ec);
    if (ec) {
      outError = "ZipWriter unable to create directory: " + parent.string() + " (" + ec.message() + ")";
      return false;
    }
  }

  auto* ofs = new std::ofstream(path, std::ios::binary | std::ios::trunc);
  if (!(*ofs)) {
    delete ofs;
    outError = "ZipWriter unable to open file: " + path.string();
    return false;
  }

  m_path = path;
  m_ofsPtr = ofs;
  m_open = true;
  m_finalized = false;
  m_entries.clear();
  return true;
}

void ZipWriter::close()
{
  if (m_ofsPtr) {
    try {
      m_ofsPtr->close();
    } catch (...) {
      // ignore
    }
    delete m_ofsPtr;
    m_ofsPtr = nullptr;
  }
  m_open = false;
  m_finalized = false;
  m_entries.clear();
  m_path.clear();
}

void ZipWriter::dosTimeDateNowUtc(std::uint16_t& outTime, std::uint16_t& outDate)
{
  dosTimeDateFromTimeT(std::time(nullptr), outTime, outDate);
}

void ZipWriter::dosTimeDateFromTimeT(std::time_t t, std::uint16_t& outTime, std::uint16_t& outDate)
{
  std::tm tm{};
#if defined(_WIN32)
  gmtime_s(&tm, &t);
#else
  gmtime_r(&t, &tm);
#endif

  int year = tm.tm_year + 1900;
  if (year < 1980) year = 1980; // DOS time starts at 1980.
  if (year > 2107) year = 2107; // 7 bits.

  const int month = std::clamp(tm.tm_mon + 1, 1, 12);
  const int day = std::clamp(tm.tm_mday, 1, 31);
  const int hour = std::clamp(tm.tm_hour, 0, 23);
  const int minute = std::clamp(tm.tm_min, 0, 59);
  const int sec2 = std::clamp(tm.tm_sec / 2, 0, 29);

  outDate = static_cast<std::uint16_t>(((year - 1980) << 9) | (month << 5) | day);
  outTime = static_cast<std::uint16_t>((hour << 11) | (minute << 5) | sec2);
}

bool ZipWriter::sanitizeZipPath(const std::string& in, std::string& out, std::string& outError)
{
  outError.clear();
  out.clear();
  if (in.empty()) {
    outError = "zip path is empty";
    return false;
  }

  // Normalize slashes.
  std::string norm = in;
  for (char& c : norm) {
    if (c == '\\') c = '/';
  }

  // Strip leading slashes.
  while (!norm.empty() && norm.front() == '/') {
    norm.erase(norm.begin());
  }

  // Split and validate.
  std::vector<std::string> parts;
  std::string cur;
  for (char c : norm) {
    if (c == '/') {
      if (!cur.empty()) {
        parts.push_back(cur);
        cur.clear();
      }
    } else {
      cur.push_back(c);
    }
  }
  if (!cur.empty()) parts.push_back(cur);

  if (parts.empty()) {
    outError = "zip path is empty after normalization";
    return false;
  }

  for (const auto& p : parts) {
    if (p == "." || p.empty()) continue;
    if (p == "..") {
      outError = "zip path contains '..' segment (blocked): " + in;
      return false;
    }
  }

  // Rebuild.
  std::ostringstream oss;
  bool first = true;
  for (const auto& p : parts) {
    if (p == "." || p.empty()) continue;
    if (!first) oss << '/';
    first = false;
    oss << p;
  }

  out = oss.str();
  if (out.empty()) {
    outError = "zip path is empty after sanitization";
    return false;
  }
  // No directory entries.
  if (!out.empty() && out.back() == '/') {
    out.pop_back();
  }
  return true;
}

bool ZipWriter::addFileFromBytes(const std::string& zipPath, const std::uint8_t* data, std::size_t size, std::string& outError)
{
  outError.clear();
  if (!m_open || !m_ofsPtr) {
    outError = "ZipWriter is not open";
    return false;
  }
  if (m_finalized) {
    outError = "ZipWriter already finalized";
    return false;
  }

  std::string name;
  if (!sanitizeZipPath(zipPath, name, outError)) {
    return false;
  }

  if (!data && size > 0) {
    outError = "ZipWriter addFileFromBytes called with null data";
    return false;
  }

  if (size > std::numeric_limits<std::uint32_t>::max()) {
    outError = "ZipWriter entry too large (ZIP64 not supported): " + name;
    return false;
  }

  std::ofstream& ofs = *m_ofsPtr;

  Entry e;
  e.name = name;
  e.flags = kFlagDataDescriptor;
  e.method = kMethodStore;
  dosTimeDateNowUtc(e.dosTime, e.dosDate);
  e.localHeaderOffset = TellpU32(ofs);

  // Local file header (sizes/CRC deferred via data descriptor).
  WriteLe32(ofs, kSigLocalHeader);
  WriteLe16(ofs, kVersionNeeded);
  WriteLe16(ofs, e.flags);
  WriteLe16(ofs, e.method);
  WriteLe16(ofs, e.dosTime);
  WriteLe16(ofs, e.dosDate);
  WriteLe32(ofs, 0); // crc32
  WriteLe32(ofs, 0); // comp size
  WriteLe32(ofs, 0); // uncomp size
  WriteLe16(ofs, static_cast<std::uint16_t>(e.name.size()));
  WriteLe16(ofs, 0); // extra len
  ofs.write(e.name.data(), static_cast<std::streamsize>(e.name.size()));
  if (!ofs.good()) {
    outError = "ZipWriter write failed (local header)";
    return false;
  }

  // Data
  std::uint32_t crc = 0xFFFFFFFFu;
  if (size > 0) {
    crc = Crc32Update(crc, data, size);
    ofs.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(size));
  }
  if (!ofs.good()) {
    outError = "ZipWriter write failed (data)";
    return false;
  }

  e.uncompSize = static_cast<std::uint32_t>(size);
  e.compSize = static_cast<std::uint32_t>(size);
  e.crc32 = crc ^ 0xFFFFFFFFu;

  // Data descriptor
  WriteLe32(ofs, kSigDataDescriptor);
  WriteLe32(ofs, e.crc32);
  WriteLe32(ofs, e.compSize);
  WriteLe32(ofs, e.uncompSize);
  if (!ofs.good()) {
    outError = "ZipWriter write failed (data descriptor)";
    return false;
  }

  m_entries.push_back(e);
  return true;
}

bool ZipWriter::addFileFromPath(const std::string& zipPath, const std::filesystem::path& srcPath, std::string& outError)
{
  outError.clear();
  if (!m_open || !m_ofsPtr) {
    outError = "ZipWriter is not open";
    return false;
  }
  if (m_finalized) {
    outError = "ZipWriter already finalized";
    return false;
  }

  if (srcPath.empty()) {
    outError = "ZipWriter source path is empty";
    return false;
  }

  std::error_code ec;
  if (!std::filesystem::exists(srcPath, ec) || ec) {
    outError = "ZipWriter source does not exist: " + srcPath.string();
    return false;
  }

  // Refuse very large files up-front (we do not implement ZIP64).
  {
    const auto sz = std::filesystem::file_size(srcPath, ec);
    if (!ec && sz > static_cast<std::uintmax_t>(std::numeric_limits<std::uint32_t>::max())) {
      outError = "ZipWriter entry too large (ZIP64 not supported): " + srcPath.string();
      return false;
    }
  }

  std::string name;
  if (!sanitizeZipPath(zipPath, name, outError)) {
    return false;
  }

  // Time
  std::uint16_t dosTime = 0, dosDate = 0;
  {
    std::time_t tt = std::time(nullptr);
    const auto ft = std::filesystem::last_write_time(srcPath, ec);
    if (!ec) {
      tt = FileTimeToTimeT(ft);
    }
    dosTimeDateFromTimeT(tt, dosTime, dosDate);
  }

  std::ifstream ifs(srcPath, std::ios::in | std::ios::binary);
  if (!ifs) {
    outError = "ZipWriter unable to open source file: " + srcPath.string();
    return false;
  }

  std::ofstream& ofs = *m_ofsPtr;

  Entry e;
  e.name = name;
  e.flags = kFlagDataDescriptor;
  e.method = kMethodStore;
  e.dosTime = dosTime;
  e.dosDate = dosDate;
  e.localHeaderOffset = TellpU32(ofs);

  // Local header
  WriteLe32(ofs, kSigLocalHeader);
  WriteLe16(ofs, kVersionNeeded);
  WriteLe16(ofs, e.flags);
  WriteLe16(ofs, e.method);
  WriteLe16(ofs, e.dosTime);
  WriteLe16(ofs, e.dosDate);
  WriteLe32(ofs, 0);
  WriteLe32(ofs, 0);
  WriteLe32(ofs, 0);
  if (e.name.size() > std::numeric_limits<std::uint16_t>::max()) {
    outError = "ZipWriter entry name too long";
    return false;
  }
  WriteLe16(ofs, static_cast<std::uint16_t>(e.name.size()));
  WriteLe16(ofs, 0);
  ofs.write(e.name.data(), static_cast<std::streamsize>(e.name.size()));
  if (!ofs.good()) {
    outError = "ZipWriter write failed (local header)";
    return false;
  }

  // Stream file content.
  std::uint32_t crc = 0xFFFFFFFFu;
  std::uint64_t total = 0;
  std::array<char, 64 * 1024> buf{};

  while (ifs) {
    ifs.read(buf.data(), static_cast<std::streamsize>(buf.size()));
    const std::streamsize got = ifs.gcount();
    if (got <= 0) break;
    total += static_cast<std::uint64_t>(got);
    if (total > std::numeric_limits<std::uint32_t>::max()) {
      outError = "ZipWriter entry too large (ZIP64 not supported): " + srcPath.string();
      return false;
    }
    crc = Crc32Update(crc, reinterpret_cast<const std::uint8_t*>(buf.data()), static_cast<std::size_t>(got));
    ofs.write(buf.data(), got);
    if (!ofs.good()) {
      outError = "ZipWriter write failed (data)";
      return false;
    }
  }

  if (!ifs.eof() && ifs.fail()) {
    outError = "ZipWriter read failed: " + srcPath.string();
    return false;
  }

  e.uncompSize = static_cast<std::uint32_t>(total);
  e.compSize = static_cast<std::uint32_t>(total);
  e.crc32 = crc ^ 0xFFFFFFFFu;

  // Data descriptor
  WriteLe32(ofs, kSigDataDescriptor);
  WriteLe32(ofs, e.crc32);
  WriteLe32(ofs, e.compSize);
  WriteLe32(ofs, e.uncompSize);
  if (!ofs.good()) {
    outError = "ZipWriter write failed (data descriptor)";
    return false;
  }

  m_entries.push_back(e);
  return true;
}

bool ZipWriter::writeCentralDirectory(std::string& outError)
{
  outError.clear();
  if (!m_open || !m_ofsPtr) {
    outError = "ZipWriter is not open";
    return false;
  }

  if (m_entries.size() > std::numeric_limits<std::uint16_t>::max()) {
    outError = "ZipWriter too many entries (ZIP64 not supported)";
    return false;
  }

  std::ofstream& ofs = *m_ofsPtr;
  const std::uint32_t cdOffset = TellpU32(ofs);

  for (const Entry& e : m_entries) {
    WriteLe32(ofs, kSigCentralHeader);
    WriteLe16(ofs, kVersionMadeBy);
    WriteLe16(ofs, kVersionNeeded);
    WriteLe16(ofs, e.flags);
    WriteLe16(ofs, e.method);
    WriteLe16(ofs, e.dosTime);
    WriteLe16(ofs, e.dosDate);
    WriteLe32(ofs, e.crc32);
    WriteLe32(ofs, e.compSize);
    WriteLe32(ofs, e.uncompSize);
    if (e.name.size() > std::numeric_limits<std::uint16_t>::max()) {
      outError = "ZipWriter entry name too long";
      return false;
    }
    WriteLe16(ofs, static_cast<std::uint16_t>(e.name.size()));
    WriteLe16(ofs, 0); // extra
    WriteLe16(ofs, 0); // comment
    WriteLe16(ofs, 0); // disk
    WriteLe16(ofs, 0); // int attrs
    WriteLe32(ofs, 0); // ext attrs
    WriteLe32(ofs, e.localHeaderOffset);
    ofs.write(e.name.data(), static_cast<std::streamsize>(e.name.size()));
    if (!ofs.good()) {
      outError = "ZipWriter write failed (central directory)";
      return false;
    }
  }

  const std::uint32_t cdEnd = TellpU32(ofs);
  const std::uint32_t cdSize = (cdEnd >= cdOffset) ? (cdEnd - cdOffset) : 0;

  // End of central directory record.
  WriteLe32(ofs, kSigEndOfCentral);
  WriteLe16(ofs, 0); // disk
  WriteLe16(ofs, 0); // start disk
  WriteLe16(ofs, static_cast<std::uint16_t>(m_entries.size()));
  WriteLe16(ofs, static_cast<std::uint16_t>(m_entries.size()));
  WriteLe32(ofs, cdSize);
  WriteLe32(ofs, cdOffset);
  WriteLe16(ofs, 0); // comment

  if (!ofs.good()) {
    outError = "ZipWriter write failed (end of central directory)";
    return false;
  }

  ofs.flush();
  if (!ofs.good()) {
    outError = "ZipWriter flush failed";
    return false;
  }
  return true;
}

bool ZipWriter::finalize(std::string& outError)
{
  outError.clear();
  if (!m_open || !m_ofsPtr) {
    outError = "ZipWriter is not open";
    return false;
  }
  if (m_finalized) {
    return true;
  }
  if (!writeCentralDirectory(outError)) {
    return false;
  }
  m_finalized = true;
  return true;
}

} // namespace isocity
