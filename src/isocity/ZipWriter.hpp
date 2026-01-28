#pragma once

#include <cstddef>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <iosfwd>
#include <string>
#include <vector>

namespace isocity {

// Minimal ZIP archive writer ("store" / no compression).
//
// Why this exists:
// - Support bundles are much easier for players to attach as a single .zip file.
// - We want to avoid adding a third-party dependency just for packaging logs.
//
// Design notes:
// - Only the "store" method (compression=0) is supported.
// - ZIP64 is not supported (support bundles should remain small).
// - Filenames are sanitized to prevent "zip slip" paths (no ".." segments).
// - CRC32 is computed using the project's existing Checksum utilities.

struct ZipWriterOptions {
  // If true, overwrite any existing file at `path`.
  bool overwrite = true;
};

class ZipWriter {
public:
  ZipWriter() = default;
  ZipWriter(const ZipWriter&) = delete;
  ZipWriter& operator=(const ZipWriter&) = delete;

  ~ZipWriter();

  bool open(const std::filesystem::path& path, std::string& outError, const ZipWriterOptions& opt = ZipWriterOptions{});

  // Add a file whose content is read from disk.
  //
  // `zipPath` is the path *inside* the archive (must use forward slashes).
  bool addFileFromPath(const std::string& zipPath, const std::filesystem::path& srcPath, std::string& outError);

  // Add an in-memory file.
  bool addFileFromBytes(const std::string& zipPath, const std::uint8_t* data, std::size_t size, std::string& outError);

  // Convenience.
  bool addFileFromString(const std::string& zipPath, const std::string& text, std::string& outError)
  {
    return addFileFromBytes(zipPath, reinterpret_cast<const std::uint8_t*>(text.data()), text.size(), outError);
  }

  // Finish the archive (writes central directory + end-of-central-directory).
  bool finalize(std::string& outError);

  // Abort writing and close the file.
  void close();

  bool active() const { return m_open; }
  const std::filesystem::path& path() const { return m_path; }

private:
  struct Entry {
    std::string name;
    std::uint32_t crc32 = 0;
    std::uint32_t compSize = 0;
    std::uint32_t uncompSize = 0;
    std::uint32_t localHeaderOffset = 0;
    std::uint16_t dosTime = 0;
    std::uint16_t dosDate = 0;
    std::uint16_t flags = 0;
    std::uint16_t method = 0;
  };

  bool writeCentralDirectory(std::string& outError);

  static bool sanitizeZipPath(const std::string& in, std::string& out, std::string& outError);
  static void dosTimeDateNowUtc(std::uint16_t& outTime, std::uint16_t& outDate);
  static void dosTimeDateFromTimeT(std::time_t t, std::uint16_t& outTime, std::uint16_t& outDate);

  bool m_open = false;
  bool m_finalized = false;
  std::filesystem::path m_path;
  std::ofstream* m_ofsPtr = nullptr; // pimpl-ish; keeps header light
  std::vector<Entry> m_entries;
};

} // namespace isocity
