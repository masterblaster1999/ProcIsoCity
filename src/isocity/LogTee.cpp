#include "isocity/LogTee.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>
#include <system_error>

namespace isocity {

namespace {

static std::filesystem::path RotatedPath(const std::filesystem::path& base, int idx)
{
  if (idx <= 0) return base;
  std::filesystem::path p = base;
  p += ".";
  p += std::to_string(idx);
  return p;
}

class TeeBuf final : public std::streambuf {
public:
  TeeBuf(std::streambuf* console,
         std::streambuf* file,
         std::mutex* m,
         bool* fileAtLineStart,
         const char* streamTag,
         bool prefixLines,
         bool prefixThreadId)
      : m_a(console)
      , m_b(file)
      , m_mutex(m)
      , m_fileAtLineStart(fileAtLineStart)
      , m_streamTag(streamTag ? streamTag : "")
      , m_prefixLines(prefixLines)
      , m_prefixThreadId(prefixThreadId)
  {
  }

protected:
  int overflow(int ch) override
  {
    if (ch == traits_type::eof()) {
      return traits_type::not_eof(ch);
    }
    const char c = static_cast<char>(ch);
    return (xsputn(&c, 1) == 1) ? ch : traits_type::eof();
  }

  std::streamsize xsputn(const char* s, std::streamsize n) override
  {
    if (!m_a || !m_b || n <= 0) return 0;
    std::scoped_lock<std::mutex> lock(*m_mutex);

    const std::streamsize ra = m_a->sputn(s, n);

    // File output optionally prefixes each line with a timestamp + stream tag.
    // This is extremely valuable when players attach logs, and does not affect
    // console output.
    std::streamsize writtenInput = 0;
    if (!m_prefixLines) {
      const std::streamsize rb = m_b->sputn(s, n);
      writtenInput = std::min<std::streamsize>(rb, n);
    } else {
      writtenInput = WriteFileWithPrefixLocked(s, n);
    }

    return std::min(ra, writtenInput);
  }

  int sync() override
  {
    if (!m_a || !m_b) return 0;
    std::scoped_lock<std::mutex> lock(*m_mutex);
    const int ra = m_a->pubsync();
    const int rb = m_b->pubsync();
    return (ra == 0 && rb == 0) ? 0 : -1;
  }

private:
  static std::string TimestampUtcNow()
  {
    using clock = std::chrono::system_clock;
    const auto now = clock::now();
    const auto msSinceEpoch = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    const auto secSinceEpoch = std::chrono::duration_cast<std::chrono::seconds>(msSinceEpoch);
    const auto msRemainder = msSinceEpoch - secSinceEpoch;

    const std::time_t tt = static_cast<std::time_t>(secSinceEpoch.count());
    std::tm tm{};

#if defined(_WIN32)
    gmtime_s(&tm, &tt);
#else
    gmtime_r(&tt, &tm);
#endif

    char buf[64];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d.%03lldZ",
                  tm.tm_year + 1900,
                  tm.tm_mon + 1,
                  tm.tm_mday,
                  tm.tm_hour,
                  tm.tm_min,
                  tm.tm_sec,
                  static_cast<long long>(msRemainder.count()));
    return std::string(buf);
  }

  std::string BuildPrefixLocked() const
  {
    std::ostringstream oss;
    oss << TimestampUtcNow();
    if (!m_streamTag.empty()) {
      oss << " [" << m_streamTag << "]";
    }
    if (m_prefixThreadId) {
      const std::size_t tid = std::hash<std::thread::id>{}(std::this_thread::get_id());
      oss << " [t=0x" << std::hex << tid << std::dec << "]";
    }
    oss << " ";
    return oss.str();
  }

  std::streamsize WriteFileWithPrefixLocked(const char* s, std::streamsize n)
  {
    if (!m_b || !m_fileAtLineStart || n <= 0) return 0;

    const char* p = s;
    const char* end = s + n;
    std::streamsize totalInputWritten = 0;

    while (p < end) {
      if (*m_fileAtLineStart) {
        const std::string prefix = BuildPrefixLocked();
        const std::streamsize pr = m_b->sputn(prefix.data(), static_cast<std::streamsize>(prefix.size()));
        if (pr != static_cast<std::streamsize>(prefix.size())) {
          // Prefix write failed; give up.
          return totalInputWritten;
        }
        *m_fileAtLineStart = false;
      }

      // Write up to and including a newline (or the rest of the buffer).
      const char* nl = static_cast<const char*>(std::memchr(p, '\n', static_cast<std::size_t>(end - p)));
      const std::streamsize chunk = nl ? static_cast<std::streamsize>((nl - p) + 1)
                                       : static_cast<std::streamsize>(end - p);
      const std::streamsize wr = m_b->sputn(p, chunk);
      if (wr <= 0) {
        return totalInputWritten;
      }

      totalInputWritten += std::min(wr, chunk);

      // If we wrote a full chunk that ends in a newline, the next write begins a new line.
      if (wr == chunk && nl) {
        *m_fileAtLineStart = true;

        // Flush on line boundaries so log files remain useful even if the
        // process crashes shortly after emitting diagnostics. This is a small
        // perf cost, but ProcIsoCity's logging is low volume and the added
        // reliability is worth it.
        (void)m_b->pubsync();
      }

      // If we couldn't write the full chunk, stop.
      if (wr < chunk) {
        return totalInputWritten;
      }

      p += chunk;
    }

    return totalInputWritten;
  }

  std::streambuf* m_a = nullptr;
  std::streambuf* m_b = nullptr;
  std::mutex* m_mutex = nullptr;

  bool* m_fileAtLineStart = nullptr;
  std::string m_streamTag;
  bool m_prefixLines = false;
  bool m_prefixThreadId = false;
};

} // namespace

struct LogTee::Impl {
  std::ofstream file;
  std::filesystem::path path;
  std::streambuf* origCout = nullptr;
  std::streambuf* origCerr = nullptr;

  std::mutex mutex;
  bool fileAtLineStart = true;
  std::unique_ptr<TeeBuf> coutBuf;
  std::unique_ptr<TeeBuf> cerrBuf;

  bool prefixLines = true;
  bool prefixThreadId = false;
};

void LogTee::ImplDeleter::operator()(Impl* p) const noexcept
{
  delete p;
}

LogTee::LogTee(const LogTeeOptions& opt, std::string& outError)
{
  (void)start(opt, outError);
}

LogTee::~LogTee()
{
  stop();
}

bool LogTee::active() const
{
  return m_impl != nullptr;
}

const std::filesystem::path& LogTee::path() const
{
  static const std::filesystem::path kEmpty;
  return m_impl ? m_impl->path : kEmpty;
}

bool LogTee::Rotate(const std::filesystem::path& basePath, int keepFiles, std::string& outError)
{
  outError.clear();
  if (keepFiles <= 0) return true;

  std::error_code ec;
  for (int i = keepFiles; i >= 1; --i) {
    const std::filesystem::path dst = RotatedPath(basePath, i);
    const std::filesystem::path src = RotatedPath(basePath, i - 1);

    if (std::filesystem::exists(src, ec)) {
      std::filesystem::remove(dst, ec);
      std::filesystem::rename(src, dst, ec);
      // Best effort: if rename fails (permissions, etc.), report but keep going.
      if (ec) {
        std::ostringstream oss;
        oss << "Failed to rotate log '" << src.string() << "' -> '" << dst.string() << "': " << ec.message();
        outError = oss.str();
        return false;
      }
    }
  }
  return true;
}

bool LogTee::start(const LogTeeOptions& opt, std::string& outError)
{
  outError.clear();
  stop();

  if (opt.path.empty()) {
    outError = "Log path is empty";
    return false;
  }

  // Ensure parent directory exists.
  {
    std::error_code ec;
    const std::filesystem::path parent = opt.path.parent_path();
    if (!parent.empty()) {
      std::filesystem::create_directories(parent, ec);
      if (ec) {
        outError = "Failed to create log directory '" + parent.string() + "': " + ec.message();
        return false;
      }
    }
  }

  if (!Rotate(opt.path, opt.keepFiles, outError)) {
    return false;
  }

  std::unique_ptr<Impl, ImplDeleter> impl(new Impl());
  impl->path = opt.path;
  impl->file.open(opt.path, std::ios::out | std::ios::binary | std::ios::trunc);
  if (!impl->file) {
    outError = "Unable to open log file for writing: " + opt.path.string();
    return false;
  }

  std::streambuf* fileBuf = impl->file.rdbuf();
  if (!fileBuf) {
    outError = "Log file stream buffer is null";
    return false;
  }

  if (opt.teeStdout) {
    impl->origCout = std::cout.rdbuf();
    impl->prefixLines = opt.prefixLines;
    impl->prefixThreadId = opt.prefixThreadId;
    impl->coutBuf = std::make_unique<TeeBuf>(impl->origCout,
                                             fileBuf,
                                             &impl->mutex,
                                             &impl->fileAtLineStart,
                                             "OUT",
                                             impl->prefixLines,
                                             impl->prefixThreadId);
    std::cout.rdbuf(impl->coutBuf.get());
  }

  if (opt.teeStderr) {
    impl->origCerr = std::cerr.rdbuf();
    impl->prefixLines = opt.prefixLines;
    impl->prefixThreadId = opt.prefixThreadId;
    impl->cerrBuf = std::make_unique<TeeBuf>(impl->origCerr,
                                             fileBuf,
                                             &impl->mutex,
                                             &impl->fileAtLineStart,
                                             "ERR",
                                             impl->prefixLines,
                                             impl->prefixThreadId);
    std::cerr.rdbuf(impl->cerrBuf.get());
  }

  m_impl = std::move(impl);
  return true;
}

void LogTee::stop()
{
  if (!m_impl) return;

  // Restore original buffers first so any logging during teardown won't hit a dead file.
  if (m_impl->origCout && std::cout.rdbuf() == m_impl->coutBuf.get()) {
    std::cout.rdbuf(m_impl->origCout);
  }
  if (m_impl->origCerr && std::cerr.rdbuf() == m_impl->cerrBuf.get()) {
    std::cerr.rdbuf(m_impl->origCerr);
  }

  try {
    m_impl->file.flush();
  } catch (...) {
    // Best effort.
  }

  m_impl.reset();
}

} // namespace isocity
