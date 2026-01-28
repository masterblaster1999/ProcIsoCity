#include "isocity/StackTrace.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#if defined(_WIN32)
  #ifndef NOMINMAX
    #define NOMINMAX
  #endif
  #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
  #endif
  #include <windows.h>
  // dbghelp is optional but provides symbolization. If the platform doesn't
  // ship it, we still fall back to raw addresses.
  #include <dbghelp.h>
#else
  #include <execinfo.h>
  #include <cxxabi.h>
#endif

namespace isocity {

namespace {

#if !defined(_WIN32)

static std::string DemangleMaybe(const char* name)
{
  if (!name || !*name) return std::string();

  int status = 0;
  std::size_t len = 0;
  char* dem = abi::__cxa_demangle(name, nullptr, &len, &status);
  if (status == 0 && dem) {
    std::string out(dem);
    std::free(dem);
    return out;
  }

  if (dem) std::free(dem);
  return std::string(name);
}

static std::string TryDemangleBacktraceSymbolsLine(const std::string& line)
{
  // Common glibc format:
  //   ./a.out(_Z3foov+0x15) [0x400636]
  // We attempt to demangle the token between '(' and '+'. If it doesn't match,
  // return the original line.
  const std::size_t lparen = line.find('(');
  if (lparen == std::string::npos) return line;

  const std::size_t plus = line.find('+', lparen + 1);
  if (plus == std::string::npos) return line;

  const std::size_t rparen = line.find(')', plus + 1);
  if (rparen == std::string::npos) return line;

  const std::string mangled = line.substr(lparen + 1, plus - (lparen + 1));
  if (mangled.empty()) return line;

  const std::string dem = DemangleMaybe(mangled.c_str());
  if (dem.empty() || dem == mangled) return line;

  std::string out = line;
  out.replace(lparen + 1, mangled.size(), dem);
  return out;
}

#endif

} // namespace

std::string CaptureStackTrace(const StackTraceOptions& opt)
{
  const int maxFrames = std::clamp(opt.maxFrames, 0, 256);
  if (maxFrames <= 0) return std::string();

#if defined(_WIN32)
  // CaptureStackBackTrace is supported on modern Windows and is a convenient
  // best-effort way to walk the stack. Symbol resolution is best-effort and
  // depends on debug symbols / PDB availability.
  std::vector<void*> stack(static_cast<std::size_t>(maxFrames));
  const ULONG skip = static_cast<ULONG>(std::max(0, opt.skipFrames));
  const USHORT captured = ::CaptureStackBackTrace(skip, static_cast<ULONG>(maxFrames), stack.data(), nullptr);
  if (captured == 0) return std::string();

  // Best effort symbolization using DbgHelp.
  HANDLE process = ::GetCurrentProcess();
  const bool symOk = (::SymInitialize(process, nullptr, TRUE) != FALSE);
  if (symOk) {
    ::SymSetOptions(SYMOPT_UNDNAME | SYMOPT_DEFERRED_LOADS);
  }

  std::ostringstream oss;
  oss << "stacktrace (" << captured << " frames)\n";

  // SYMBOL_INFO is a variable-sized struct.
  constexpr std::size_t kMaxNameLen = 255;
  std::vector<std::uint8_t> symBuf;
  SYMBOL_INFO* sym = nullptr;
  if (symOk) {
    symBuf.resize(sizeof(SYMBOL_INFO) + kMaxNameLen + 1);
    sym = reinterpret_cast<SYMBOL_INFO*>(symBuf.data());
    std::memset(sym, 0, symBuf.size());
    sym->MaxNameLen = static_cast<ULONG>(kMaxNameLen);
    sym->SizeOfStruct = sizeof(SYMBOL_INFO);
  }

  for (USHORT i = 0; i < captured; ++i) {
    const DWORD64 addr = reinterpret_cast<DWORD64>(stack[i]);
    oss << "  #" << i << "  0x" << std::hex << addr << std::dec;

    if (symOk && sym) {
      DWORD64 disp = 0;
      if (::SymFromAddr(process, addr, &disp, sym)) {
        oss << "  " << sym->Name;
        if (disp) {
          oss << " +0x" << std::hex << disp << std::dec;
        }
      }
    }

    oss << "\n";
  }

  if (symOk) {
    ::SymCleanup(process);
  }

  return oss.str();
#else
  std::vector<void*> buffer(static_cast<std::size_t>(maxFrames));
  const int frames = ::backtrace(buffer.data(), maxFrames);
  if (frames <= 0) return std::string();

  const int skip = std::max(0, opt.skipFrames);
  const int begin = std::min(skip, frames);

  char** syms = ::backtrace_symbols(buffer.data(), frames);
  if (!syms) return std::string();

  std::ostringstream oss;
  oss << "stacktrace (" << (frames - begin) << " frames)\n";
  for (int i = begin; i < frames; ++i) {
    const std::string line = syms[i] ? std::string(syms[i]) : std::string("(null)");
    oss << "  #" << (i - begin) << "  " << TryDemangleBacktraceSymbolsLine(line) << "\n";
  }

  std::free(syms);
  return oss.str();
#endif
}

} // namespace isocity
