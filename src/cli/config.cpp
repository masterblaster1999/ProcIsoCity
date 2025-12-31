#include "isocity/ConfigIO.hpp"
#include "isocity/SaveLoad.hpp"

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace {

using namespace isocity;

static void PrintHelp()
{
  std::cout
      << "proc_isocity_config (save config dump/apply tool)\n\n"
      << "Usage:\n"
      << "  proc_isocity_config dump  <save.bin> [options]\n"
      << "  proc_isocity_config apply <in.bin> <out.bin> [options]\n\n"
      << "Dump options:\n"
      << "  --proc <out_proc.json>   Write ProcGenConfig JSON\n"
      << "  --sim  <out_sim.json>    Write SimConfig JSON\n"
      << "  --all  <out.json>        Write combined JSON: {\\\"proc\\\":{...},\\\"sim\\\":{...}}\n"
      << "  --quiet                  Suppress stdout (errors still print)\n\n"
      << "Apply options:\n"
      << "  --proc <proc.json>       Merge ProcGenConfig overrides from JSON\n"
      << "  --sim  <sim.json>        Merge SimConfig overrides from JSON\n"
      << "  --all  <config.json>     Merge combined overrides with keys 'proc' and/or 'sim'\n"
      << "  --reset-proc             Reset ProcGenConfig to defaults before applying overrides\n"
      << "  --reset-sim              Reset SimConfig to defaults before applying overrides\n"
      << "  --quiet                  Suppress stdout summary\n\n"
      << "Notes:\n"
      << "  - JSON input uses merge semantics: missing keys leave the existing config unchanged.\n"
      << "  - This tool does not modify tiles directly; it only updates the embedded configs\n"
      << "    stored inside the save.\n";
}

static bool ReadFileText(const std::string& path, std::string& out)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  std::ostringstream oss;
  oss << f.rdbuf();
  out = oss.str();
  return true;
}

static bool WriteFileText(const std::string& path, const std::string& text)
{
  std::ofstream f(path, std::ios::binary);
  if (!f) return false;
  f << text;
  return static_cast<bool>(f);
}

static bool LoadAndApplyCombinedOverrides(const std::string& path, ProcGenConfig& ioProc, SimConfig& ioSim,
                                         std::string& outErr)
{
  std::string text;
  if (!ReadFileText(path, text)) {
    outErr = "failed to read file";
    return false;
  }

  JsonValue root;
  std::string err;
  if (!ParseJson(text, root, err)) {
    outErr = err;
    return false;
  }
  if (!root.isObject()) {
    outErr = "combined config JSON must be an object";
    return false;
  }

  const JsonValue* proc = FindJsonMember(root, "proc");
  if (proc) {
    if (!proc->isObject()) {
      outErr = "proc must be an object";
      return false;
    }
    if (!ApplyProcGenConfigJson(*proc, ioProc, err)) {
      outErr = std::string("proc: ") + err;
      return false;
    }
  }

  const JsonValue* sim = FindJsonMember(root, "sim");
  if (sim) {
    if (!sim->isObject()) {
      outErr = "sim must be an object";
      return false;
    }
    if (!ApplySimConfigJson(*sim, ioSim, err)) {
      outErr = std::string("sim: ") + err;
      return false;
    }
  }

  outErr.clear();
  return true;
}

} // namespace

int main(int argc, char** argv)
{
  if (argc < 2) {
    PrintHelp();
    return 1;
  }

  const std::string mode = argv[1] ? std::string(argv[1]) : std::string();
  if (mode == "-h" || mode == "--help" || mode == "help") {
    PrintHelp();
    return 0;
  }

  if (mode == "dump") {
    if (argc < 3) {
      PrintHelp();
      return 2;
    }

    const std::string inPath = argv[2];
    std::string outProc;
    std::string outSim;
    std::string outAll;
    bool quiet = false;

    for (int i = 3; i < argc; ++i) {
      const std::string a = argv[i] ? std::string(argv[i]) : std::string();
      if (a == "--proc") {
        if (i + 1 >= argc) {
          std::cerr << "--proc requires a path\n";
          return 2;
        }
        outProc = argv[++i];
      } else if (a == "--sim") {
        if (i + 1 >= argc) {
          std::cerr << "--sim requires a path\n";
          return 2;
        }
        outSim = argv[++i];
      } else if (a == "--all") {
        if (i + 1 >= argc) {
          std::cerr << "--all requires a path\n";
          return 2;
        }
        outAll = argv[++i];
      } else if (a == "--quiet") {
        quiet = true;
      } else if (a == "-h" || a == "--help") {
        PrintHelp();
        return 0;
      } else {
        std::cerr << "Unknown option: " << a << "\n";
        return 2;
      }
    }

    World w;
    ProcGenConfig proc;
    SimConfig sim;
    std::string err;

    if (!LoadWorldBinary(w, proc, sim, inPath, err)) {
      std::cerr << "Load failed: " << err << "\n";
      return 3;
    }

    if (!outProc.empty()) {
      if (!WriteProcGenConfigJsonFile(outProc, proc, err)) {
        std::cerr << "Write proc JSON failed: " << err << "\n";
        return 3;
      }
    }

    if (!outSim.empty()) {
      if (!WriteSimConfigJsonFile(outSim, sim, err)) {
        std::cerr << "Write sim JSON failed: " << err << "\n";
        return 3;
      }
    }

    if (!outAll.empty()) {
      const std::string text = CombinedConfigToJson(proc, sim, 2);
      if (!WriteFileText(outAll, text)) {
        std::cerr << "Write combined JSON failed: " << outAll << "\n";
        return 3;
      }
    }

    if (!quiet) {
      if (outProc.empty() && outSim.empty() && outAll.empty()) {
        // Default: print combined JSON.
        std::cout << CombinedConfigToJson(proc, sim, 2);
      } else {
        std::cout << "ok\n";
      }
    }

    return 0;
  }

  if (mode == "apply") {
    if (argc < 4) {
      PrintHelp();
      return 2;
    }

    const std::string inPath = argv[2];
    const std::string outPath = argv[3];

    std::string procJson;
    std::string simJson;
    std::string allJson;
    bool resetProc = false;
    bool resetSim = false;
    bool quiet = false;

    for (int i = 4; i < argc; ++i) {
      const std::string a = argv[i] ? std::string(argv[i]) : std::string();
      if (a == "--proc") {
        if (i + 1 >= argc) {
          std::cerr << "--proc requires a path\n";
          return 2;
        }
        procJson = argv[++i];
      } else if (a == "--sim") {
        if (i + 1 >= argc) {
          std::cerr << "--sim requires a path\n";
          return 2;
        }
        simJson = argv[++i];
      } else if (a == "--all") {
        if (i + 1 >= argc) {
          std::cerr << "--all requires a path\n";
          return 2;
        }
        allJson = argv[++i];
      } else if (a == "--reset-proc") {
        resetProc = true;
      } else if (a == "--reset-sim") {
        resetSim = true;
      } else if (a == "--quiet") {
        quiet = true;
      } else if (a == "-h" || a == "--help") {
        PrintHelp();
        return 0;
      } else {
        std::cerr << "Unknown option: " << a << "\n";
        return 2;
      }
    }

    if (procJson.empty() && simJson.empty() && allJson.empty() && !resetProc && !resetSim) {
      std::cerr << "apply: no changes requested (use --proc/--sim/--all or --reset-*)\n";
      return 2;
    }

    World w;
    ProcGenConfig proc;
    SimConfig sim;
    std::string err;

    if (!LoadWorldBinary(w, proc, sim, inPath, err)) {
      std::cerr << "Load failed: " << err << "\n";
      return 3;
    }

    if (resetProc) proc = ProcGenConfig{};
    if (resetSim) sim = SimConfig{};

    if (!allJson.empty()) {
      if (!LoadAndApplyCombinedOverrides(allJson, proc, sim, err)) {
        std::cerr << "Load --all failed: " << err << "\n";
        return 3;
      }
    }

    if (!procJson.empty()) {
      if (!LoadProcGenConfigJsonFile(procJson, proc, err)) {
        std::cerr << "Load proc JSON failed: " << err << "\n";
        return 3;
      }
    }

    if (!simJson.empty()) {
      if (!LoadSimConfigJsonFile(simJson, sim, err)) {
        std::cerr << "Load sim JSON failed: " << err << "\n";
        return 3;
      }
    }

    if (!SaveWorldBinary(w, proc, sim, outPath, err)) {
      std::cerr << "Save failed: " << err << "\n";
      return 3;
    }

    if (!quiet) {
      std::cout << "Wrote: " << outPath << "\n";
    }

    return 0;
  }

  std::cerr << "Unknown mode: " << mode << "\n";
  PrintHelp();
  return 2;
}
