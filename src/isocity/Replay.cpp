#include "isocity/Replay.hpp"

#include "isocity/SaveLoad.hpp"
#include "isocity/WorldPatch.hpp"

#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>

namespace isocity {

namespace {

constexpr std::uint8_t kMagic[8] = {'I','S','O','R','E','P','L','\0'};
constexpr std::uint32_t kReplayVersion = 1;

bool ReadExact(std::istream& in, void* dst, std::size_t n)
{
  in.read(reinterpret_cast<char*>(dst), static_cast<std::streamsize>(n));
  return in.good();
}

bool WriteExact(std::ostream& out, const void* src, std::size_t n)
{
  out.write(reinterpret_cast<const char*>(src), static_cast<std::streamsize>(n));
  return out.good();
}

template <typename T>
bool ReadPOD(std::istream& in, T& out)
{
  return ReadExact(in, &out, sizeof(out));
}

template <typename T>
bool WritePOD(std::ostream& out, const T& v)
{
  return WriteExact(out, &v, sizeof(v));
}

bool ReadBytes(std::istream& in, std::vector<std::uint8_t>& outBytes, std::uint32_t n)
{
  outBytes.clear();
  outBytes.resize(n);
  if (n == 0) return true;
  return ReadExact(in, outBytes.data(), outBytes.size());
}

bool WriteBytes(std::ostream& out, const std::vector<std::uint8_t>& bytes)
{
  if (bytes.empty()) return true;
  return WriteExact(out, bytes.data(), bytes.size());
}

// Load an embedded save blob into a World.
//
// SaveLoad now supports in-memory loading, so replay playback can avoid temp files.
bool LoadWorldFromSaveBytes(const std::vector<std::uint8_t>& saveBytes, World& outWorld, ProcGenConfig& outProc,
                            SimConfig& outSim, std::string& outError)
{
  outError.clear();
  if (saveBytes.empty()) {
    outError = "Embedded save blob is empty";
    return false;
  }

  return LoadWorldBinaryFromBytes(outWorld, outProc, outSim, saveBytes, outError);
}


} // namespace

bool SaveReplayBinary(const Replay& replay, const std::string& path, std::string& outError)
{
  outError.clear();

  if (path.empty()) {
    outError = "Empty replay path";
    return false;
  }
  if (replay.baseSave.empty()) {
    outError = "Replay has no base save";
    return false;
  }

  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out) {
    outError = "Unable to open replay for writing: " + path;
    return false;
  }

  if (!WriteExact(out, kMagic, sizeof(kMagic))) {
    outError = "Write failed (magic)";
    return false;
  }

  const std::uint32_t version = kReplayVersion;
  if (!WritePOD(out, version)) {
    outError = "Write failed (version)";
    return false;
  }

  if (replay.baseSave.size() > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
    outError = "Base save blob is too large";
    return false;
  }
  const std::uint32_t baseSize = static_cast<std::uint32_t>(replay.baseSave.size());
  if (!WritePOD(out, baseSize) || !WriteBytes(out, replay.baseSave)) {
    outError = "Write failed (base save blob)";
    return false;
  }

  for (const ReplayEvent& e : replay.events) {
    const std::uint8_t type = static_cast<std::uint8_t>(e.type);
    if (!WritePOD(out, type)) {
      outError = "Write failed (event type)";
      return false;
    }

    if (e.type == ReplayEventType::Tick) {
      if (!WritePOD(out, e.ticks)) {
        outError = "Write failed (tick event)";
        return false;
      }
      continue;
    }

    const std::vector<std::uint8_t>& blob = (e.type == ReplayEventType::Patch) ? e.patch : e.snapshot;
    if (blob.size() > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
      outError = "Replay event blob is too large";
      return false;
    }
    const std::uint32_t sz = static_cast<std::uint32_t>(blob.size());
    if (!WritePOD(out, sz) || !WriteBytes(out, blob)) {
      outError = "Write failed (event blob)";
      return false;
    }
  }

  return true;
}

bool LoadReplayBinary(Replay& outReplay, const std::string& path, std::string& outError)
{
  outError.clear();
  outReplay = {};

  if (path.empty()) {
    outError = "Empty replay path";
    return false;
  }

  std::ifstream in(path, std::ios::binary);
  if (!in) {
    outError = "Unable to open replay for reading: " + path;
    return false;
  }

  std::uint8_t magic[8]{};
  if (!ReadExact(in, magic, sizeof(magic)) || std::memcmp(magic, kMagic, sizeof(kMagic)) != 0) {
    outError = "Bad replay magic (not an ISOREPL file)";
    return false;
  }

  std::uint32_t version = 0;
  if (!ReadPOD(in, version)) {
    outError = "Read failed (version)";
    return false;
  }
  if (version != kReplayVersion) {
    outError = "Unsupported replay version";
    return false;
  }
  outReplay.version = version;

  std::uint32_t baseSize = 0;
  if (!ReadPOD(in, baseSize)) {
    outError = "Read failed (base size)";
    return false;
  }
  if (!ReadBytes(in, outReplay.baseSave, baseSize)) {
    outError = "Read failed (base save blob)";
    return false;
  }
  if (outReplay.baseSave.empty()) {
    outError = "Replay contains an empty base save blob";
    return false;
  }

  // Events until EOF.
  while (true) {
    std::uint8_t typeU8 = 0;
    in.read(reinterpret_cast<char*>(&typeU8), 1);
    if (in.eof()) {
      break;
    }
    if (!in.good()) {
      outError = "Read failed (event type)";
      return false;
    }

    const ReplayEventType type = static_cast<ReplayEventType>(typeU8);
    if (type != ReplayEventType::Tick && type != ReplayEventType::Patch && type != ReplayEventType::Snapshot) {
      outError = "Unknown replay event type";
      return false;
    }

    ReplayEvent e;
    e.type = type;

    if (type == ReplayEventType::Tick) {
      if (!ReadPOD(in, e.ticks)) {
        outError = "Read failed (tick event)";
        return false;
      }
    } else {
      std::uint32_t sz = 0;
      if (!ReadPOD(in, sz)) {
        outError = "Read failed (event blob size)";
        return false;
      }

      std::vector<std::uint8_t> blob;
      if (!ReadBytes(in, blob, sz)) {
        outError = "Read failed (event blob)";
        return false;
      }

      if (type == ReplayEventType::Patch) {
        e.patch = std::move(blob);
      } else {
        e.snapshot = std::move(blob);
      }
    }

    outReplay.events.push_back(std::move(e));
  }

  return true;
}

bool PlayReplay(const Replay& replay, World& outWorld, ProcGenConfig& outProcCfg, SimConfig& outSimCfg,
                std::string& outError, bool strictPatches, std::vector<Stats>* outTickStats)
{
  outError.clear();

  if (outTickStats) outTickStats->clear();

  if (replay.baseSave.empty()) {
    outError = "Replay has no base save";
    return false;
  }

  if (!LoadWorldFromSaveBytes(replay.baseSave, outWorld, outProcCfg, outSimCfg, outError)) {
    return false;
  }

  if (outTickStats) outTickStats->push_back(outWorld.stats());

  Simulator sim(outSimCfg);
  sim.resetTimer();

  for (std::size_t i = 0; i < replay.events.size(); ++i) {
    const ReplayEvent& e = replay.events[i];
    switch (e.type) {
      case ReplayEventType::Tick: {
        for (std::uint32_t t = 0; t < e.ticks; ++t) {
          sim.stepOnce(outWorld);
          if (outTickStats) outTickStats->push_back(outWorld.stats());
        }
        break;
      }
      case ReplayEventType::Patch: {
        WorldPatch patch;
        if (!DeserializeWorldPatchBinary(patch, e.patch.data(), e.patch.size(), outError)) {
          outError = "Replay patch event " + std::to_string(i) + " failed to parse: " + outError;
          return false;
        }
        const bool force = !strictPatches;
        if (!ApplyWorldPatch(outWorld, outProcCfg, outSimCfg, patch, outError, force)) {
          outError = "Replay patch event " + std::to_string(i) + " failed to apply: " + outError;
          return false;
        }
        sim.config() = outSimCfg;
        sim.resetTimer();
        break;
      }
      case ReplayEventType::Snapshot: {
        if (!LoadWorldFromSaveBytes(e.snapshot, outWorld, outProcCfg, outSimCfg, outError)) {
          outError = "Replay snapshot event " + std::to_string(i) + " failed to load: " + outError;
          return false;
        }
        sim = Simulator(outSimCfg);
        sim.resetTimer();
        if (outTickStats) outTickStats->push_back(outWorld.stats());
        break;
      }
    }
  }

  return true;
}

} // namespace isocity
