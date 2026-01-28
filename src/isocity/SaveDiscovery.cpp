#include "isocity/SaveDiscovery.hpp"

#include <algorithm>
#include <sstream>

namespace isocity {

namespace {

constexpr const char* kLegacyQuickSavePath = "isocity_save.bin";

std::filesystem::path ManualSlotPath(const std::filesystem::path& dir, int slot)
{
  if (slot <= 1) {
    return dir / kLegacyQuickSavePath;
  }
  return dir / (std::string("isocity_save_slot") + std::to_string(slot) + ".bin");
}

std::filesystem::path AutosaveSlotPath(const std::filesystem::path& dir, int slot)
{
  return dir / (std::string("isocity_autosave_slot") + std::to_string(slot) + ".bin");
}

void AppendFsError(std::string& ioErr, const std::filesystem::path& p, const std::error_code& ec)
{
  if (!ec) return;
  if (!ioErr.empty()) ioErr += "\n";
  ioErr += p.string();
  ioErr += ": ";
  ioErr += ec.message();
}

void TryAdd(SaveScanResult& io, const std::filesystem::path& p, SaveKind kind, int slot)
{
  // ProcIsoCity writes a temp file and/or backup during save transactions.
  // If the app crashed mid-save, the canonical slot file may be missing or stale,
  // while a newer ".tmp" (or ".bak") is present. For "--resume" and diagnostics,
  // treat the most recently modified of {p, p.tmp, p.bak} as the representative
  // save artifact for this slot.

  struct Candidate {
    std::filesystem::path path;
    std::filesystem::file_time_type timestamp{};
  };

  std::optional<Candidate> best;

  auto consider = [&](const std::filesystem::path& fp) {
    std::error_code ec;

    const bool exists = std::filesystem::exists(fp, ec);
    if (ec) {
      AppendFsError(io.err, fp, ec);
      return;
    }
    if (!exists) return;

    const bool isFile = std::filesystem::is_regular_file(fp, ec);
    if (ec) {
      AppendFsError(io.err, fp, ec);
      return;
    }
    if (!isFile) return;

    const auto t = std::filesystem::last_write_time(fp, ec);
    if (ec) {
      AppendFsError(io.err, fp, ec);
      return;
    }

    if (!best || t > best->timestamp) {
      best = Candidate{fp, t};
    }
  };

  consider(p);

  std::filesystem::path tmp = p;
  tmp += ".tmp";
  consider(tmp);

  std::filesystem::path bak = p;
  bak += ".bak";
  consider(bak);

  if (!best) return;

  SaveCandidate c;
  c.path = best->path;
  c.kind = kind;
  c.slot = slot;
  c.timestamp = best->timestamp;
  io.found.push_back(std::move(c));
}

} // namespace

SaveScanResult ScanKnownSaveFiles(const std::filesystem::path& dir, int manualSlotsMax, int autosaveSlotsMax)
{
  SaveScanResult r;

  const int manualMax = std::max(1, manualSlotsMax);
  for (int slot = 1; slot <= manualMax; ++slot) {
    TryAdd(r, ManualSlotPath(dir, slot), SaveKind::Manual, slot);
  }

  const int autoMax = std::max(0, autosaveSlotsMax);
  for (int slot = 1; slot <= autoMax; ++slot) {
    TryAdd(r, AutosaveSlotPath(dir, slot), SaveKind::Autosave, slot);
  }

  // Sort newest-first for convenience.
  std::sort(r.found.begin(), r.found.end(), [](const SaveCandidate& a, const SaveCandidate& b) {
    if (a.timestamp != b.timestamp) return a.timestamp > b.timestamp;
    // Tie-breaker: prefer manual saves if timestamps match.
    if (a.kind != b.kind) return a.kind == SaveKind::Manual;
    return a.slot < b.slot;
  });

  return r;
}

std::optional<SaveCandidate> FindMostRecentSave(const std::filesystem::path& dir, int manualSlotsMax, int autosaveSlotsMax)
{
  SaveScanResult scan = ScanKnownSaveFiles(dir, manualSlotsMax, autosaveSlotsMax);
  if (scan.found.empty()) return std::nullopt;
  return scan.found.front();
}

std::string SaveKindToString(SaveKind k)
{
  switch (k) {
  case SaveKind::Manual: return "manual";
  case SaveKind::Autosave: return "autosave";
  default: return "unknown";
  }
}

} // namespace isocity
