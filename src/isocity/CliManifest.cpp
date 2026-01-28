#include "isocity/CliManifest.hpp"

#include "isocity/Json.hpp"
#include "isocity/FileHash.hpp"
#include "isocity/FileSync.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <system_error>

namespace isocity {

namespace {

bool ReadFileToString(const std::filesystem::path& path, std::string& outText, std::string& outErr)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    outErr = "failed to open file";
    return false;
  }
  std::ostringstream oss;
  oss << f.rdbuf();
  outText = oss.str();
  outErr.clear();
  return true;
}

const JsonValue* FindObjMember(const JsonValue& obj, const char* key)
{
  if (!key) return nullptr;
  return FindJsonMember(obj, std::string(key));
}

bool ReadStringOpt(const JsonValue& obj, const char* key, std::string& out)
{
  const JsonValue* v = FindObjMember(obj, key);
  if (!v) return false;
  if (!v->isString()) return false;
  out = v->stringValue;
  return true;
}

bool ReadI32Opt(const JsonValue& obj, const char* key, int& out)
{
  const JsonValue* v = FindObjMember(obj, key);
  if (!v) return false;
  if (!v->isNumber()) return false;
  const double d = v->numberValue;
  if (!std::isfinite(d)) return false;
  if (d < static_cast<double>(std::numeric_limits<int>::min()) ||
      d > static_cast<double>(std::numeric_limits<int>::max())) {
    return false;
  }
  out = static_cast<int>(d);
  return true;
}

bool ReadU64OptLossy(const JsonValue& obj, const char* key, std::uint64_t& out)
{
  const JsonValue* v = FindObjMember(obj, key);
  if (!v) return false;
  if (!v->isNumber()) return false;
  const double d = v->numberValue;
  if (!std::isfinite(d)) return false;
  if (d < 0.0 || d > static_cast<double>(std::numeric_limits<std::uint64_t>::max())) return false;
  out = static_cast<std::uint64_t>(d);
  return true;
}

bool ParseHexU64(std::string_view s, std::uint64_t& out)
{
  if (s.empty()) return false;
  std::uint64_t v = 0;
  for (char c : s) {
    int d = -1;
    if (c >= '0' && c <= '9') d = c - '0';
    else if (c >= 'a' && c <= 'f') d = 10 + (c - 'a');
    else if (c >= 'A' && c <= 'F') d = 10 + (c - 'A');
    else return false;

    if (v > (std::numeric_limits<std::uint64_t>::max() >> 4)) return false;
    v = (v << 4) | static_cast<std::uint64_t>(d);
  }
  out = v;
  return true;
}

void ReplaceAll(std::string& s, const std::string& from, const std::string& to)
{
  if (from.empty()) return;
  std::size_t pos = 0;
  while ((pos = s.find(from, pos)) != std::string::npos) {
    s.replace(pos, from.size(), to);
    pos += to.size();
  }
}

} // namespace

bool LoadCliRunManifest(const std::filesystem::path& manifestPath,
                        CliRunManifest& out,
                        std::string& outError)
{
  out = CliRunManifest{};
  outError.clear();

  std::string text;
  if (!ReadFileToString(manifestPath, text, outError)) {
    outError = "manifest: " + outError;
    return false;
  }

  JsonValue root;
  if (!ParseJson(text, root, outError)) {
    outError = "manifest JSON parse error: " + outError;
    return false;
  }

  if (!root.isObject()) {
    outError = "manifest root must be an object";
    return false;
  }

  // Top-level metadata (optional).
  ReadStringOpt(root, "tool", out.tool);
  ReadStringOpt(root, "tool_version", out.toolVersion);
  ReadStringOpt(root, "tool_git_sha", out.toolGitSha);
  ReadStringOpt(root, "build_stamp", out.buildStamp);
  ReadStringOpt(root, "cwd", out.cwd);

  ReadI32Opt(root, "run_index", out.runIndex);
  ReadI32Opt(root, "width", out.width);
  ReadI32Opt(root, "height", out.height);
  ReadI32Opt(root, "days", out.days);
  ReadStringOpt(root, "world_hash", out.worldHashHex);
  ReadStringOpt(root, "load", out.loadPath);

  // Seeds: prefer hex strings (lossless). Fall back to numeric fields (lossy for large u64).
  ReadStringOpt(root, "seed_hex", out.seedHex);
  if (!out.seedHex.empty()) {
    std::uint64_t v = 0;
    if (ParseHexU64(out.seedHex, v)) {
      out.actualSeed = v;
    }
  }
  if (out.actualSeed == 0) {
    ReadU64OptLossy(root, "actual_seed", out.actualSeed);
  }
  ReadU64OptLossy(root, "requested_seed", out.requestedSeed);

  // argv (optional)
  if (const JsonValue* av = FindObjMember(root, "argv")) {
    if (av->isArray()) {
      out.argv.reserve(av->arrayValue.size());
      for (const auto& e : av->arrayValue) {
        if (e.isString()) out.argv.push_back(e.stringValue);
      }
    }
  }

  // Artifacts.
  const JsonValue* arr = FindObjMember(root, "artifacts");
  if (arr && arr->isArray()) {
    out.artifacts.reserve(arr->arrayValue.size());
    for (const auto& e : arr->arrayValue) {
      if (!e.isObject()) continue;
      CliManifestArtifact a;
      ReadStringOpt(e, "kind", a.kind);
      ReadStringOpt(e, "path", a.path);
      ReadStringOpt(e, "layer", a.layer);
      ReadStringOpt(e, "hash_fnv1a64", a.hashFNV1a64Hex);
      ReadStringOpt(e, "hash_error", a.hashError);

      // size_bytes is a number in the manifest.
      {
        const JsonValue* sb = FindObjMember(e, "size_bytes");
        if (sb && sb->isNumber() && std::isfinite(sb->numberValue) && sb->numberValue >= 0.0) {
          const double d = sb->numberValue;
          const double maxd = static_cast<double>(std::numeric_limits<std::uint64_t>::max());
          if (d <= maxd) a.sizeBytes = static_cast<std::uint64_t>(d);
        }
      }

      if (!a.kind.empty() && !a.path.empty()) {
        out.artifacts.push_back(std::move(a));
      }
    }
  }

  return true;
}

const CliManifestArtifact* FindFirstArtifactByKind(const CliRunManifest& m,
                                                   std::string_view kind,
                                                   std::string_view layer)
{
  for (const auto& a : m.artifacts) {
    if (a.kind != kind) continue;
    if (!layer.empty() && a.layer != layer) continue;
    return &a;
  }
  return nullptr;
}

std::filesystem::path ResolveManifestArtifactPath(const std::filesystem::path& manifestPath,
                                                  const std::string& artifactPath)
{
  const std::filesystem::path p(artifactPath);
  if (p.is_absolute()) return p;
  const std::filesystem::path base = manifestPath.parent_path();
  if (base.empty()) return p;
  return base / p;
}

std::filesystem::path ResolveManifestArtifactPathSmart(const std::filesystem::path& manifestPath,
                                                      const CliRunManifest& manifest,
                                                      const std::string& artifactPath,
                                                      const std::filesystem::path& invocationCwd,
                                                      std::string* outDebug)
{
  if (outDebug) outDebug->clear();

  const std::filesystem::path p(artifactPath);
  if (p.empty()) {
    if (outDebug) *outDebug = "artifact path is empty";
    return {};
  }

  if (p.is_absolute()) {
    if (outDebug) *outDebug = "artifact path is absolute";
    return p;
  }

  const std::filesystem::path base = manifestPath.parent_path();
  const std::filesystem::path fallback = ResolveManifestArtifactPath(manifestPath, artifactPath);

  struct Cand {
    const char* label;
    std::filesystem::path path;
  };

  std::vector<Cand> cands;
  cands.reserve(4);

  if (!base.empty()) {
    cands.push_back(Cand{"manifest_dir", base / p});

    // Common pattern: manifest is written inside an output directory (e.g. out/manifest.json)
    // and artifact paths were passed as "out/save.bin" relative to the CLI working directory.
    // Joining those naively yields "out/out/save.bin".
    //
    // If the artifact path already starts with the manifest directory name, also try
    // resolving it against the parent of the manifest directory.
    const std::filesystem::path baseName = base.filename();
    if (!baseName.empty()) {
      auto it = p.begin();
      if (it != p.end() && *it == baseName) {
        const std::filesystem::path parent = base.parent_path();
        if (!parent.empty()) {
          cands.push_back(Cand{"manifest_dir_parent", parent / p});
        }
      }
    }
  }

  if (!manifest.cwd.empty()) {
    const std::filesystem::path cwdPath = std::filesystem::path(manifest.cwd);
    if (!cwdPath.empty()) {
      cands.push_back(Cand{"manifest_cwd", cwdPath / p});
    }
  }

  if (!invocationCwd.empty()) {
    cands.push_back(Cand{"invocation_cwd", invocationCwd / p});
  }

  // Deduplicate candidates while preserving order.
  {
    std::vector<Cand> unique;
    unique.reserve(cands.size());
    for (const auto& c : cands) {
      bool dup = false;
      for (const auto& u : unique) {
        if (u.path == c.path) { dup = true; break; }
      }
      if (!dup) unique.push_back(c);
    }
    cands.swap(unique);
  }

  std::filesystem::path chosen = fallback;
  const char* chosenLabel = "fallback";

  std::ostringstream dbg;
  dbg << "resolve_artifact_path: '" << artifactPath << "'\n";
  dbg << "  manifest: " << manifestPath.string() << "\n";
  if (!manifest.cwd.empty()) dbg << "  manifest.cwd: " << manifest.cwd << "\n";
  if (!invocationCwd.empty()) dbg << "  invocation_cwd: " << invocationCwd.string() << "\n";

  for (const auto& c : cands) {
    std::error_code ec;
    const bool exists = std::filesystem::exists(c.path, ec) && !ec;
    dbg << "  try[" << c.label << "]: " << c.path.string() << (exists ? " (exists)" : "")
        << (ec ? (" (error: " + ec.message() + ")") : "") << "\n";
    if (exists) {
      chosen = c.path;
      chosenLabel = c.label;
      break;
    }
  }

  dbg << "  chosen[" << chosenLabel << "]: " << chosen.string() << "\n";
  if (outDebug) *outDebug = dbg.str();
  return chosen;
}

std::string ExpandCliManifestTemplate(const std::string& tmpl, const CliRunManifest& m)
{
  if (tmpl.empty()) return {};
  std::string out = tmpl;
  ReplaceAll(out, "{seed}", std::to_string(m.actualSeed));
  ReplaceAll(out, "{run}", std::to_string(m.runIndex));
  ReplaceAll(out, "{w}", std::to_string(m.width));
  ReplaceAll(out, "{h}", std::to_string(m.height));
  ReplaceAll(out, "{days}", std::to_string(m.days));
  if (!m.worldHashHex.empty()) ReplaceAll(out, "{hash}", m.worldHashHex);
  return out;
}


namespace {

static std::string HexU64NoPrefix(std::uint64_t v)
{
  static const char* kHex = "0123456789abcdef";
  std::string s(16, '0');
  for (int i = 15; i >= 0; --i) {
    s[static_cast<std::size_t>(i)] = kHex[static_cast<std::size_t>(v & 0xFu)];
    v >>= 4;
  }
  return s;
}

static bool WriteTextFile(const std::filesystem::path& path,
                          const std::string& text,
                          std::string& outError)
{
  outError.clear();
  std::ofstream f(path, std::ios::binary | std::ios::trunc);
  if (!f) {
    outError = "failed to open file for write: " + path.string();
    return false;
  }
  f.write(text.data(), static_cast<std::streamsize>(text.size()));
  if (!f) {
    outError = "failed to write file: " + path.string();
    return false;
  }
  f.flush();
  if (!f) {
    outError = "failed to flush file: " + path.string();
    return false;
  }
  return true;
}

static bool RenameOrCopy(const std::filesystem::path& src,
                         const std::filesystem::path& dst,
                         std::string& outError)
{
  outError.clear();

  std::error_code ec;

  // Best-effort: remove destination first so rename can succeed on Windows.
  std::filesystem::remove(dst, ec);
  ec.clear();

  std::filesystem::rename(src, dst, ec);
  if (!ec) return true;

  // Fallback: copy + remove.
  std::error_code ec2;
  std::filesystem::copy_file(src, dst, std::filesystem::copy_options::overwrite_existing, ec2);
  if (ec2) {
    outError = "failed to replace '" + dst.string() + "': rename error '" + ec.message() +
               "', copy fallback error '" + ec2.message() + "'";
    return false;
  }

  std::filesystem::remove(src, ec2);
  if (ec2) {
    outError = "replaced '" + dst.string() + "' but failed to delete temp file '" + src.string() +
               "': " + ec2.message();
  }

  return true;
}

static std::string ReadStringMember(const JsonValue& obj, const char* key)
{
  const JsonValue* v = FindObjMember(obj, key);
  if (!v || !v->isString()) return {};
  return v->stringValue;
}

static bool EnsureArtifactsArray(JsonValue& root, JsonValue*& outArr)
{
  outArr = nullptr;
  if (!root.isObject()) return false;

  JsonValue* v = FindJsonMember(root, std::string("artifacts"));
  if (v) {
    if (!v->isArray()) {
      *v = JsonValue::MakeArray();
    }
    outArr = v;
    return true;
  }

  root.objectValue.emplace_back("artifacts", JsonValue::MakeArray());
  outArr = &root.objectValue.back().second;
  return true;
}

static JsonValue BuildArtifactObject(const CliManifestArtifact& a,
                                    const std::filesystem::path& diskPath)
{
  using isocity::JsonValue;

  JsonValue obj = JsonValue::MakeObject();
  obj.objectValue.emplace_back("kind", JsonValue::MakeString(a.kind));
  obj.objectValue.emplace_back("path", JsonValue::MakeString(a.path));
  if (!a.layer.empty()) obj.objectValue.emplace_back("layer", JsonValue::MakeString(a.layer));

  if (!diskPath.empty()) {
    FileHashInfo info;
    std::string herr;
    if (!ComputeFileHashFNV1a64(diskPath.string(), info, herr)) {
      obj.objectValue.emplace_back("hash_error", JsonValue::MakeString(herr));
    } else {
      obj.objectValue.emplace_back("size_bytes", JsonValue::MakeNumber(static_cast<double>(info.sizeBytes)));
      obj.objectValue.emplace_back("hash_fnv1a64", JsonValue::MakeString(HexU64NoPrefix(info.fnv1a64)));
    }
  } else if (!a.hashError.empty()) {
    obj.objectValue.emplace_back("hash_error", JsonValue::MakeString(a.hashError));
  } else if (!a.hashFNV1a64Hex.empty()) {
    if (a.sizeBytes != 0) obj.objectValue.emplace_back("size_bytes", JsonValue::MakeNumber(static_cast<double>(a.sizeBytes)));
    obj.objectValue.emplace_back("hash_fnv1a64", JsonValue::MakeString(a.hashFNV1a64Hex));
  }

  return obj;
}

static void UpsertArtifactInArray(JsonValue& arr, JsonValue obj, const CliManifestArtifact& a)
{
  if (!arr.isArray()) return;

  for (std::size_t i = 0; i < arr.arrayValue.size(); ++i) {
    JsonValue& e = arr.arrayValue[i];
    if (!e.isObject()) continue;

    const std::string kind = ReadStringMember(e, "kind");
    const std::string path = ReadStringMember(e, "path");
    const std::string layer = ReadStringMember(e, "layer");

    if (kind != a.kind) continue;
    if (path != a.path) continue;

    if (!a.layer.empty()) {
      if (layer != a.layer) continue;
    } else {
      // If the caller did not specify a layer, only match artifacts that also have no layer.
      if (!layer.empty()) continue;
    }

    e = std::move(obj);
    return;
  }

  arr.arrayValue.push_back(std::move(obj));
}

} // namespace


bool FindCliRunManifestsInDir(const std::filesystem::path& dir,
                              std::vector<std::filesystem::path>& outManifestPaths,
                              std::string& outError,
                              bool newestFirst)
{
  outManifestPaths.clear();
  outError.clear();

  if (dir.empty()) {
    outError = "manifest dir is empty";
    return false;
  }

  std::error_code ec;
  if (!std::filesystem::exists(dir, ec) || ec) {
    outError = "manifest dir does not exist: " + dir.string();
    return false;
  }
  if (!std::filesystem::is_directory(dir, ec) || ec) {
    outError = "manifest path is not a directory: " + dir.string();
    return false;
  }

  struct Found {
    std::filesystem::file_time_type t;
    std::filesystem::path p;
  };

  std::vector<Found> found;

  for (const auto& entry : std::filesystem::directory_iterator(dir, ec)) {
    if (ec) break;

    if (!entry.is_regular_file(ec) || ec) {
      ec.clear();
      continue;
    }

    const std::filesystem::path p = entry.path();
    if (p.extension() != ".json") continue;

    CliRunManifest m;
    std::string merr;
    if (!LoadCliRunManifest(p, m, merr)) {
      continue; // not a CLI manifest
    }
    if (!m.tool.empty() && m.tool != "proc_isocity_cli") {
      continue;
    }

    std::error_code ec2;
    const auto t = std::filesystem::last_write_time(p, ec2);
    if (ec2) continue;

    found.push_back(Found{t, p});
  }

  if (ec) {
    outError = "failed to scan manifest dir: " + ec.message();
    return false;
  }

  if (found.empty()) {
    outError = "no proc_isocity_cli manifest JSON found in dir: " + dir.string();
    return false;
  }

  std::sort(found.begin(), found.end(), [newestFirst](const Found& a, const Found& b) {
    if (a.t == b.t) return a.p.string() < b.p.string();
    return newestFirst ? (a.t > b.t) : (a.t < b.t);
  });

  outManifestPaths.reserve(found.size());
  for (const auto& f : found) {
    outManifestPaths.push_back(f.p);
  }

  return true;
}

bool FindLatestCliRunManifestInDir(const std::filesystem::path& dir,
                                  std::filesystem::path& outManifestPath,
                                  std::string& outError)
{
  outManifestPath.clear();

  std::vector<std::filesystem::path> manifests;
  if (!FindCliRunManifestsInDir(dir, manifests, outError, true)) {
    return false;
  }

  outManifestPath = manifests.front();
  outError.clear();
  return true;
}


bool UpsertCliRunManifestArtifact(const std::filesystem::path& manifestPath,
                                 const CliManifestArtifact& artifact,
                                 const std::filesystem::path& artifactDiskPath,
                                 std::string& outError,
                                 bool atomicWrite)
{
  outError.clear();

  if (manifestPath.empty()) {
    outError = "manifest path is empty";
    return false;
  }
  if (artifact.kind.empty()) {
    outError = "artifact.kind is empty";
    return false;
  }
  if (artifact.path.empty()) {
    outError = "artifact.path is empty";
    return false;
  }

  std::string text;
  if (!ReadFileToString(manifestPath, text, outError)) {
    outError = "manifest: " + outError;
    return false;
  }

  JsonValue root;
  if (!ParseJson(text, root, outError)) {
    outError = "manifest JSON parse error: " + outError;
    return false;
  }
  if (!root.isObject()) {
    outError = "manifest root must be an object";
    return false;
  }

  JsonValue* artifactsArr = nullptr;
  if (!EnsureArtifactsArray(root, artifactsArr) || !artifactsArr) {
    outError = "manifest: failed to access/create artifacts array";
    return false;
  }

  JsonValue obj = BuildArtifactObject(artifact, artifactDiskPath);
  UpsertArtifactInArray(*artifactsArr, std::move(obj), artifact);

  const isocity::JsonWriteOptions wopt{.pretty = true, .indent = 2, .sortKeys = false};

  if (!atomicWrite) {
    std::string err;
    if (!isocity::WriteJsonFile(manifestPath.string(), root, err, wopt)) {
      outError = err;
      return false;
    }
    return true;
  }

  const std::string outJson = isocity::JsonStringify(root, wopt);
  const std::filesystem::path tmpPath = std::filesystem::path(manifestPath.string() + ".tmp");

  // Clean any stale temp file.
  {
    std::error_code ec;
    std::filesystem::remove(tmpPath, ec);
  }

  std::string werr;
  if (!WriteTextFile(tmpPath, outJson, werr)) {
    outError = "manifest: " + werr;
    return false;
  }

  // Best-effort durability.
  BestEffortSyncFile(tmpPath);

  std::string rerr;
  const bool moved = RenameOrCopy(tmpPath, manifestPath, rerr);
  if (!moved) {
    outError = "manifest: " + rerr;
    return false;
  }
  // Any rerr here is a non-fatal warning (e.g., temp file cleanup).
  BestEffortSyncDirectory(manifestPath.parent_path());

  return true;
}


} // namespace isocity
