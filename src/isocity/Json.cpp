#include "isocity/Json.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

namespace isocity {

JsonValue JsonValue::MakeNull()
{
  JsonValue v;
  v.type = Type::Null;
  return v;
}

JsonValue JsonValue::MakeBool(bool b)
{
  JsonValue v;
  v.type = Type::Bool;
  v.boolValue = b;
  return v;
}

JsonValue JsonValue::MakeNumber(double n)
{
  JsonValue v;
  v.type = Type::Number;
  v.numberValue = n;
  return v;
}

JsonValue JsonValue::MakeString(std::string s)
{
  JsonValue v;
  v.type = Type::String;
  v.stringValue = std::move(s);
  return v;
}

JsonValue JsonValue::MakeArray()
{
  JsonValue v;
  v.type = Type::Array;
  v.arrayValue.clear();
  return v;
}

JsonValue JsonValue::MakeObject()
{
  JsonValue v;
  v.type = Type::Object;
  v.objectValue.clear();
  return v;
}

const JsonValue* FindJsonMember(const JsonValue& obj, const std::string& key)
{
  if (!obj.isObject()) return nullptr;
  for (const auto& kv : obj.objectValue) {
    if (kv.first == key) return &kv.second;
  }
  return nullptr;
}

JsonValue* FindJsonMember(JsonValue& obj, const std::string& key)
{
  if (!obj.isObject()) return nullptr;
  for (auto& kv : obj.objectValue) {
    if (kv.first == key) return &kv.second;
  }
  return nullptr;
}

std::string JsonEscape(const std::string& s)
{
  std::ostringstream oss;
  for (unsigned char ch : s) {
    switch (ch) {
      case '\\':
        oss << "\\\\";
        break;
      case '"':
        oss << "\\\"";
        break;
      case '\b':
        oss << "\\b";
        break;
      case '\f':
        oss << "\\f";
        break;
      case '\n':
        oss << "\\n";
        break;
      case '\r':
        oss << "\\r";
        break;
      case '\t':
        oss << "\\t";
        break;
      default:
        if (ch < 0x20) {
          static const char* hex = "0123456789ABCDEF";
          oss << "\\u00" << hex[(ch >> 4) & 0xF] << hex[ch & 0xF];
        } else {
          // Emit UTF-8 bytes as-is.
          oss << static_cast<char>(ch);
        }
        break;
    }
  }
  return oss.str();
}

namespace {

static void AppendUtf8(std::string& out, std::uint32_t cp)
{
  // Unicode scalar range is U+0000..U+10FFFF (excluding surrogate code points).
  if (cp <= 0x7F) {
    out.push_back(static_cast<char>(cp));
  } else if (cp <= 0x7FF) {
    out.push_back(static_cast<char>(0xC0 | ((cp >> 6) & 0x1F)));
    out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
  } else if (cp <= 0xFFFF) {
    out.push_back(static_cast<char>(0xE0 | ((cp >> 12) & 0x0F)));
    out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
    out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
  } else {
    out.push_back(static_cast<char>(0xF0 | ((cp >> 18) & 0x07)));
    out.push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
    out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
    out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
  }
}

static bool IsHighSurrogate(std::uint16_t u) { return u >= 0xD800u && u <= 0xDBFFu; }
static bool IsLowSurrogate(std::uint16_t u) { return u >= 0xDC00u && u <= 0xDFFFu; }

static std::string FormatNumber(double v)
{
  if (std::isfinite(v) == 0) return std::string();

  // Prefer integer rendering when the number is exactly representable as a 64-bit integer.
  const double rounded = std::round(v);
  if (std::fabs(v - rounded) <= 0.0 &&
      rounded >= static_cast<double>(std::numeric_limits<std::int64_t>::min()) &&
      rounded <= static_cast<double>(std::numeric_limits<std::int64_t>::max())) {
    return std::to_string(static_cast<std::int64_t>(rounded));
  }

  // Use a moderately high precision for readability (avoids the classic 0.1 -> 0.10000000000000001 issue).
  std::ostringstream oss;
  oss.setf(std::ios::fmtflags(0), std::ios::floatfield);
  oss << std::setprecision(15) << v;
  std::string s = oss.str();

  // Trim trailing zeros for non-scientific floats.
  const std::size_t ePos = s.find_first_of("eE");
  const std::size_t dotPos = s.find('.');
  if (ePos == std::string::npos && dotPos != std::string::npos) {
    while (!s.empty() && s.back() == '0') s.pop_back();
    if (!s.empty() && s.back() == '.') s.pop_back();
  }

  if (s.empty()) s = "0";
  return s;
}

static void Indent(std::ostream& os, const JsonWriteOptions& opt, int depth)
{
  if (!opt.pretty) return;
  const int n = std::max(0, opt.indent) * std::max(0, depth);
  for (int i = 0; i < n; ++i) os.put(' ');
}

static bool WriteJsonValue(std::ostream& os, const JsonValue& v, const JsonWriteOptions& opt, int depth,
                           std::string& err)
{
  switch (v.type) {
    case JsonValue::Type::Null:
      os << "null";
      return true;
    case JsonValue::Type::Bool:
      os << (v.boolValue ? "true" : "false");
      return true;
    case JsonValue::Type::Number: {
      if (std::isfinite(v.numberValue) == 0) {
        err = "cannot serialize non-finite number to JSON";
        return false;
      }
      const std::string n = FormatNumber(v.numberValue);
      if (n.empty()) {
        err = "failed to format number";
        return false;
      }
      os << n;
      return true;
    }
    case JsonValue::Type::String:
      os << '"' << JsonEscape(v.stringValue) << '"';
      return true;
    case JsonValue::Type::Array: {
      os << '[';
      if (v.arrayValue.empty()) {
        os << ']';
        return true;
      }
      if (opt.pretty) os << '\n';
      for (std::size_t i = 0; i < v.arrayValue.size(); ++i) {
        if (opt.pretty) Indent(os, opt, depth + 1);
        if (!WriteJsonValue(os, v.arrayValue[i], opt, depth + 1, err)) return false;
        if (i + 1 < v.arrayValue.size()) os << ',';
        if (opt.pretty) os << '\n';
      }
      if (opt.pretty) Indent(os, opt, depth);
      os << ']';
      return true;
    }
    case JsonValue::Type::Object: {
      os << '{';
      if (v.objectValue.empty()) {
        os << '}';
        return true;
      }

      // Determine iteration order.
      std::vector<std::size_t> order;
      order.reserve(v.objectValue.size());
      for (std::size_t i = 0; i < v.objectValue.size(); ++i) order.push_back(i);
      if (opt.sortKeys) {
        std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
          return v.objectValue[a].first < v.objectValue[b].first;
        });
      }

      if (opt.pretty) os << '\n';
      for (std::size_t oi = 0; oi < order.size(); ++oi) {
        const std::size_t i = order[oi];
        const auto& kv = v.objectValue[i];
        if (opt.pretty) Indent(os, opt, depth + 1);
        os << '"' << JsonEscape(kv.first) << '"' << ':';
        if (opt.pretty) os << ' ';
        if (!WriteJsonValue(os, kv.second, opt, depth + 1, err)) return false;
        if (oi + 1 < order.size()) os << ',';
        if (opt.pretty) os << '\n';
      }
      if (opt.pretty) Indent(os, opt, depth);
      os << '}';
      return true;
    }
    default:
      err = "unknown JsonValue type";
      return false;
  }
}

struct Parser {
  const std::string& s;
  std::size_t i = 0;
  std::string err;

  explicit Parser(const std::string& str) : s(str) {}

  void skipWs()
  {
    while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i])) != 0) ++i;
  }

  char peek() const { return i < s.size() ? s[i] : '\0'; }

  bool consume(char c)
  {
    if (peek() != c) return false;
    ++i;
    return true;
  }

  bool fail(const std::string& msg)
  {
    std::ostringstream oss;
    oss << "JSON parse error @" << i << ": " << msg;
    err = oss.str();
    return false;
  }

  bool parseValue(JsonValue& out)
  {
    skipWs();
    const char c = peek();
    if (c == '\0') return fail("unexpected end of input");

    if (c == 'n') return parseNull(out);
    if (c == 't' || c == 'f') return parseBool(out);
    if (c == '"') {
      std::string tmp;
      if (!parseString(tmp)) return false;
      out = JsonValue::MakeString(std::move(tmp));
      return true;
    }
    if (c == '[') return parseArray(out);
    if (c == '{') return parseObject(out);
    if (c == '-' || std::isdigit(static_cast<unsigned char>(c)) != 0) return parseNumber(out);

    return fail(std::string("unexpected character '") + c + "'");
  }

  bool parseNull(JsonValue& out)
  {
    if (s.compare(i, 4, "null") != 0) return fail("expected 'null'");
    i += 4;
    out = JsonValue::MakeNull();
    return true;
  }

  bool parseBool(JsonValue& out)
  {
    if (s.compare(i, 4, "true") == 0) {
      i += 4;
      out = JsonValue::MakeBool(true);
      return true;
    }
    if (s.compare(i, 5, "false") == 0) {
      i += 5;
      out = JsonValue::MakeBool(false);
      return true;
    }
    return fail("expected boolean");
  }

  bool parseNumber(JsonValue& out)
  {
    skipWs();
    const std::size_t start = i;

    if (peek() == '-') ++i;

    if (peek() == '0') {
      ++i;
    } else {
      if (std::isdigit(static_cast<unsigned char>(peek())) == 0) return fail("expected digit");
      while (std::isdigit(static_cast<unsigned char>(peek())) != 0) ++i;
    }

    if (peek() == '.') {
      ++i;
      if (std::isdigit(static_cast<unsigned char>(peek())) == 0) return fail("expected digit after '.'");
      while (std::isdigit(static_cast<unsigned char>(peek())) != 0) ++i;
    }

    if (peek() == 'e' || peek() == 'E') {
      ++i;
      if (peek() == '+' || peek() == '-') ++i;
      if (std::isdigit(static_cast<unsigned char>(peek())) == 0) return fail("expected exponent digits");
      while (std::isdigit(static_cast<unsigned char>(peek())) != 0) ++i;
    }

    const std::string numStr = s.substr(start, i - start);
    errno = 0;
    char* end = nullptr;
    const double v = std::strtod(numStr.c_str(), &end);
    if (errno != 0 || end == numStr.c_str() || (end && *end != '\0')) return fail("invalid number");

    out = JsonValue::MakeNumber(v);
    return true;
  }

  bool readHex4(std::uint16_t& out)
  {
    if (i + 4 > s.size()) return fail("invalid \\u escape");
    unsigned int code = 0;
    for (int k = 0; k < 4; ++k) {
      const char h = s[i++];
      code <<= 4;
      if (h >= '0' && h <= '9') code |= static_cast<unsigned int>(h - '0');
      else if (h >= 'a' && h <= 'f') code |= static_cast<unsigned int>(h - 'a' + 10);
      else if (h >= 'A' && h <= 'F') code |= static_cast<unsigned int>(h - 'A' + 10);
      else return fail("invalid hex digit in \\u escape");
    }
    out = static_cast<std::uint16_t>(code & 0xFFFFu);
    return true;
  }

  bool parseString(std::string& out)
  {
    skipWs();
    if (!consume('"')) return fail("expected string\"");

    std::string result;
    while (i < s.size()) {
      const char c = s[i++];
      if (c == '"') {
        out = std::move(result);
        return true;
      }
      if (c == '\\') {
        if (i >= s.size()) return fail("unterminated escape sequence");
        const char e = s[i++];
        switch (e) {
          case '"':
            result.push_back('"');
            break;
          case '\\':
            result.push_back('\\');
            break;
          case '/':
            result.push_back('/');
            break;
          case 'b':
            result.push_back('\b');
            break;
          case 'f':
            result.push_back('\f');
            break;
          case 'n':
            result.push_back('\n');
            break;
          case 'r':
            result.push_back('\r');
            break;
          case 't':
            result.push_back('\t');
            break;
          case 'u': {
            // Full UTF-16 \uXXXX decoding with surrogate pair support.
            std::uint16_t u1 = 0;
            if (!readHex4(u1)) return false;

            if (IsHighSurrogate(u1)) {
              // Must be followed by another \uXXXX for the low surrogate.
              if (i + 2 > s.size() || s[i] != '\\' || s[i + 1] != 'u') {
                return fail("high surrogate not followed by \\u escape");
              }
              i += 2;
              std::uint16_t u2 = 0;
              if (!readHex4(u2)) return false;
              if (!IsLowSurrogate(u2)) return fail("invalid low surrogate in \\u escape");

              const std::uint32_t hi = static_cast<std::uint32_t>(u1 - 0xD800u);
              const std::uint32_t lo = static_cast<std::uint32_t>(u2 - 0xDC00u);
              const std::uint32_t cp = 0x10000u + ((hi << 10) | lo);
              AppendUtf8(result, cp);
            } else if (IsLowSurrogate(u1)) {
              return fail("unexpected low surrogate in \\u escape");
            } else {
              AppendUtf8(result, static_cast<std::uint32_t>(u1));
            }
            break;
          }
          default:
            return fail("unknown escape sequence");
        }
      } else {
        result.push_back(c);
      }
    }

    return fail("unterminated string");
  }

  bool parseArray(JsonValue& out)
  {
    skipWs();
    if (!consume('[')) return fail("expected '['");

    JsonValue arr = JsonValue::MakeArray();
    skipWs();
    if (consume(']')) {
      out = std::move(arr);
      return true;
    }

    while (true) {
      JsonValue v;
      if (!parseValue(v)) return false;
      arr.arrayValue.push_back(std::move(v));

      skipWs();
      if (consume(']')) break;
      if (!consume(',')) return fail("expected ',' or ']'");
    }

    out = std::move(arr);
    return true;
  }

  bool parseObject(JsonValue& out)
  {
    skipWs();
    if (!consume('{')) return fail("expected '{'");

    JsonValue obj = JsonValue::MakeObject();
    skipWs();
    if (consume('}')) {
      out = std::move(obj);
      return true;
    }

    while (true) {
      std::string key;
      if (!parseString(key)) return false;

      skipWs();
      if (!consume(':')) return fail("expected ':'");

      JsonValue val;
      if (!parseValue(val)) return false;

      obj.objectValue.emplace_back(std::move(key), std::move(val));

      skipWs();
      if (consume('}')) break;
      if (!consume(',')) return fail("expected ',' or '}'");
    }

    out = std::move(obj);
    return true;
  }
};

} // namespace

bool ParseJson(const std::string& text, JsonValue& outValue, std::string& outError)
{
  Parser p(text);
  JsonValue v;
  if (!p.parseValue(v)) {
    outError = p.err;
    return false;
  }
  p.skipWs();
  if (p.i != text.size()) {
    outError = p.err.empty() ? (std::string("JSON parse error @") + std::to_string(p.i) + ": trailing characters")
                             : p.err;
    return false;
  }

  outValue = std::move(v);
  outError.clear();
  return true;
}

bool WriteJson(std::ostream& os, const JsonValue& value, std::string& outError, const JsonWriteOptions& opt)
{
  outError.clear();
  if (!WriteJsonValue(os, value, opt, 0, outError)) return false;
  if (opt.pretty) os << '\n';
  if (!os) {
    outError = "JSON write failed";
    return false;
  }
  return true;
}

std::string JsonStringify(const JsonValue& value, const JsonWriteOptions& opt)
{
  std::ostringstream oss;
  std::string err;
  // Ignore write failure in stringify; return partial output if any.
  (void)WriteJsonValue(oss, value, opt, 0, err);
  if (opt.pretty) oss << '\n';
  return oss.str();
}

bool WriteJsonFile(const std::string& path, const JsonValue& value, std::string& outError, const JsonWriteOptions& opt)
{
  std::ofstream f(path, std::ios::binary);
  if (!f) {
    outError = "failed to open for write: " + path;
    return false;
  }
  if (!WriteJson(f, value, outError, opt)) return false;
  return true;
}



// -------------------------------------------------------------------------------------------------
// JsonWriter (streaming)
// -------------------------------------------------------------------------------------------------

JsonWriter::JsonWriter(std::ostream& os, JsonWriteOptions opt)
  : m_os(&os)
  , m_opt(opt)
{
}

void JsonWriter::reset()
{
  m_stack.clear();
  m_finished = false;
  m_error.clear();
}

bool JsonWriter::setError(std::string msg)
{
  if (m_error.empty()) m_error = std::move(msg);
  return false;
}

bool JsonWriter::writeChar(char c)
{
  if (!m_os) return setError("JsonWriter: null stream");
  m_os->put(c);
  if (!m_os->good()) return setError("JsonWriter: stream write failed");
  return true;
}

bool JsonWriter::writeRaw(const char* s)
{
  if (!s) return true;
  if (!m_os) return setError("JsonWriter: null stream");
  (*m_os) << s;
  if (!m_os->good()) return setError("JsonWriter: stream write failed");
  return true;
}

bool JsonWriter::writeRaw(const std::string& s)
{
  if (!m_os) return setError("JsonWriter: null stream");
  (*m_os) << s;
  if (!m_os->good()) return setError("JsonWriter: stream write failed");
  return true;
}

void JsonWriter::indent(int depth)
{
  if (!m_os) return;
  Indent(*m_os, m_opt, depth);
}

bool JsonWriter::prepareValue()
{
  if (!m_error.empty()) return false;
  if (!m_os) return setError("JsonWriter: null stream");

  if (m_stack.empty()) {
    if (m_finished) return setError("JsonWriter: root value already finished");
    return true;
  }

  Frame& f = m_stack.back();
  if (f.kind == Frame::Kind::Array) {
    if (!f.first) {
      if (!writeChar(',')) return false;
    }
    if (m_opt.pretty) {
      if (!writeChar('\n')) return false;
      indent(static_cast<int>(m_stack.size()));
    }
    f.first = false;
    return true;
  }

  // Object.
  if (f.expectingKey) return setError("JsonWriter: expected key() in object");
  // key() already wrote the `"k": ` portion.
  return true;
}

bool JsonWriter::finishValue()
{
  if (!m_error.empty()) return false;

  if (m_stack.empty()) {
    m_finished = true;
    return true;
  }

  Frame& f = m_stack.back();
  if (f.kind == Frame::Kind::Object) {
    f.expectingKey = true;
  }
  return true;
}

bool JsonWriter::beginContainer(Frame::Kind kind, char openChar)
{
  if (!prepareValue()) return false;
  if (!writeChar(openChar)) return false;

  Frame f;
  f.kind = kind;
  f.first = true;
  f.expectingKey = (kind == Frame::Kind::Object);
  m_stack.push_back(f);
  return true;
}

bool JsonWriter::endContainer(Frame::Kind kind, char closeChar)
{
  if (!m_error.empty()) return false;
  if (!m_os) return setError("JsonWriter: null stream");
  if (m_stack.empty()) return setError("JsonWriter: endContainer with empty stack");

  Frame f = m_stack.back();
  if (f.kind != kind) return setError("JsonWriter: mismatched container end");

  if (kind == Frame::Kind::Object && !f.expectingKey) {
    return setError("JsonWriter: endObject while expecting a value");
  }

  // If non-empty, close on its own line for pretty mode.
  if (m_opt.pretty && !f.first) {
    if (!writeChar('\n')) return false;
    indent(static_cast<int>(m_stack.size()) - 1);
  }

  if (!writeChar(closeChar)) return false;

  m_stack.pop_back();
  return finishValue();
}

bool JsonWriter::beginObject()
{
  return beginContainer(Frame::Kind::Object, '{');
}

bool JsonWriter::endObject()
{
  return endContainer(Frame::Kind::Object, '}');
}

bool JsonWriter::beginArray()
{
  return beginContainer(Frame::Kind::Array, '[');
}

bool JsonWriter::endArray()
{
  return endContainer(Frame::Kind::Array, ']');
}

bool JsonWriter::key(const std::string& k)
{
  if (!m_error.empty()) return false;
  if (!m_os) return setError("JsonWriter: null stream");
  if (m_stack.empty()) return setError("JsonWriter: key() outside any object");

  Frame& f = m_stack.back();
  if (f.kind != Frame::Kind::Object) return setError("JsonWriter: key() inside non-object container");
  if (!f.expectingKey) return setError("JsonWriter: key() called but a value is expected");

  if (!f.first) {
    if (!writeChar(',')) return false;
  }
  if (m_opt.pretty) {
    if (!writeChar('\n')) return false;
    indent(static_cast<int>(m_stack.size()));
  }

  if (!writeChar('"')) return false;
  if (!writeRaw(JsonEscape(k))) return false;
  if (!writeRaw("\":")) return false;
  if (m_opt.pretty) {
    if (!writeChar(' ')) return false;
  }

  f.first = false;
  f.expectingKey = false;
  return true;
}

bool JsonWriter::nullValue()
{
  if (!prepareValue()) return false;
  if (!writeRaw("null")) return false;
  return finishValue();
}

bool JsonWriter::boolValue(bool b)
{
  if (!prepareValue()) return false;
  if (!writeRaw(b ? "true" : "false")) return false;
  return finishValue();
}

bool JsonWriter::numberValue(double n)
{
  if (!prepareValue()) return false;
  if (std::isfinite(n) == 0) return setError("JsonWriter: cannot write non-finite number");

  const std::string s = FormatNumber(n);
  if (s.empty()) return setError("JsonWriter: failed to format number");
  if (!writeRaw(s)) return false;
  return finishValue();
}

bool JsonWriter::intValue(std::int64_t n)
{
  if (!prepareValue()) return false;
  if (!writeRaw(std::to_string(n))) return false;
  return finishValue();
}

bool JsonWriter::uintValue(std::uint64_t n)
{
  if (!prepareValue()) return false;
  if (!writeRaw(std::to_string(n))) return false;
  return finishValue();
}

bool JsonWriter::stringValue(const std::string& s)
{
  if (!prepareValue()) return false;
  if (!writeChar('"')) return false;
  if (!writeRaw(JsonEscape(s))) return false;
  if (!writeChar('"')) return false;
  return finishValue();
}

bool JsonWriter::value(const JsonValue& v)
{
  if (!prepareValue()) return false;

  JsonWriteOptions subOpt = m_opt;
  subOpt.pretty = false; // embed as compact subtree
  const std::string s = JsonStringify(v, subOpt);

  if (!writeRaw(s)) return false;
  return finishValue();
}


} // namespace isocity
