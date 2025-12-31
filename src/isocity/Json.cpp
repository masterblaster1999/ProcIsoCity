#include "isocity/Json.hpp"

#include <cctype>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <sstream>

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
    case '\\': oss << "\\\\"; break;
    case '"': oss << "\\\""; break;
    case '\b': oss << "\\b"; break;
    case '\f': oss << "\\f"; break;
    case '\n': oss << "\\n"; break;
    case '\r': oss << "\\r"; break;
    case '\t': oss << "\\t"; break;
    default:
      if (ch < 0x20) {
        oss << "\\u";
        static const char* hex = "0123456789ABCDEF";
        oss << hex[(ch >> 4) & 0xF] << hex[ch & 0xF];
        oss << "00";
      } else {
        oss << static_cast<char>(ch);
      }
      break;
    }
  }
  return oss.str();
}

namespace {

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
        case '"': result.push_back('"'); break;
        case '\\': result.push_back('\\'); break;
        case '/': result.push_back('/'); break;
        case 'b': result.push_back('\b'); break;
        case 'f': result.push_back('\f'); break;
        case 'n': result.push_back('\n'); break;
        case 'r': result.push_back('\r'); break;
        case 't': result.push_back('\t'); break;
        case 'u': {
          // Minimal \uXXXX support. We decode only basic ASCII for now.
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
          if (code <= 0x7F) {
            result.push_back(static_cast<char>(code));
          } else {
            // Fallback: preserve something readable without pulling in a full UTF-16 decoder.
            result.push_back('?');
          }
          break;
        }
        default: return fail("unknown escape sequence");
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

} // namespace isocity
