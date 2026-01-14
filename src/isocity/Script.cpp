#include "isocity/Script.hpp"

#include "isocity/Brush.hpp"
#include "isocity/DistrictStats.hpp"
#include "isocity/Districting.hpp"
#include "isocity/Export.hpp"
#include "isocity/FloodFill.hpp"
#include "isocity/Goods.hpp"
#include "isocity/Hash.hpp"
#include "isocity/LandValue.hpp"
#include "isocity/Pathfinding.hpp"
#include "isocity/ProcGen.hpp"
#include "isocity/SaveLoad.hpp"
#include "isocity/Traffic.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace isocity {

namespace {

static std::string Trim(std::string s)
{
  auto isWs = [](unsigned char c) { return std::isspace(c) != 0; };
  while (!s.empty() && isWs(static_cast<unsigned char>(s.front()))) s.erase(s.begin());
  while (!s.empty() && isWs(static_cast<unsigned char>(s.back()))) s.pop_back();
  return s;
}

static std::vector<std::string> SplitWS(const std::string& s)
{
  std::vector<std::string> out;
  std::string cur;
  for (unsigned char uc : s) {
    const char c = static_cast<char>(uc);
    if (std::isspace(uc)) {
      if (!cur.empty()) {
        out.push_back(cur);
        cur.clear();
      }
      continue;
    }
    cur.push_back(c);
  }
  if (!cur.empty()) out.push_back(cur);
  return out;
}

static std::string ToLower(std::string s)
{
  for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

static const char* ToolApplyResultName(ToolApplyResult r)
{
  switch (r) {
  case ToolApplyResult::Applied: return "Applied";
  case ToolApplyResult::Noop: return "Noop";
  case ToolApplyResult::OutOfBounds: return "OutOfBounds";
  case ToolApplyResult::BlockedWater: return "BlockedWater";
  case ToolApplyResult::BlockedNoRoad: return "BlockedNoRoad";
  case ToolApplyResult::BlockedOccupied: return "BlockedOccupied";
  case ToolApplyResult::InsufficientFunds: return "InsufficientFunds";
  default: return "Unknown";
  }
}

static bool CheckedAddI64(std::int64_t a, std::int64_t b, std::int64_t* out)
{
  if (!out) return false;
  const auto max = std::numeric_limits<std::int64_t>::max();
  const auto min = std::numeric_limits<std::int64_t>::min();
  if ((b > 0 && a > max - b) || (b < 0 && a < min - b)) return false;
  *out = a + b;
  return true;
}

static bool CheckedSubI64(std::int64_t a, std::int64_t b, std::int64_t* out)
{
  if (!out) return false;
  const auto max = std::numeric_limits<std::int64_t>::max();
  const auto min = std::numeric_limits<std::int64_t>::min();
  if ((b > 0 && a < min + b) || (b < 0 && a > max + b)) return false;
  *out = a - b;
  return true;
}

static bool CheckedMulI64(std::int64_t a, std::int64_t b, std::int64_t* out)
{
  if (!out) return false;
  const auto max = std::numeric_limits<std::int64_t>::max();
  const auto min = std::numeric_limits<std::int64_t>::min();

  if (a == 0 || b == 0) {
    *out = 0;
    return true;
  }

  if (a == -1) {
    if (b == min) return false;
    *out = -b;
    return true;
  }

  if (b == -1) {
    if (a == min) return false;
    *out = -a;
    return true;
  }

  if (a > 0) {
    if (b > 0) {
      if (a > max / b) return false;
    } else {  // b < 0
      if (b < min / a) return false;
    }
  } else {  // a < 0
    if (b > 0) {
      if (a < min / b) return false;
    } else {  // b < 0
      if (a < max / b) return false;
    }
  }

  *out = a * b;
  return true;
}

static bool CheckedDivI64(std::int64_t a, std::int64_t b, std::int64_t* out)
{
  if (!out) return false;
  if (b == 0) return false;

  const auto min = std::numeric_limits<std::int64_t>::min();
  if (a == min && b == -1) return false;

  *out = a / b;
  return true;
}

static bool CheckedModI64(std::int64_t a, std::int64_t b, std::int64_t* out)
{
  if (!out) return false;
  if (b == 0) return false;

  const auto min = std::numeric_limits<std::int64_t>::min();
  if (a == min && b == -1) {
    // C++ defines this, but keep behavior consistent with CheckedDivI64.
    return false;
  }

  *out = a % b;
  return true;
}

static bool CheckedAddU64(std::uint64_t a, std::uint64_t b, std::uint64_t* out)
{
  if (!out) return false;
  const auto max = std::numeric_limits<std::uint64_t>::max();
  if (a > max - b) return false;
  *out = a + b;
  return true;
}

static bool CheckedSubU64(std::uint64_t a, std::uint64_t b, std::uint64_t* out)
{
  if (!out) return false;
  if (a < b) return false;
  *out = a - b;
  return true;
}

static bool CheckedMulU64(std::uint64_t a, std::uint64_t b, std::uint64_t* out)
{
  if (!out) return false;
  if (a == 0 || b == 0) {
    *out = 0;
    return true;
  }
  const auto max = std::numeric_limits<std::uint64_t>::max();
  if (a > max / b) return false;
  *out = a * b;
  return true;
}

static bool CheckedDivU64(std::uint64_t a, std::uint64_t b, std::uint64_t* out)
{
  if (!out) return false;
  if (b == 0) return false;
  *out = a / b;
  return true;
}

static bool CheckedModU64(std::uint64_t a, std::uint64_t b, std::uint64_t* out)
{
  if (!out) return false;
  if (b == 0) return false;
  *out = a % b;
  return true;
}

static int hexDigit(char c)
{
  if (c >= '0' && c <= '9') return (c - '0');
  if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
  if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
  return -1;
}

static bool ParseU64Literal(const std::string& s, std::size_t* posInOut, std::uint64_t* out)
{
  if (!posInOut) return false;
  if (!out) return false;

  std::size_t pos = *posInOut;
  if (pos >= s.size()) return false;

  int base = 10;
  if (pos + 2 <= s.size() && s[pos] == '0' && (s[pos + 1] == 'x' || s[pos + 1] == 'X')) {
    base = 16;
    pos += 2;
  }

  bool any = false;
  std::uint64_t v = 0;

  const auto max = std::numeric_limits<std::uint64_t>::max();

  while (pos < s.size()) {
    const char c = s[pos];
    int d = -1;

    if (base == 10) {
      if (c >= '0' && c <= '9') d = (c - '0');
      else break;
    } else {
      d = hexDigit(c);
      if (d < 0) break;
    }

    any = true;

    // v = v * base + d (with overflow checking)
    if (v > max / static_cast<std::uint64_t>(base)) return false;
    v *= static_cast<std::uint64_t>(base);

    if (v > max - static_cast<std::uint64_t>(d)) return false;
    v += static_cast<std::uint64_t>(d);

    ++pos;
  }

  if (!any) return false;
  *posInOut = pos;
  *out = v;
  return true;
}

static bool EvalI64Expr(const std::string& s, std::int64_t* out)
{
  if (!out) return false;
  if (s.empty()) return false;

  struct Parser {
    const std::string& s;
    std::size_t pos = 0;

    void skipWs()
    {
      while (pos < s.size() && std::isspace(static_cast<unsigned char>(s[pos]))) {
        ++pos;
      }
    }

    bool match(char c)
    {
      if (pos < s.size() && s[pos] == c) {
        ++pos;
        return true;
      }
      return false;
    }

    bool parseExpr(std::int64_t* out)
    {
      std::int64_t v = 0;
      if (!parseTerm(&v)) return false;

      while (true) {
        skipWs();
        if (match('+')) {
          std::int64_t rhs = 0;
          if (!parseTerm(&rhs)) return false;
          if (!CheckedAddI64(v, rhs, &v)) return false;
          continue;
        }
        if (match('-')) {
          std::int64_t rhs = 0;
          if (!parseTerm(&rhs)) return false;
          if (!CheckedSubI64(v, rhs, &v)) return false;
          continue;
        }
        break;
      }

      *out = v;
      return true;
    }

    bool parseTerm(std::int64_t* out)
    {
      std::int64_t v = 0;
      if (!parseFactor(&v)) return false;

      while (true) {
        skipWs();
        if (match('*')) {
          std::int64_t rhs = 0;
          if (!parseFactor(&rhs)) return false;
          if (!CheckedMulI64(v, rhs, &v)) return false;
          continue;
        }
        if (match('/')) {
          std::int64_t rhs = 0;
          if (!parseFactor(&rhs)) return false;
          if (!CheckedDivI64(v, rhs, &v)) return false;
          continue;
        }
        if (match('%')) {
          std::int64_t rhs = 0;
          if (!parseFactor(&rhs)) return false;
          if (!CheckedModI64(v, rhs, &v)) return false;
          continue;
        }
        break;
      }

      *out = v;
      return true;
    }

    bool parseFactor(std::int64_t* out)
    {
      skipWs();

      if (match('+')) {
        return parseFactor(out);
      }

      if (match('-')) {
        std::int64_t v = 0;
        if (!parseFactor(&v)) return false;

        const auto min = std::numeric_limits<std::int64_t>::min();
        if (v == min) return false;
        *out = -v;
        return true;
      }

      if (match('(')) {
        std::int64_t v = 0;
        if (!parseExpr(&v)) return false;
        skipWs();
        if (!match(')')) return false;
        *out = v;
        return true;
      }

      return parseNumber(out);
    }

    bool parseNumber(std::int64_t* out)
    {
      skipWs();
      std::uint64_t mag = 0;
      const std::size_t startPos = pos;
      if (!ParseU64Literal(s, &pos, &mag)) {
        pos = startPos;
        return false;
      }

      const auto max = static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max());
      if (mag > max) return false;

      *out = static_cast<std::int64_t>(mag);
      return true;
    }

    bool parse(std::int64_t* out)
    {
      pos = 0;
      skipWs();
      if (!parseExpr(out)) return false;
      skipWs();
      return pos == s.size();
    }
  };

  Parser p{s};
  std::int64_t v = 0;
  if (!p.parse(&v)) return false;
  *out = v;
  return true;
}

static bool EvalU64Expr(const std::string& s, std::uint64_t* out)
{
  if (!out) return false;
  if (s.empty()) return false;

  struct Parser {
    const std::string& s;
    std::size_t pos = 0;

    void skipWs()
    {
      while (pos < s.size() && std::isspace(static_cast<unsigned char>(s[pos]))) {
        ++pos;
      }
    }

    bool match(char c)
    {
      if (pos < s.size() && s[pos] == c) {
        ++pos;
        return true;
      }
      return false;
    }

    bool parseExpr(std::uint64_t* out)
    {
      std::uint64_t v = 0;
      if (!parseTerm(&v)) return false;

      while (true) {
        skipWs();
        if (match('+')) {
          std::uint64_t rhs = 0;
          if (!parseTerm(&rhs)) return false;
          if (!CheckedAddU64(v, rhs, &v)) return false;
          continue;
        }
        if (match('-')) {
          std::uint64_t rhs = 0;
          if (!parseTerm(&rhs)) return false;
          if (!CheckedSubU64(v, rhs, &v)) return false;
          continue;
        }
        break;
      }

      *out = v;
      return true;
    }

    bool parseTerm(std::uint64_t* out)
    {
      std::uint64_t v = 0;
      if (!parseFactor(&v)) return false;

      while (true) {
        skipWs();
        if (match('*')) {
          std::uint64_t rhs = 0;
          if (!parseFactor(&rhs)) return false;
          if (!CheckedMulU64(v, rhs, &v)) return false;
          continue;
        }
        if (match('/')) {
          std::uint64_t rhs = 0;
          if (!parseFactor(&rhs)) return false;
          if (!CheckedDivU64(v, rhs, &v)) return false;
          continue;
        }
        if (match('%')) {
          std::uint64_t rhs = 0;
          if (!parseFactor(&rhs)) return false;
          if (!CheckedModU64(v, rhs, &v)) return false;
          continue;
        }
        break;
      }

      *out = v;
      return true;
    }

    bool parseFactor(std::uint64_t* out)
    {
      skipWs();

      if (match('+')) {
        return parseFactor(out);
      }

      if (match('-')) {
        // Unsigned expressions do not allow unary minus.
        return false;
      }

      if (match('(')) {
        std::uint64_t v = 0;
        if (!parseExpr(&v)) return false;
        skipWs();
        if (!match(')')) return false;
        *out = v;
        return true;
      }

      return parseNumber(out);
    }

    bool parseNumber(std::uint64_t* out)
    {
      skipWs();
      const std::size_t startPos = pos;
      std::uint64_t v = 0;
      if (!ParseU64Literal(s, &pos, &v)) {
        pos = startPos;
        return false;
      }
      *out = v;
      return true;
    }

    bool parse(std::uint64_t* out)
    {
      pos = 0;
      skipWs();
      if (!parseExpr(out)) return false;
      skipWs();
      return pos == s.size();
    }
  };

  Parser p{s};
  std::uint64_t v = 0;
  if (!p.parse(&v)) return false;
  *out = v;
  return true;
}

// A permissive, C-like expression evaluator used for control flow (`if`/`while`/`expect`).
//
// The result is an i64 value, where comparisons/logical operators yield 0 or 1.
// Supported operators (C-like precedence):
//  - logical:     ||  &&  !
//  - comparison:  ==  !=  <  <=  >  >=
//  - arithmetic:  +  -  *  /  %  (unary +/-)
// Parentheses are supported. Integers are decimal or 0x... hex.
static bool EvalI64LogicExpr(const std::string& s, std::int64_t* out)
{
  if (!out) return false;
  if (s.empty()) return false;

  struct Parser {
    const std::string& s;
    std::size_t pos = 0;

    void skipWs()
    {
      while (pos < s.size() && std::isspace(static_cast<unsigned char>(s[pos]))) {
        ++pos;
      }
    }

    bool matchChar(char c)
    {
      if (pos < s.size() && s[pos] == c) {
        ++pos;
        return true;
      }
      return false;
    }

    bool matchStr(const char* lit)
    {
      const std::size_t n = std::strlen(lit);
      if (pos + n > s.size()) return false;
      for (std::size_t i = 0; i < n; ++i) {
        if (s[pos + i] != lit[i]) return false;
      }
      pos += n;
      return true;
    }

    // Grammar:
    //  expr := or
    //  or := and ( '||' and )*
    //  and := eq ( '&&' eq )*
    //  eq := rel ( ('==' | '!=') rel )*
    //  rel := add ( ('<=' | '>=' | '<' | '>') add )*
    //  add := mul ( ('+' | '-') mul )*
    //  mul := unary ( ('*' | '/' | '%') unary )*
    //  unary := ('+' | '-' | '!') unary | primary
    //  primary := number | '(' expr ')'

    bool parseExpr(std::int64_t* out)
    {
      return parseOr(out);
    }

    bool parseOr(std::int64_t* out)
    {
      std::int64_t v = 0;
      if (!parseAnd(&v)) return false;

      while (true) {
        skipWs();
        if (matchStr("||")) {
          std::int64_t rhs = 0;
          if (!parseAnd(&rhs)) return false;
          v = ((v != 0) || (rhs != 0)) ? 1 : 0;
          continue;
        }
        break;
      }

      *out = v;
      return true;
    }

    bool parseAnd(std::int64_t* out)
    {
      std::int64_t v = 0;
      if (!parseEq(&v)) return false;

      while (true) {
        skipWs();
        if (matchStr("&&")) {
          std::int64_t rhs = 0;
          if (!parseEq(&rhs)) return false;
          v = ((v != 0) && (rhs != 0)) ? 1 : 0;
          continue;
        }
        break;
      }

      *out = v;
      return true;
    }

    bool parseEq(std::int64_t* out)
    {
      std::int64_t v = 0;
      if (!parseRel(&v)) return false;

      while (true) {
        skipWs();
        if (matchStr("==")) {
          std::int64_t rhs = 0;
          if (!parseRel(&rhs)) return false;
          v = (v == rhs) ? 1 : 0;
          continue;
        }
        if (matchStr("!=")) {
          std::int64_t rhs = 0;
          if (!parseRel(&rhs)) return false;
          v = (v != rhs) ? 1 : 0;
          continue;
        }
        break;
      }

      *out = v;
      return true;
    }

    bool parseRel(std::int64_t* out)
    {
      std::int64_t v = 0;
      if (!parseAdd(&v)) return false;

      while (true) {
        skipWs();
        if (matchStr("<=")) {
          std::int64_t rhs = 0;
          if (!parseAdd(&rhs)) return false;
          v = (v <= rhs) ? 1 : 0;
          continue;
        }
        if (matchStr(">=")) {
          std::int64_t rhs = 0;
          if (!parseAdd(&rhs)) return false;
          v = (v >= rhs) ? 1 : 0;
          continue;
        }
        if (matchChar('<')) {
          std::int64_t rhs = 0;
          if (!parseAdd(&rhs)) return false;
          v = (v < rhs) ? 1 : 0;
          continue;
        }
        if (matchChar('>')) {
          std::int64_t rhs = 0;
          if (!parseAdd(&rhs)) return false;
          v = (v > rhs) ? 1 : 0;
          continue;
        }
        break;
      }

      *out = v;
      return true;
    }

    bool parseAdd(std::int64_t* out)
    {
      std::int64_t v = 0;
      if (!parseMul(&v)) return false;

      while (true) {
        skipWs();
        if (matchChar('+')) {
          std::int64_t rhs = 0;
          if (!parseMul(&rhs)) return false;
          if (!CheckedAddI64(v, rhs, &v)) return false;
          continue;
        }
        if (matchChar('-')) {
          std::int64_t rhs = 0;
          if (!parseMul(&rhs)) return false;
          if (!CheckedSubI64(v, rhs, &v)) return false;
          continue;
        }
        break;
      }

      *out = v;
      return true;
    }

    bool parseMul(std::int64_t* out)
    {
      std::int64_t v = 0;
      if (!parseUnary(&v)) return false;

      while (true) {
        skipWs();
        if (matchChar('*')) {
          std::int64_t rhs = 0;
          if (!parseUnary(&rhs)) return false;
          if (!CheckedMulI64(v, rhs, &v)) return false;
          continue;
        }
        if (matchChar('/')) {
          std::int64_t rhs = 0;
          if (!parseUnary(&rhs)) return false;
          if (!CheckedDivI64(v, rhs, &v)) return false;
          continue;
        }
        if (matchChar('%')) {
          std::int64_t rhs = 0;
          if (!parseUnary(&rhs)) return false;
          if (!CheckedModI64(v, rhs, &v)) return false;
          continue;
        }
        break;
      }

      *out = v;
      return true;
    }

    bool parseUnary(std::int64_t* out)
    {
      skipWs();

      if (matchChar('+')) {
        return parseUnary(out);
      }

      if (matchChar('-')) {
        std::int64_t v = 0;
        if (!parseUnary(&v)) return false;
        const auto min = std::numeric_limits<std::int64_t>::min();
        if (v == min) return false;
        *out = -v;
        return true;
      }

      if (matchChar('!')) {
        std::int64_t v = 0;
        if (!parseUnary(&v)) return false;
        *out = (v == 0) ? 1 : 0;
        return true;
      }

      return parsePrimary(out);
    }

    bool parsePrimary(std::int64_t* out)
    {
      skipWs();
      if (matchChar('(')) {
        std::int64_t v = 0;
        if (!parseExpr(&v)) return false;
        skipWs();
        if (!matchChar(')')) return false;
        *out = v;
        return true;
      }
      return parseNumber(out);
    }

    bool parseNumber(std::int64_t* out)
    {
      skipWs();
      std::uint64_t mag = 0;
      const std::size_t startPos = pos;
      if (!ParseU64Literal(s, &pos, &mag)) {
        pos = startPos;
        return false;
      }

      const auto max = static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max());
      if (mag > max) return false;

      *out = static_cast<std::int64_t>(mag);
      return true;
    }

    bool parse(std::int64_t* out)
    {
      pos = 0;
      skipWs();
      if (!parseExpr(out)) return false;
      skipWs();
      return pos == s.size();
    }
  };

  Parser p{s};
  std::int64_t v = 0;
  if (!p.parse(&v)) return false;
  *out = v;
  return true;
}

static bool ParseI32(const std::string& s, int* out)
{
  if (!out) return false;
  if (s.empty()) return false;

  std::int64_t v = 0;
  if (!EvalI64Expr(s, &v)) return false;

  if (v < std::numeric_limits<int>::min() || v > std::numeric_limits<int>::max()) return false;
  *out = static_cast<int>(v);
  return true;
}

static bool ParseF32(const std::string& s, float* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  char* end = nullptr;
  const float v = std::strtof(s.c_str(), &end);
  if (!end || *end != '\0') return false;
  *out = v;
  return true;
}

static bool ParseU64(const std::string& s, std::uint64_t* out)
{
  if (!out) return false;
  if (s.empty()) return false;
  return EvalU64Expr(s, out);
}

static bool WriteStatsCsv(const std::string& path, const std::vector<Stats>& rows, std::string& outError)
{
  outError.clear();
  std::ofstream f(path, std::ios::binary);
  if (!f) {
    outError = "failed to open for writing";
    return false;
  }

  f << "day,population,money,housingCapacity,jobsCapacity,jobsCapacityAccessible,employed,happiness,roads,parks,avgCommuteTime,trafficCongestion,goodsDemand,goodsDelivered,goodsSatisfaction,avgLandValue,demandResidential\n";
  for (const auto& s : rows) {
    f << s.day << ','
      << s.population << ','
      << s.money << ','
      << s.housingCapacity << ','
      << s.jobsCapacity << ','
      << s.jobsCapacityAccessible << ','
      << s.employed << ','
      << s.happiness << ','
      << s.roads << ','
      << s.parks << ','
      << s.avgCommuteTime << ','
      << s.trafficCongestion << ','
      << s.goodsDemand << ','
      << s.goodsDelivered << ','
      << s.goodsSatisfaction << ','
      << s.avgLandValue << ','
      << s.demandResidential << '\n';
  }

  if (!f) {
    outError = "write failed";
    return false;
  }
  return true;
}

static bool ParseBool01(const std::string& s, bool* out)
{
  if (!out) return false;
  const std::string k = ToLower(s);
  if (k == "1" || k == "true" || k == "yes" || k == "on") {
    *out = true;
    return true;
  }
  if (k == "0" || k == "false" || k == "no" || k == "off") {
    *out = false;
    return true;
  }
  return false;
}

static std::vector<std::string> SplitCommaLower(const std::string& s)
{
  std::vector<std::string> out;
  std::string cur;
  cur.reserve(s.size());
  for (char c : s) {
    if (c == ',') {
      if (!cur.empty()) out.push_back(ToLower(cur));
      cur.clear();
      continue;
    }
    cur.push_back(c);
  }
  if (!cur.empty()) out.push_back(ToLower(cur));
  return out;
}

static bool ParseTileFieldMaskList(const std::string& s, std::uint8_t* outMask, std::string* outError)
{
  if (outError) outError->clear();
  if (!outMask) return false;
  if (s.empty()) {
    if (outError) *outError = "empty fields list";
    return false;
  }

  std::uint8_t m = 0;
  for (const auto& t : SplitCommaLower(s)) {
    if (t == "all") {
      m = 0xFFu;
      continue;
    }
    if (t == "none") {
      m = 0;
      continue;
    }

    if (t == "terrain") m |= static_cast<std::uint8_t>(TileFieldMask::Terrain);
    else if (t == "overlay") m |= static_cast<std::uint8_t>(TileFieldMask::Overlay);
    else if (t == "height") m |= static_cast<std::uint8_t>(TileFieldMask::Height);
    else if (t == "variation") m |= static_cast<std::uint8_t>(TileFieldMask::Variation);
    else if (t == "level") m |= static_cast<std::uint8_t>(TileFieldMask::Level);
    else if (t == "occupants") m |= static_cast<std::uint8_t>(TileFieldMask::Occupants);
    else if (t == "district") m |= static_cast<std::uint8_t>(TileFieldMask::District);
    else {
      if (outError) *outError = "unknown field: " + t;
      return false;
    }
  }

  *outMask = m;
  return true;
}

static bool ParseWxH(const std::string& s, int* outW, int* outH)
{
  if (!outW || !outH) return false;
  const std::size_t pos = s.find_first_of("xX");
  if (pos == std::string::npos) return false;
  int w = 0;
  int h = 0;
  if (!ParseI32(s.substr(0, pos), &w)) return false;
  if (!ParseI32(s.substr(pos + 1), &h)) return false;
  if (w <= 0 || h <= 0) return false;
  *outW = w;
  *outH = h;
  return true;
}

static std::string HexU64(std::uint64_t v)
{
  std::ostringstream oss;
  oss << "0x" << std::hex << std::setw(16) << std::setfill('0') << v;
  return oss.str();
}

static bool ApplyZoneTile(World& world, Tool tool, int x, int y, int targetLevel, ToolApplyResult* outFail = nullptr)
{
  targetLevel = std::clamp(targetLevel, 1, 3);

  // Ensure the correct overlay is present (placement or upgrade).
  ToolApplyResult r = world.applyTool(tool, x, y);
  if (r != ToolApplyResult::Applied && r != ToolApplyResult::Noop) {
    if (outFail) *outFail = r;
    return false;
  }

  Tile& t = world.at(x, y);
  if (tool == Tool::Residential && t.overlay != Overlay::Residential) return false;
  if (tool == Tool::Commercial && t.overlay != Overlay::Commercial) return false;
  if (tool == Tool::Industrial && t.overlay != Overlay::Industrial) return false;

  // Upgrade until desired level.
  while (static_cast<int>(t.level) < targetLevel) {
    r = world.applyTool(tool, x, y);
    if (r != ToolApplyResult::Applied && r != ToolApplyResult::Noop) {
      if (outFail) *outFail = r;
      return false;
    }
  }

  return true;
}

static bool ApplyRoadTile(World& world, int x, int y, int level, ToolApplyResult* outFail = nullptr)
{
  const ToolApplyResult r = world.applyRoad(x, y, level);
  if (outFail) *outFail = r;
  return (r == ToolApplyResult::Applied || r == ToolApplyResult::Noop);
}

static bool ApplyFill(World& world, const std::string& what, Point a, Point b, int arg, Point* outFailP = nullptr,
                      ToolApplyResult* outFailR = nullptr)
{
  const std::string k = ToLower(what);
  const int argOr1 = (arg > 0) ? arg : 1;

  const bool recognized = (k == "road" || k == "park" || k == "bulldoze" || k == "district" || k == "res" ||
                           k == "residential" || k == "com" || k == "commercial" || k == "ind" || k == "industrial");
  if (!recognized) return false;

  bool ok = true;
  Point failP{0, 0};
  ToolApplyResult failR = ToolApplyResult::Noop;

  ForEachRectFilled(a, b, [&](Point p) {
    if (!ok) return;
    if (!world.inBounds(p.x, p.y)) {
      ok = false;
      failP = p;
      failR = ToolApplyResult::OutOfBounds;
      return;
    }

    if (k == "road") {
      ToolApplyResult r = ToolApplyResult::Noop;
      if (!ApplyRoadTile(world, p.x, p.y, argOr1, &r)) {
        ok = false;
        failP = p;
        failR = r;
      }
      return;
    }

    if (k == "park") {
      const ToolApplyResult r = world.applyTool(Tool::Park, p.x, p.y);
      if (r != ToolApplyResult::Applied && r != ToolApplyResult::Noop) {
        ok = false;
        failP = p;
        failR = r;
      }
      return;
    }

    if (k == "bulldoze") {
      const ToolApplyResult r = world.applyTool(Tool::Bulldoze, p.x, p.y);
      if (r != ToolApplyResult::Applied && r != ToolApplyResult::Noop) {
        ok = false;
        failP = p;
        failR = r;
      }
      return;
    }

    if (k == "district") {
      const ToolApplyResult r = world.applyDistrict(p.x, p.y, arg);
      if (r != ToolApplyResult::Applied && r != ToolApplyResult::Noop) {
        ok = false;
        failP = p;
        failR = r;
      }
      return;
    }

    // Zones.
    Tool tool = Tool::Residential;
    if (k == "res" || k == "residential") tool = Tool::Residential;
    else if (k == "com" || k == "commercial") tool = Tool::Commercial;
    else if (k == "ind" || k == "industrial") tool = Tool::Industrial;

    ToolApplyResult r = ToolApplyResult::Noop;
    if (!ApplyZoneTile(world, tool, p.x, p.y, argOr1, &r)) {
      ok = false;
      failP = p;
      failR = r;
    }
  });

  if (!ok) {
    if (outFailP) *outFailP = failP;
    if (outFailR) *outFailR = failR;
  }

  return ok;
}

static bool ApplyOutline(World& world, const std::string& what, Point a, Point b, int arg, Point* outFailP = nullptr,
                         ToolApplyResult* outFailR = nullptr)
{
  const std::string k = ToLower(what);
  const int argOr1 = (arg > 0) ? arg : 1;

  const bool recognized = (k == "road" || k == "park" || k == "bulldoze" || k == "district" || k == "res" ||
                           k == "residential" || k == "com" || k == "commercial" || k == "ind" || k == "industrial");
  if (!recognized) return false;

  bool ok = true;
  Point failP{0, 0};
  ToolApplyResult failR = ToolApplyResult::Noop;

  ForEachRectOutline(a, b, [&](Point p) {
    if (!ok) return;
    if (!world.inBounds(p.x, p.y)) {
      ok = false;
      failP = p;
      failR = ToolApplyResult::OutOfBounds;
      return;
    }

    if (k == "road") {
      ToolApplyResult r = ToolApplyResult::Noop;
      if (!ApplyRoadTile(world, p.x, p.y, argOr1, &r)) {
        ok = false;
        failP = p;
        failR = r;
      }
      return;
    }

    if (k == "park") {
      const ToolApplyResult r = world.applyTool(Tool::Park, p.x, p.y);
      if (r != ToolApplyResult::Applied && r != ToolApplyResult::Noop) {
        ok = false;
        failP = p;
        failR = r;
      }
      return;
    }

    if (k == "bulldoze") {
      const ToolApplyResult r = world.applyTool(Tool::Bulldoze, p.x, p.y);
      if (r != ToolApplyResult::Applied && r != ToolApplyResult::Noop) {
        ok = false;
        failP = p;
        failR = r;
      }
      return;
    }

    if (k == "district") {
      const ToolApplyResult r = world.applyDistrict(p.x, p.y, arg);
      if (r != ToolApplyResult::Applied && r != ToolApplyResult::Noop) {
        ok = false;
        failP = p;
        failR = r;
      }
      return;
    }

    // Zones.
    Tool tool = Tool::Residential;
    if (k == "res" || k == "residential") tool = Tool::Residential;
    else if (k == "com" || k == "commercial") tool = Tool::Commercial;
    else if (k == "ind" || k == "industrial") tool = Tool::Industrial;

    ToolApplyResult r = ToolApplyResult::Noop;
    if (!ApplyZoneTile(world, tool, p.x, p.y, argOr1, &r)) {
      ok = false;
      failP = p;
      failR = r;
    }
  });

  if (!ok) {
    if (outFailP) *outFailP = failP;
    if (outFailR) *outFailR = failR;
  }

  return ok;
}

static bool ApplyFlood(World& world, const std::string& what, Point start, int arg, bool includeRoads,
                       Point* outFailP = nullptr, ToolApplyResult* outFailR = nullptr)
{
  const std::string k = ToLower(what);
  const int argOr1 = (arg > 0) ? arg : 1;

  const bool recognized = (k == "road" || k == "park" || k == "bulldoze" || k == "district" || k == "res" ||
                           k == "residential" || k == "com" || k == "commercial" || k == "ind" ||
                           k == "industrial");
  if (!recognized) {
    if (outFailP) *outFailP = start;
    if (outFailR) *outFailR = ToolApplyResult::Noop;
    return false;
  }

  if (!world.inBounds(start.x, start.y)) {
    if (outFailP) *outFailP = start;
    if (outFailR) *outFailR = ToolApplyResult::OutOfBounds;
    return false;
  }

  const FloodFillResult region = FloodFillAuto(world, start, includeRoads);

  bool ok = true;
  Point failP{0, 0};
  ToolApplyResult failR = ToolApplyResult::Noop;

  auto recordFail = [&](Point p, ToolApplyResult r) {
    ok = false;
    failP = p;
    failR = r;
  };

  for (const Point& p : region.tiles) {
    if (!ok) break;
    if (!world.inBounds(p.x, p.y)) continue;

    ToolApplyResult r = ToolApplyResult::Noop;

    if (k == "road") {
      if (!ApplyRoadTile(world, p.x, p.y, argOr1, &r)) recordFail(p, r);
    } else if (k == "park") {
      r = world.applyTool(Tool::Park, p.x, p.y);
      if (r != ToolApplyResult::Applied && r != ToolApplyResult::Noop) recordFail(p, r);
    } else if (k == "bulldoze") {
      r = world.applyTool(Tool::Bulldoze, p.x, p.y);
      if (r != ToolApplyResult::Applied && r != ToolApplyResult::Noop) recordFail(p, r);
    } else if (k == "district") {
      r = world.applyDistrict(p.x, p.y, arg);
      if (r != ToolApplyResult::Applied && r != ToolApplyResult::Noop) recordFail(p, r);
    } else if (k == "res" || k == "residential") {
      if (!ApplyZoneTile(world, Tool::Residential, p.x, p.y, argOr1, &r)) recordFail(p, r);
    } else if (k == "com" || k == "commercial") {
      if (!ApplyZoneTile(world, Tool::Commercial, p.x, p.y, argOr1, &r)) recordFail(p, r);
    } else if (k == "ind" || k == "industrial") {
      if (!ApplyZoneTile(world, Tool::Industrial, p.x, p.y, argOr1, &r)) recordFail(p, r);
    }
  }

  if (!ok) {
    if (outFailP) *outFailP = failP;
    if (outFailR) *outFailR = failR;
    return false;
  }

  return true;
}

static bool WriteDistrictsJsonFile(const std::string& path, const World& world, const SimConfig& simCfg)
{
  if (path.empty()) return true;

  // Derived fields for land-value-aware taxes.
  LandValueConfig lvc;
  lvc.requireOutsideConnection = simCfg.requireOutsideConnection;

  std::vector<std::uint8_t> roadToEdge;
  if (simCfg.requireOutsideConnection) {
    ComputeRoadsConnectedToEdge(world, roadToEdge);
  }

  const LandValueResult lv =
      ComputeLandValue(world, lvc, nullptr, simCfg.requireOutsideConnection ? &roadToEdge : nullptr);

  const DistrictStatsResult ds = ComputeDistrictStats(world, simCfg, &lv.value,
                                                     simCfg.requireOutsideConnection ? &roadToEdge : nullptr);

  std::ostringstream oss;
  oss << "{\n";
  oss << "  \"total\": {\n";
  oss << "    \"tiles\": " << ds.total.tiles << ",\n";
  oss << "    \"population\": " << ds.total.population << ",\n";
  oss << "    \"jobsCapacityAccessible\": " << ds.total.jobsCapacityAccessible << ",\n";
  oss << "    \"taxRevenue\": " << ds.total.taxRevenue << ",\n";
  oss << "    \"maintenanceCost\": " << ds.total.maintenanceCost << ",\n";
  oss << "    \"net\": " << ds.total.net << "\n";
  oss << "  },\n";
  oss << "  \"districts\": [\n";

  for (int i = 0; i < kDistrictCount; ++i) {
    const DistrictSummary& d = ds.districts[static_cast<std::size_t>(i)];
    oss << "    {\n";
    oss << "      \"id\": " << d.id << ",\n";
    oss << "      \"tiles\": " << d.tiles << ",\n";
    oss << "      \"population\": " << d.population << ",\n";
    oss << "      \"jobsCapacityAccessible\": " << d.jobsCapacityAccessible << ",\n";
    oss << "      \"avgLandValue\": " << d.avgLandValue << ",\n";
    oss << "      \"taxRevenue\": " << d.taxRevenue << ",\n";
    oss << "      \"maintenanceCost\": " << d.maintenanceCost << ",\n";
    oss << "      \"net\": " << d.net << "\n";
    oss << "    }";
    if (i != kDistrictCount - 1) oss << ',';
    oss << "\n";
  }

  oss << "  ]\n";
  oss << "}\n";

  std::ofstream f(path, std::ios::binary);
  if (!f) return false;
  f << oss.str();
  return static_cast<bool>(f);
}

} // namespace

ScriptRunner::ScriptRunner() = default;

void ScriptRunner::clearError()
{
  m_lastError.clear();
  m_lastErrorPath.clear();
  m_lastErrorLine = 0;
}

bool ScriptRunner::fail(const std::string& path, int line, const std::string& msg)
{
  m_lastErrorPath = path;
  m_lastErrorLine = line;
  m_lastError = path + ':' + std::to_string(line) + ": " + msg;
  emitError(m_lastError);
  return false;
}

void ScriptRunner::emitPrint(const std::string& line) const
{
  if (m_cb.print) m_cb.print(line);
}

void ScriptRunner::emitInfo(const std::string& line) const
{
  if (m_opt.quiet) return;
  if (m_cb.info) m_cb.info(line);
}

void ScriptRunner::emitError(const std::string& line) const
{
  if (m_cb.error) m_cb.error(line);
}

std::string ScriptRunner::expandPathTemplate(const std::string& tmpl, int run) const
{
  if (run < 0) run = m_ctx.runIndex;

  const std::uint64_t seed = m_ctx.hasWorld ? m_ctx.world.seed() : m_ctx.seed;
  const int w = m_ctx.hasWorld ? m_ctx.world.width() : m_ctx.w;
  const int h = m_ctx.hasWorld ? m_ctx.world.height() : m_ctx.h;
  const int day = m_ctx.hasWorld ? m_ctx.world.stats().day : 0;
  const int money = m_ctx.hasWorld ? m_ctx.world.stats().money : 0;

  bool hashComputed = false;
  std::uint64_t hashValue = 0;
  auto getHash = [&]() -> std::uint64_t {
    if (hashComputed) return hashValue;
    hashComputed = true;
    hashValue = m_ctx.hasWorld ? HashWorld(m_ctx.world, true) : 0;
    return hashValue;
  };

  constexpr int kMaxDepth = 8;

  auto expandRec = [&](const std::string& in, auto&& self, int depth) -> std::string {
    if (depth > kMaxDepth) return in;

    std::string out;
    out.reserve(in.size() + 16);

    for (std::size_t i = 0; i < in.size();) {
      if (in[i] == '{') {
        const std::size_t j = in.find('}', i + 1);
        if (j != std::string::npos) {
          const std::string rawKey = in.substr(i + 1, j - i - 1);
          const std::string key = ToLower(rawKey);

          // Built-ins (reserved)
          if (key == "seed") {
            out += std::to_string(seed);
            i = j + 1;
            continue;
          }
          if (key == "w") {
            out += std::to_string(w);
            i = j + 1;
            continue;
          }
          if (key == "h") {
            out += std::to_string(h);
            i = j + 1;
            continue;
          }
          if (key == "day") {
            out += std::to_string(day);
            i = j + 1;
            continue;
          }
          if (key == "money") {
            out += std::to_string(money);
            i = j + 1;
            continue;
          }
          if (key == "run") {
            out += std::to_string(run);
            i = j + 1;
            continue;
          }
          if (key == "hash") {
            out += HexU64(getHash());
            i = j + 1;
            continue;
          }

          // User vars (expanded recursively).
          const auto it = m_ctx.vars.find(key);
          if (it != m_ctx.vars.end()) {
            out += self(it->second, self, depth + 1);
            i = j + 1;
            continue;
          }

          // Unknown token -> keep verbatim.
          out.append(in, i, (j - i + 1));
          i = j + 1;
          continue;
        }
      }

      out.push_back(in[i]);
      ++i;
    }

    return out;
  };

  return expandRec(tmpl, expandRec, 0);
}

bool ScriptRunner::runFile(const std::string& path)
{
  clearError();
  return runFileInternal(path, 0);
}

bool ScriptRunner::runText(const std::string& text, const std::string& virtualPath)
{
  clearError();
  return runTextInternal(text, virtualPath, 0);
}

bool ScriptRunner::runFileInternal(const std::string& path, int depth)
{
  if (depth > m_opt.includeDepthLimit) {
    return fail(path, 1, "include depth limit exceeded");
  }

  std::ifstream f(path, std::ios::binary);
  if (!f) {
    return fail(path, 1, "failed to open script");
  }

  std::ostringstream oss;
  oss << f.rdbuf();
  return runTextInternal(oss.str(), path, depth);
}

static bool EnsureWorld(ScriptRunnerState& ctx, ScriptRunner& runner, const std::string& path, int lineNo)
{
  if (ctx.hasWorld) return true;
  return runner.fail(path, lineNo, "no world loaded/generated yet (use load/generate)");
}

static void RefreshIfDirty(ScriptRunnerState& ctx)
{
  if (!ctx.hasWorld) return;
  if (!ctx.dirtyDerived) return;
  ctx.sim.config() = ctx.simCfg;
  ctx.sim.refreshDerivedStats(ctx.world);
  ctx.dirtyDerived = false;
}

static bool CmdProc(ScriptRunnerState& ctx, ScriptRunner& runner, const std::vector<std::string>& t,
                    const std::string& path, int lineNo)
{
  if (t.size() != 3) {
    return runner.fail(path, lineNo, "proc expects: proc <key> <value>");
  }

  const std::string key = ToLower(t[1]);
  const std::string val = t[2];

  if (key == "terrainscale") {
    return ParseF32(val, &ctx.procCfg.terrainScale);
  }
  if (key == "waterlevel") {
    return ParseF32(val, &ctx.procCfg.waterLevel);
  }
  if (key == "sandlevel") {
    return ParseF32(val, &ctx.procCfg.sandLevel);
  }
  if (key == "hubs") {
    return ParseI32(val, &ctx.procCfg.hubs);
  }
  if (key == "extraconnections" || key == "extra_connections") {
    return ParseI32(val, &ctx.procCfg.extraConnections);
  }
  if (key == "roadlayout" || key == "road_layout") {
    ProcGenRoadLayout layout{};
    if (!ParseProcGenRoadLayout(val, layout)) {
      return runner.fail(path, lineNo, "unknown road_layout (try: organic|grid|radial|space_colonization)");
    }
    ctx.procCfg.roadLayout = layout;
    return true;
  }
  if (key == "zonechance" || key == "zone_chance") {
    return ParseF32(val, &ctx.procCfg.zoneChance);
  }
  if (key == "parkchance" || key == "park_chance") {
    return ParseF32(val, &ctx.procCfg.parkChance);
  }

  // Macro terrain presets (save v10+)
  if (key == "terrainpreset" || key == "terrain_preset" || key == "preset") {
    ProcGenTerrainPreset p{};
    if (!ParseProcGenTerrainPreset(val, p)) {
      return runner.fail(path, lineNo, "unknown terrain_preset (try: classic|island|archipelago|inland_sea|river_valley|mountain_ring)");
    }
    ctx.procCfg.terrainPreset = p;
    return true;
  }
  if (key == "terrainpresetstrength" || key == "terrain_preset_strength" || key == "presetstrength" ||
      key == "preset_strength") {
    float s = ctx.procCfg.terrainPresetStrength;
    if (!ParseF32(val, &s)) return false;
    ctx.procCfg.terrainPresetStrength = std::clamp(s, 0.0f, 5.0f);
    return true;
  }

  // Procedural road hierarchy pass (v11).
  if (key == "roadhierarchy" || key == "road_hierarchy" || key == "road_hierarchy_enabled" ||
      key == "road_hierarchy_enable") {
    bool b = false;
    if (!ParseBool01(val, &b)) return false;
    ctx.procCfg.roadHierarchyEnabled = b;
    return true;
  }
  if (key == "roadhierarchystrength" || key == "road_hierarchy_strength" || key == "road_strength" ||
      key == "road_hierarchy_str") {
    float s = ctx.procCfg.roadHierarchyStrength;
    if (!ParseF32(val, &s)) return false;
    ctx.procCfg.roadHierarchyStrength = std::clamp(s, 0.0f, 3.0f);
    return true;
  }

  // Procedural district assignment (v12).
  if (key == "districtingmode" || key == "districting_mode" || key == "district_mode" ||
      key == "districts_mode") {
    ProcGenDistrictingMode mode{};
    if (!ParseProcGenDistrictingMode(val, mode)) {
      return runner.fail(path, lineNo, "proc: districting_mode expects one of: voronoi|road_flow|block_graph");
    }
    ctx.procCfg.districtingMode = mode;
    return true;
  }

  // --- Erosion controls (new in save v9 / patch v2) ---
  if (key == "erosion" || key == "erosion_enabled" || key == "erode") {
    bool b = false;
    if (!ParseBool01(val, &b)) return false;
    ctx.procCfg.erosion.enabled = b;
    return true;
  }
  if (key == "rivers" || key == "rivers_enabled") {
    bool b = false;
    if (!ParseBool01(val, &b)) return false;
    ctx.procCfg.erosion.riversEnabled = b;
    return true;
  }

  if (key == "thermaliters" || key == "thermal_iterations" || key == "erosion_thermal_iters") {
    return ParseI32(val, &ctx.procCfg.erosion.thermalIterations);
  }
  if (key == "thermaltalus" || key == "thermal_talus" || key == "erosion_talus") {
    return ParseF32(val, &ctx.procCfg.erosion.thermalTalus);
  }
  if (key == "thermalrate" || key == "thermal_rate" || key == "erosion_rate") {
    return ParseF32(val, &ctx.procCfg.erosion.thermalRate);
  }

  if (key == "riverminaccum" || key == "river_min_accum" || key == "river_minaccum") {
    return ParseI32(val, &ctx.procCfg.erosion.riverMinAccum);
  }
  if (key == "rivercarve" || key == "river_carve") {
    return ParseF32(val, &ctx.procCfg.erosion.riverCarve);
  }
  if (key == "riverpower" || key == "river_power") {
    return ParseF32(val, &ctx.procCfg.erosion.riverCarvePower);
  }

  if (key == "smoothiters" || key == "smooth_iterations" || key == "erosion_smooth_iters") {
    return ParseI32(val, &ctx.procCfg.erosion.smoothIterations);
  }
  if (key == "smoothrate" || key == "smooth_rate") {
    return ParseF32(val, &ctx.procCfg.erosion.smoothRate);
  }

  if (key == "quantizescale" || key == "quantize_scale" || key == "erosion_quantize_scale") {
    return ParseI32(val, &ctx.procCfg.erosion.quantizeScale);
  }

  return runner.fail(path, lineNo, "unknown proc key: " + t[1]);
}

static bool CmdSim(ScriptRunnerState& ctx, ScriptRunner& runner, const std::vector<std::string>& t,
                   const std::string& path, int lineNo)
{
  if (t.size() != 3) {
    return runner.fail(path, lineNo, "sim expects: sim <key> <value>");
  }

  const std::string key = ToLower(t[1]);
  const std::string val = t[2];

  if (key == "tickseconds" || key == "tick_seconds") {
    return ParseF32(val, &ctx.simCfg.tickSeconds);
  }
  if (key == "parkinfluenceradius" || key == "park_influence_radius") {
    return ParseI32(val, &ctx.simCfg.parkInfluenceRadius);
  }
  if (key == "requireoutsideconnection" || key == "require_outside_connection" || key == "require_outside") {
    bool b = false;
    if (!ParseBool01(val, &b)) return false;
    ctx.simCfg.requireOutsideConnection = b;
    return true;
  }

  if (key == "taxresidential" || key == "tax_residential" || key == "tax_res") {
    return ParseI32(val, &ctx.simCfg.taxResidential);
  }
  if (key == "taxcommercial" || key == "tax_commercial" || key == "tax_com") {
    return ParseI32(val, &ctx.simCfg.taxCommercial);
  }
  if (key == "taxindustrial" || key == "tax_industrial" || key == "tax_ind") {
    return ParseI32(val, &ctx.simCfg.taxIndustrial);
  }

  if (key == "maintenanceroad" || key == "maintenance_road" || key == "maint_road") {
    return ParseI32(val, &ctx.simCfg.maintenanceRoad);
  }
  if (key == "maintenancepark" || key == "maintenance_park" || key == "maint_park") {
    return ParseI32(val, &ctx.simCfg.maintenancePark);
  }

  if (key == "taxhappinesspercapita" || key == "tax_happiness_per_capita") {
    return ParseF32(val, &ctx.simCfg.taxHappinessPerCapita);
  }

  if (key == "residentialdesirabilityweight" || key == "residential_desirability_weight") {
    return ParseF32(val, &ctx.simCfg.residentialDesirabilityWeight);
  }
  if (key == "commercialdesirabilityweight" || key == "commercial_desirability_weight") {
    return ParseF32(val, &ctx.simCfg.commercialDesirabilityWeight);
  }
  if (key == "industrialdesirabilityweight" || key == "industrial_desirability_weight") {
    return ParseF32(val, &ctx.simCfg.industrialDesirabilityWeight);
  }

  if (key == "districtpoliciesenabled" || key == "district_policies_enabled") {
    bool b = false;
    if (!ParseBool01(val, &b)) return false;
    ctx.simCfg.districtPoliciesEnabled = b;
    return true;
  }

  return runner.fail(path, lineNo, "unknown sim key: " + t[1]);
}

static bool CmdPolicy(ScriptRunnerState& ctx, ScriptRunner& runner, const std::vector<std::string>& t,
                      const std::string& path, int lineNo)
{
  if (t.size() != 4) {
    return runner.fail(path, lineNo, "policy expects: policy <districtId> <key> <value>");
  }

  int id = 0;
  if (!ParseI32(t[1], &id)) return false;
  id = std::clamp(id, 0, kDistrictCount - 1);

  const std::string key = ToLower(t[2]);
  const std::string val = t[3];

  DistrictPolicy& p = ctx.simCfg.districtPolicies[static_cast<std::size_t>(id)];

  if (key == "taxresidentialmult" || key == "tax_residential_mult") return ParseF32(val, &p.taxResidentialMult);
  if (key == "taxcommercialmult" || key == "tax_commercial_mult") return ParseF32(val, &p.taxCommercialMult);
  if (key == "taxindustrialmult" || key == "tax_industrial_mult") return ParseF32(val, &p.taxIndustrialMult);
  if (key == "roadmaintenancemult" || key == "road_maintenance_mult") return ParseF32(val, &p.roadMaintenanceMult);
  if (key == "parkmaintenancemult" || key == "park_maintenance_mult") return ParseF32(val, &p.parkMaintenanceMult);

  return runner.fail(path, lineNo, "unknown policy key: " + t[2]);
}

static bool CmdTrafficModel(ScriptRunnerState& ctx, ScriptRunner& runner, const std::vector<std::string>& t,
                            const std::string& path, int lineNo)
{
  if (t.size() != 3) {
    return runner.fail(path, lineNo, "traffic_model expects: traffic_model <key> <value>");
  }

  const std::string key = ToLower(t[1]);
  const std::string val = t[2];

  TrafficModelSettings& tm = ctx.sim.trafficModel();

  if (key == "congestionawarerouting" || key == "congestion_aware_routing") {
    bool b = false;
    if (!ParseBool01(val, &b)) return false;
    tm.congestionAwareRouting = b;
    return true;
  }
  if (key == "congestioniterations" || key == "congestion_iterations") {
    return ParseI32(val, &tm.congestionIterations);
  }
  if (key == "congestionalpha" || key == "congestion_alpha") {
    return ParseF32(val, &tm.congestionAlpha);
  }
  if (key == "congestionbeta" || key == "congestion_beta") {
    return ParseF32(val, &tm.congestionBeta);
  }
  if (key == "congestioncapacityscale" || key == "congestion_capacity_scale") {
    return ParseF32(val, &tm.congestionCapacityScale);
  }
  if (key == "congestionratioclamp" || key == "congestion_ratio_clamp") {
    return ParseF32(val, &tm.congestionRatioClamp);
  }

  return runner.fail(path, lineNo, "unknown traffic_model key: " + t[1]);
}

bool ScriptRunner::runTextInternal(const std::string& text, const std::string& virtualPath, int depth)
{
  if (depth > m_opt.includeDepthLimit) {
    return fail(virtualPath, 1, "include depth limit exceeded");
  }

  // ---- Parse the script into stable, executable lines (comment/blank lines removed) ----
  struct ScriptLine {
    int lineNo = 0;
    std::vector<std::string> tokens;
  };

  std::vector<ScriptLine> lines;
  {
    std::istringstream iss(text);
    std::string line;
    int lineNo = 0;
    while (std::getline(iss, line)) {
      lineNo++;

      // Strip comments.
      const std::size_t hashPos = line.find('#');
      if (hashPos != std::string::npos) line = line.substr(0, hashPos);
      line = Trim(line);
      if (line.empty()) continue;

      std::vector<std::string> t = SplitWS(line);
      if (t.empty()) continue;

      lines.push_back(ScriptLine{lineNo, std::move(t)});
    }
  }

  // ---- Precompute block structure for control flow ----
  enum class CtrlKind : std::uint8_t {
    None = 0,
    Repeat,
    While,
    If,
    Else,
    End,
  };

  struct CtrlInfo {
    CtrlKind kind = CtrlKind::None;
    int endIndex = -1;   // repeat/while/if/else -> matching end
    int elseIndex = -1;  // if -> else line (optional)
  };

  std::vector<CtrlInfo> ctrl(lines.size());

  struct BlockEntry {
    CtrlKind kind = CtrlKind::None;
    int startIndex = -1;
    int elseIndex = -1;
  };

  std::vector<BlockEntry> blockStack;
  blockStack.reserve(32);

  for (int i = 0; i < static_cast<int>(lines.size()); ++i) {
    const auto& t = lines[static_cast<std::size_t>(i)].tokens;
    if (t.empty()) continue;

    const std::string cmd = ToLower(t[0]);

    if (cmd == "repeat") {
      ctrl[static_cast<std::size_t>(i)].kind = CtrlKind::Repeat;
      blockStack.push_back(BlockEntry{CtrlKind::Repeat, i, -1});
      continue;
    }
    if (cmd == "while") {
      ctrl[static_cast<std::size_t>(i)].kind = CtrlKind::While;
      blockStack.push_back(BlockEntry{CtrlKind::While, i, -1});
      continue;
    }
    if (cmd == "if") {
      ctrl[static_cast<std::size_t>(i)].kind = CtrlKind::If;
      blockStack.push_back(BlockEntry{CtrlKind::If, i, -1});
      continue;
    }
    if (cmd == "else") {
      ctrl[static_cast<std::size_t>(i)].kind = CtrlKind::Else;

      if (blockStack.empty() || blockStack.back().kind != CtrlKind::If) {
        return fail(virtualPath, lines[static_cast<std::size_t>(i)].lineNo, "else without matching if");
      }
      if (blockStack.back().elseIndex != -1) {
        return fail(virtualPath, lines[static_cast<std::size_t>(i)].lineNo, "else already used for this if");
      }
      blockStack.back().elseIndex = i;
      continue;
    }
    if (cmd == "end") {
      ctrl[static_cast<std::size_t>(i)].kind = CtrlKind::End;

      if (blockStack.empty()) {
        return fail(virtualPath, lines[static_cast<std::size_t>(i)].lineNo, "end without matching block start");
      }

      const BlockEntry top = blockStack.back();
      blockStack.pop_back();

      // Match start -> end.
      ctrl[static_cast<std::size_t>(top.startIndex)].endIndex = i;

      // Match if -> else/end.
      if (top.kind == CtrlKind::If) {
        ctrl[static_cast<std::size_t>(top.startIndex)].elseIndex = top.elseIndex;
        if (top.elseIndex != -1) {
          ctrl[static_cast<std::size_t>(top.elseIndex)].endIndex = i;
        }
      }

      continue;
    }
  }

  if (!blockStack.empty()) {
    const BlockEntry top = blockStack.back();
    const int startLineNo = lines[static_cast<std::size_t>(top.startIndex)].lineNo;
    const std::string startCmd = ToLower(lines[static_cast<std::size_t>(top.startIndex)].tokens[0]);
    return fail(virtualPath, startLineNo, "missing end for block: " + startCmd);
  }

  auto joinTokens = [](const std::vector<std::string>& t, std::size_t start) {
    std::string out;
    for (std::size_t i = start; i < t.size(); ++i) {
      if (i > start) out.push_back(' ');
      out += t[i];
    }
    return out;
  };

  struct LoopFrame {
    enum class Kind : std::uint8_t { Repeat = 0, While };
    Kind kind = Kind::Repeat;
    int startIndex = 0;   // index of the repeat/while line
    int endIndex = 0;     // index of the matching end line
    int remaining = 0;    // repeat only
  };

  std::vector<LoopFrame> loopStack;
  loopStack.reserve(16);

  // ---- Execute with a movable instruction pointer (enables loops/conditionals) ----
  int ip = 0;
  while (ip < static_cast<int>(lines.size())) {
    const int cur = ip;
    ip = cur + 1;  // default: advance to next executable line

    const int lineNo = lines[static_cast<std::size_t>(cur)].lineNo;
    std::vector<std::string> t = lines[static_cast<std::size_t>(cur)].tokens;
    if (t.empty()) continue;

    const std::string cmd = ToLower(t[0]);

    // Expand {tokens} / {vars} in arguments (but keep raw template for `set` values).
    for (std::size_t i = 1; i < t.size(); ++i) {
      if (cmd == "set" && i == 2) continue;
      t[i] = expandPathTemplate(t[i]);
    }

    auto isReservedVar = [](const std::string& k) -> bool {
      return k == "seed" || k == "w" || k == "h" || k == "day" || k == "money" || k == "run" || k == "hash";
    };

    auto isValidVarName = [](const std::string& name) -> bool {
      if (name.empty()) return false;
      auto isStart = [](unsigned char c) { return (std::isalpha(c) != 0) || c == '_'; };
      auto isChar = [](unsigned char c) { return (std::isalnum(c) != 0) || c == '_'; };
      if (!isStart(static_cast<unsigned char>(name[0]))) return false;
      for (std::size_t i = 1; i < name.size(); ++i) {
        if (!isChar(static_cast<unsigned char>(name[i]))) return false;
      }
      return true;
    };

    // ---- Control flow ----
    if (cmd == "repeat") {
      if (t.size() < 2) {
        return fail(virtualPath, lineNo, "repeat expects: repeat <countExpr>");
      }
      const int endIndex = ctrl[static_cast<std::size_t>(cur)].endIndex;
      if (endIndex < 0) {
        return fail(virtualPath, lineNo, "repeat missing matching end");
      }
      const std::string expr = joinTokens(t, 1);
      int n = 0;
      if (!ParseI32(expr, &n) || n < 0) {
        return fail(virtualPath, lineNo, "repeat: invalid non-negative count expression");
      }
      if (n == 0) {
        ip = endIndex + 1;
        continue;
      }
      loopStack.push_back(LoopFrame{LoopFrame::Kind::Repeat, cur, endIndex, n});
      continue;
    }

    if (cmd == "while") {
      if (t.size() < 2) {
        return fail(virtualPath, lineNo, "while expects: while <conditionExpr>");
      }
      const int endIndex = ctrl[static_cast<std::size_t>(cur)].endIndex;
      if (endIndex < 0) {
        return fail(virtualPath, lineNo, "while missing matching end");
      }

      const bool isRecheck = (!loopStack.empty() && loopStack.back().kind == LoopFrame::Kind::While && loopStack.back().startIndex == cur);

      const std::string expr = joinTokens(t, 1);
      std::int64_t v = 0;
      if (!EvalI64LogicExpr(expr, &v)) {
        return fail(virtualPath, lineNo, "while: invalid condition expression");
      }
      const bool cond = (v != 0);

      if (!cond) {
        if (isRecheck) {
          loopStack.pop_back();
        }
        ip = endIndex + 1;
        continue;
      }

      if (!isRecheck) {
        loopStack.push_back(LoopFrame{LoopFrame::Kind::While, cur, endIndex, 0});
      }
      continue;
    }

    if (cmd == "if") {
      if (t.size() < 2) {
        return fail(virtualPath, lineNo, "if expects: if <conditionExpr>");
      }
      const int endIndex = ctrl[static_cast<std::size_t>(cur)].endIndex;
      if (endIndex < 0) {
        return fail(virtualPath, lineNo, "if missing matching end");
      }

      const std::string expr = joinTokens(t, 1);
      std::int64_t v = 0;
      if (!EvalI64LogicExpr(expr, &v)) {
        return fail(virtualPath, lineNo, "if: invalid condition expression");
      }
      const bool cond = (v != 0);
      if (cond) {
        continue;  // fall through into then-block
      }

      const int elseIndex = ctrl[static_cast<std::size_t>(cur)].elseIndex;
      if (elseIndex >= 0) {
        ip = elseIndex + 1;
      } else {
        ip = endIndex + 1;
      }
      continue;
    }

    if (cmd == "else") {
      if (t.size() != 1) {
        return fail(virtualPath, lineNo, "else expects: else");
      }
      const int endIndex = ctrl[static_cast<std::size_t>(cur)].endIndex;
      if (endIndex < 0) {
        return fail(virtualPath, lineNo, "else missing matching end");
      }
      // We only execute `else` when the if-branch ran; skip the else-block.
      ip = endIndex + 1;
      continue;
    }

    if (cmd == "end") {
      if (t.size() != 1) {
        return fail(virtualPath, lineNo, "end expects: end");
      }

      if (!loopStack.empty() && loopStack.back().endIndex == cur) {
        LoopFrame& lf = loopStack.back();
        if (lf.kind == LoopFrame::Kind::Repeat) {
          lf.remaining--;
          if (lf.remaining > 0) {
            ip = lf.startIndex + 1;
            continue;
          }
          loopStack.pop_back();
          continue;
        }
        if (lf.kind == LoopFrame::Kind::While) {
          ip = lf.startIndex;
          continue;
        }
      }

      continue;
    }

    if (cmd == "break") {
      if (t.size() != 1) {
        return fail(virtualPath, lineNo, "break expects: break");
      }
      if (loopStack.empty()) {
        return fail(virtualPath, lineNo, "break used outside a loop");
      }
      const LoopFrame lf = loopStack.back();
      loopStack.pop_back();
      ip = lf.endIndex + 1;
      continue;
    }

    if (cmd == "continue") {
      if (t.size() != 1) {
        return fail(virtualPath, lineNo, "continue expects: continue");
      }
      if (loopStack.empty()) {
        return fail(virtualPath, lineNo, "continue used outside a loop");
      }
      ip = loopStack.back().endIndex;
      continue;
    }

    if (cmd == "expect") {
      if (t.size() < 2) {
        return fail(virtualPath, lineNo, "expect expects: expect <conditionExpr>");
      }
      const std::string expr = joinTokens(t, 1);
      std::int64_t v = 0;
      if (!EvalI64LogicExpr(expr, &v)) {
        return fail(virtualPath, lineNo, "expect: invalid condition expression");
      }
      if (v == 0) {
        return fail(virtualPath, lineNo, "expect failed: " + expr);
      }
      continue;
    }

    if (cmd == "set") {
      if (t.size() != 3) {
        return fail(virtualPath, lineNo, "set expects: set <name> <value>");
      }
      const std::string name = ToLower(t[1]);
      if (!isValidVarName(name)) {
        return fail(virtualPath, lineNo, "set: invalid name (expected [A-Za-z_][A-Za-z0-9_]*)");
      }
      if (isReservedVar(name)) {
        return fail(virtualPath, lineNo, "set: name is reserved (seed,w,h,day,money,run,hash)");
      }
      m_ctx.vars[name] = t[2];
      emitInfo("set: " + name);
      continue;
    }

    if (cmd == "unset") {
      if (t.size() != 2) {
        return fail(virtualPath, lineNo, "unset expects: unset <name>");
      }
      const std::string name = ToLower(t[1]);
      m_ctx.vars.erase(name);
      emitInfo("unset: " + name);
      continue;
    }

    if (cmd == "add") {
      if (t.size() != 3) {
        return fail(virtualPath, lineNo, "add expects: add <name> <delta>");
      }
      const std::string name = ToLower(t[1]);
      if (!isValidVarName(name)) {
        return fail(virtualPath, lineNo, "add: invalid name (expected [A-Za-z_][A-Za-z0-9_]*)");
      }
      if (isReservedVar(name)) {
        return fail(virtualPath, lineNo, "add: name is reserved (seed,w,h,day,money,run,hash)");
      }
      int delta = 0;
      if (!ParseI32(t[2], &delta)) {
        return fail(virtualPath, lineNo, "add expects integer delta (supports +,-,*,/,%, parentheses)");
      }

      int curVal = 0;
      const auto it = m_ctx.vars.find(name);
      if (it != m_ctx.vars.end()) {
        const std::string expanded = expandPathTemplate(it->second);
        if (!ParseI32(expanded, &curVal)) {
          return fail(virtualPath, lineNo, "add: current variable value is not an integer");
        }
      }

      const long long sum = static_cast<long long>(curVal) + static_cast<long long>(delta);
      m_ctx.vars[name] = std::to_string(sum);
      emitInfo("add: " + name + "=" + std::to_string(sum));
      continue;
    }

    if (cmd == "echo") {
      std::ostringstream oss;
      for (std::size_t i = 1; i < t.size(); ++i) {
        if (i > 1) oss << ' ';
        oss << t[i];
      }
      emitPrint(oss.str());
      continue;
    }

    if (cmd == "vars") {
      for (const auto& kv : m_ctx.vars) {
        emitPrint(kv.first + "=" + expandPathTemplate(kv.second));
      }
      continue;
    }

    if (cmd == "include") {
      if (t.size() != 2) {
        return fail(virtualPath, lineNo, "include expects: include <script.txt>");
      }
      // Resolve relative includes against the including script's directory.
      std::filesystem::path inc = std::filesystem::path(t[1]);
      if (inc.is_relative()) {
        std::filesystem::path base = std::filesystem::path(virtualPath).parent_path();
        if (!base.empty()) inc = base / inc;
      }
      if (!runFileInternal(inc.string(), depth + 1)) {
        // runFileInternal already populated m_lastError.
        return false;
      }
      continue;
    }

    if (cmd == "size") {
      if (t.size() != 2 || !ParseWxH(t[1], &m_ctx.w, &m_ctx.h)) {
        return fail(virtualPath, lineNo, "size expects WxH");
      }
      continue;
    }

    if (cmd == "seed") {
      if (t.size() != 2 || !ParseU64(t[1], &m_ctx.seed)) {
        return fail(virtualPath, lineNo, "seed expects u64 (decimal or 0x...)");
      }
      continue;
    }

    if (cmd == "proc") {
      if (!CmdProc(m_ctx, *this, t, virtualPath, lineNo)) return false;
      continue;
    }

    if (cmd == "sim") {
      if (!CmdSim(m_ctx, *this, t, virtualPath, lineNo)) return false;
      m_ctx.dirtyDerived = true;
      continue;
    }

    if (cmd == "policy") {
      if (!CmdPolicy(m_ctx, *this, t, virtualPath, lineNo)) return false;
      m_ctx.dirtyDerived = true;
      continue;
    }

    if (cmd == "bot") {
      if (t.size() == 2) {
        const std::string sub = ToLower(t[1]);
        if (sub == "reset") {
          m_ctx.autoBuildCfg = AutoBuildConfig{};
          emitInfo("bot: reset");
          continue;
        }
        if (sub == "show") {
          std::ostringstream oss;
          oss << "{\n"
              << "  \"zonesPerDay\": " << m_ctx.autoBuildCfg.zonesPerDay << ",\n"
              << "  \"zoneClusterMaxTiles\": " << m_ctx.autoBuildCfg.zoneClusterMaxTiles << ",\n"
              << "  \"roadsPerDay\": " << m_ctx.autoBuildCfg.roadsPerDay << ",\n"
              << "  \"parksPerDay\": " << m_ctx.autoBuildCfg.parksPerDay << ",\n"
              << "  \"useParkOptimizer\": " << (m_ctx.autoBuildCfg.useParkOptimizer ? "true" : "false") << ",\n"
              << "  \"roadLevel\": " << m_ctx.autoBuildCfg.roadLevel << ",\n"
              << "  \"useRoadPlanner\": " << (m_ctx.autoBuildCfg.useRoadPlanner ? "true" : "false") << ",\n"
              << "  \"maxRoadSpurLength\": " << m_ctx.autoBuildCfg.maxRoadSpurLength << ",\n"
              << "  \"allowBridges\": " << (m_ctx.autoBuildCfg.allowBridges ? "true" : "false") << ",\n"
              << "  \"minMoneyReserve\": " << m_ctx.autoBuildCfg.minMoneyReserve << ",\n"
              << "  \"parkPerZoneTiles\": " << m_ctx.autoBuildCfg.parkPerZoneTiles << ",\n"
              << "  \"autoUpgradeRoads\": " << (m_ctx.autoBuildCfg.autoUpgradeRoads ? "true" : "false") << ",\n"
              << "  \"congestionUpgradeThreshold\": " << m_ctx.autoBuildCfg.congestionUpgradeThreshold << ",\n"
              << "  \"roadUpgradesPerDay\": " << m_ctx.autoBuildCfg.roadUpgradesPerDay << ",\n"
              << "  \"landValueRecalcDays\": " << m_ctx.autoBuildCfg.landValueRecalcDays << ",\n"
              << "  \"respectOutsideConnection\": " << (m_ctx.autoBuildCfg.respectOutsideConnection ? "true" : "false") << ",\n"
              << "  \"ensureOutsideConnection\": " << (m_ctx.autoBuildCfg.ensureOutsideConnection ? "true" : "false") << "\n"
              << "}\n";
          emitPrint(oss.str());
          continue;
        }
      }

      if (t.size() != 3) {
        return fail(virtualPath, lineNo, "bot expects: bot <key> <value> (or: bot show / bot reset)");
      }
      std::string err;
      if (!ParseAutoBuildKey(t[1], t[2], m_ctx.autoBuildCfg, err)) {
        return fail(virtualPath, lineNo, "bot parse error: " + err);
      }
      emitInfo("bot: set " + t[1] + "=" + t[2]);
      continue;
    }

    if (cmd == "traffic_model") {
      if (!CmdTrafficModel(m_ctx, *this, t, virtualPath, lineNo)) return false;
      m_ctx.dirtyDerived = true;
      continue;
    }

    if (cmd == "load") {
      if (t.size() != 2) {
        return fail(virtualPath, lineNo, "load expects: load <save.bin>");
      }

      std::string err;
      const std::string p = expandPathTemplate(t[1]);
      World w;
      ProcGenConfig pc{};
      SimConfig sc{};
      if (!LoadWorldBinary(w, pc, sc, p, err)) {
        return fail(virtualPath, lineNo, "load failed: " + err);
      }

      m_ctx.world = std::move(w);
      m_ctx.procCfg = pc;
      m_ctx.simCfg = sc;
      m_ctx.sim.config() = m_ctx.simCfg;
      m_ctx.sim.resetTimer();
      m_ctx.sim.refreshDerivedStats(m_ctx.world);
      m_ctx.hasWorld = true;
      m_ctx.dirtyDerived = false;
      emitInfo("loaded: " + p);
      continue;
    }

    if (cmd == "generate") {
      const std::uint64_t seed = (m_ctx.seed == 0) ? 1 : m_ctx.seed;
      m_ctx.world = GenerateWorld(m_ctx.w, m_ctx.h, seed, m_ctx.procCfg);
      // Actual seed may differ (GenerateWorld keeps what you pass, but stay consistent).
      m_ctx.seed = m_ctx.world.seed();
      m_ctx.sim.config() = m_ctx.simCfg;
      m_ctx.sim.resetTimer();
      m_ctx.sim.refreshDerivedStats(m_ctx.world);
      m_ctx.hasWorld = true;
      m_ctx.dirtyDerived = false;
      emitInfo("generated: " + std::to_string(m_ctx.w) + "x" + std::to_string(m_ctx.h) + " seed=" +
               std::to_string(m_ctx.world.seed()));
      continue;
    }

    if (cmd == "save") {
      if (!EnsureWorld(m_ctx, *this, virtualPath, lineNo)) return false;
      if (t.size() != 2) {
        return fail(virtualPath, lineNo, "save expects: save <out.bin>");
      }
      const std::string p = expandPathTemplate(t[1]);
      std::string err;
      if (!SaveWorldBinary(m_ctx.world, m_ctx.procCfg, m_ctx.simCfg, p, err)) {
        return fail(virtualPath, lineNo, "save failed: " + err);
      }
      emitInfo("saved: " + p);
      continue;
    }

    // --- Blueprint commands (bp_*) ---
    if (cmd == "bp_clear") {
      m_ctx.blueprint = Blueprint{};
      m_ctx.hasBlueprint = false;
      emitInfo("bp: cleared");
      continue;
    }

    if (cmd == "bp_info") {
      if (!m_ctx.hasBlueprint) {
        return fail(virtualPath, lineNo, "bp_info: no blueprint loaded");
      }
      std::ostringstream oss;
      oss << "bp: " << m_ctx.blueprint.width << "x" << m_ctx.blueprint.height
          << " deltas=" << m_ctx.blueprint.tiles.size();
      emitInfo(oss.str());
      continue;
    }

    if (cmd == "bp_capture") {
      if (!EnsureWorld(m_ctx, *this, virtualPath, lineNo)) return false;
      if (t.size() < 5) {
        return fail(virtualPath, lineNo, "bp_capture expects: bp_capture x0 y0 w h [key value]...");
      }
      int x0 = 0, y0 = 0, w = 0, h = 0;
      if (!ParseI32(t[1], &x0) || !ParseI32(t[2], &y0) || !ParseI32(t[3], &w) || !ParseI32(t[4], &h)) {
        return fail(virtualPath, lineNo, "bp_capture: expected integers x0 y0 w h");
      }

      BlueprintCaptureOptions opt;
      opt.fieldMask = static_cast<std::uint8_t>(TileFieldMask::Overlay) |
                      static_cast<std::uint8_t>(TileFieldMask::Level) |
                      static_cast<std::uint8_t>(TileFieldMask::District) |
                      static_cast<std::uint8_t>(TileFieldMask::Variation);
      opt.sparseByOverlay = true;
      opt.zeroOccupants = true;

      if (((t.size() - 5) % 2) != 0) {
        return fail(virtualPath, lineNo, "bp_capture: options must be key/value pairs");
      }

      for (std::size_t i = 5; i + 1 < t.size(); i += 2) {
        const std::string key = ToLower(t[i]);
        const std::string val = t[i + 1];
        if (key == "fields") {
          std::string err;
          if (!ParseTileFieldMaskList(val, &opt.fieldMask, &err)) {
            return fail(virtualPath, lineNo, "bp_capture: " + err);
          }
        } else if (key == "sparse") {
          bool b = false;
          if (!ParseBool01(val, &b)) {
            return fail(virtualPath, lineNo, "bp_capture: sparse expects 0|1");
          }
          opt.sparseByOverlay = b;
        } else if (key == "zero_occ") {
          bool b = false;
          if (!ParseBool01(val, &b)) {
            return fail(virtualPath, lineNo, "bp_capture: zero_occ expects 0|1");
          }
          opt.zeroOccupants = b;
        } else {
          return fail(virtualPath, lineNo, "bp_capture: unknown option: " + key);
        }
      }

      Blueprint bp;
      std::string err;
      if (!CaptureBlueprintRect(m_ctx.world, x0, y0, w, h, bp, err, opt)) {
        return fail(virtualPath, lineNo, "bp_capture failed: " + err);
      }

      m_ctx.blueprint = std::move(bp);
      m_ctx.hasBlueprint = true;

      std::ostringstream oss;
      oss << "bp: captured " << w << "x" << h << " deltas=" << m_ctx.blueprint.tiles.size();
      emitInfo(oss.str());
      continue;
    }

    if (cmd == "bp_save") {
      if (!m_ctx.hasBlueprint) {
        return fail(virtualPath, lineNo, "bp_save: no blueprint loaded");
      }
      if (t.size() < 2) {
        return fail(virtualPath, lineNo, "bp_save expects: bp_save <out.isobp> [compress none|sllz]");
      }
      const std::string p = expandPathTemplate(t[1]);

      BlueprintCompression comp = BlueprintCompression::SLLZ;
      if (t.size() > 2) {
        if (t.size() != 4 || ToLower(t[2]) != "compress") {
          return fail(virtualPath, lineNo, "bp_save expects: bp_save <out.isobp> [compress none|sllz]");
        }
        const std::string c = ToLower(t[3]);
        if (c == "none") comp = BlueprintCompression::None;
        else if (c == "sllz") comp = BlueprintCompression::SLLZ;
        else return fail(virtualPath, lineNo, "bp_save: compress expects none|sllz");
      }

      std::string err;
      if (!SaveBlueprintBinary(m_ctx.blueprint, p, err, comp)) {
        return fail(virtualPath, lineNo, "bp_save failed: " + err);
      }
      emitInfo("bp: saved -> " + p);
      continue;
    }

    if (cmd == "bp_load") {
      if (t.size() != 2) {
        return fail(virtualPath, lineNo, "bp_load expects: bp_load <bp.isobp>");
      }
      const std::string p = expandPathTemplate(t[1]);
      std::string err;
      Blueprint bp;
      if (!LoadBlueprintBinary(bp, p, err)) {
        return fail(virtualPath, lineNo, "bp_load failed: " + err);
      }
      m_ctx.blueprint = std::move(bp);
      m_ctx.hasBlueprint = true;
      emitInfo("bp: loaded -> " + p);
      continue;
    }

    if (cmd == "bp_apply") {
      if (!EnsureWorld(m_ctx, *this, virtualPath, lineNo)) return false;
      if (!m_ctx.hasBlueprint) {
        return fail(virtualPath, lineNo, "bp_apply: no blueprint loaded");
      }
      if (t.size() < 3) {
        return fail(virtualPath, lineNo, "bp_apply expects: bp_apply dstX dstY [key value]...");
      }
      int dstX = 0, dstY = 0;
      if (!ParseI32(t[1], &dstX) || !ParseI32(t[2], &dstY)) {
        return fail(virtualPath, lineNo, "bp_apply: expected integers dstX dstY");
      }

      BlueprintApplyOptions opt;
      opt.mode = BlueprintApplyMode::Stamp;
      opt.fieldMask = 0xFFu;
      opt.allowOutOfBounds = false;
      opt.force = true;
      opt.recomputeRoadMasks = true;
      opt.transform.rotateDeg = 0;
      opt.transform.mirrorX = false;
      opt.transform.mirrorY = false;

      if (((t.size() - 3) % 2) != 0) {
        return fail(virtualPath, lineNo, "bp_apply: options must be key/value pairs");
      }

      for (std::size_t i = 3; i + 1 < t.size(); i += 2) {
        const std::string key = ToLower(t[i]);
        const std::string val = t[i + 1];
        if (key == "mode") {
          const std::string m = ToLower(val);
          if (m == "replace") opt.mode = BlueprintApplyMode::Replace;
          else if (m == "stamp") opt.mode = BlueprintApplyMode::Stamp;
          else return fail(virtualPath, lineNo, "bp_apply: mode expects replace|stamp");
        } else if (key == "fields") {
          std::string err;
          if (!ParseTileFieldMaskList(val, &opt.fieldMask, &err)) {
            return fail(virtualPath, lineNo, "bp_apply: " + err);
          }
        } else if (key == "rotate") {
          int r = 0;
          if (!ParseI32(val, &r)) return fail(virtualPath, lineNo, "bp_apply: rotate expects 0|90|180|270");
          opt.transform.rotateDeg = r;
        } else if (key == "mirrorx") {
          bool b = false;
          if (!ParseBool01(val, &b)) return fail(virtualPath, lineNo, "bp_apply: mirrorx expects 0|1");
          opt.transform.mirrorX = b;
        } else if (key == "mirrory") {
          bool b = false;
          if (!ParseBool01(val, &b)) return fail(virtualPath, lineNo, "bp_apply: mirrory expects 0|1");
          opt.transform.mirrorY = b;
        } else if (key == "allow_oob") {
          bool b = false;
          if (!ParseBool01(val, &b)) return fail(virtualPath, lineNo, "bp_apply: allow_oob expects 0|1");
          opt.allowOutOfBounds = b;
        } else if (key == "force") {
          bool b = false;
          if (!ParseBool01(val, &b)) return fail(virtualPath, lineNo, "bp_apply: force expects 0|1");
          opt.force = b;
        } else if (key == "recompute_roads") {
          bool b = false;
          if (!ParseBool01(val, &b)) return fail(virtualPath, lineNo, "bp_apply: recompute_roads expects 0|1");
          opt.recomputeRoadMasks = b;
        } else {
          return fail(virtualPath, lineNo, "bp_apply: unknown option: " + key);
        }
      }

      std::string err;
      if (!ApplyBlueprint(m_ctx.world, m_ctx.blueprint, dstX, dstY, opt, err)) {
        return fail(virtualPath, lineNo, "bp_apply failed: " + err);
      }

      m_ctx.dirtyDerived = true;
      emitInfo("bp: applied");
      continue;
    }

    if (cmd == "money") {
      if (!EnsureWorld(m_ctx, *this, virtualPath, lineNo)) return false;
      if (t.size() != 2) {
        return fail(virtualPath, lineNo, "money expects: money <N>");
      }
      int v = 0;
      if (!ParseI32(t[1], &v)) {
        return fail(virtualPath, lineNo, "money expects integer");
      }
      m_ctx.world.stats().money = v;
      m_ctx.dirtyDerived = true;
      continue;
    }

    if (cmd == "tick") {
      if (!EnsureWorld(m_ctx, *this, virtualPath, lineNo)) return false;
      if (t.size() != 2) {
        return fail(virtualPath, lineNo, "tick expects: tick <N>");
      }
      int n = 0;
      if (!ParseI32(t[1], &n) || n < 0) {
        return fail(virtualPath, lineNo, "tick expects non-negative integer");
      }

      m_ctx.sim.config() = m_ctx.simCfg;
      for (int i = 0; i < n; ++i) {
        m_ctx.sim.stepOnce(m_ctx.world);
        m_ctx.tickStats.push_back(m_ctx.world.stats());
      }
      m_ctx.dirtyDerived = false;
      continue;
    }

    if (cmd == "autobuild") {
      if (!EnsureWorld(m_ctx, *this, virtualPath, lineNo)) return false;
      if (t.size() != 2) {
        return fail(virtualPath, lineNo, "autobuild expects: autobuild <days>");
      }
      int n = 0;
      if (!ParseI32(t[1], &n) || n < 0) {
        return fail(virtualPath, lineNo, "autobuild expects non-negative integer days");
      }
      m_ctx.sim.config() = m_ctx.simCfg;
      const AutoBuildReport rep = RunAutoBuild(m_ctx.world, m_ctx.sim, m_ctx.autoBuildCfg, n, &m_ctx.tickStats);
      m_ctx.dirtyDerived = false;
      std::ostringstream oss;
      oss << "autobuild: daysSimulated=" << rep.daysSimulated
          << " roadsBuilt=" << rep.roadsBuilt
          << " roadsUpgraded=" << rep.roadsUpgraded
          << " zonesBuilt=" << rep.zonesBuilt
          << " parksBuilt=" << rep.parksBuilt
          << " failedBuilds=" << rep.failedBuilds;
      emitInfo(oss.str());
      continue;
    }

    if (cmd == "stats_clear") {
      m_ctx.tickStats.clear();
      emitInfo("stats: cleared");
      continue;
    }

    if (cmd == "stats_csv") {
      if (t.size() != 2) {
        return fail(virtualPath, lineNo, "stats_csv expects: stats_csv <out.csv>");
      }
      const std::string p = expandPathTemplate(t[1]);
      std::string err;
      if (!WriteStatsCsv(p, m_ctx.tickStats, err)) {
        return fail(virtualPath, lineNo, "stats_csv failed: " + err);
      }
      emitInfo("wrote stats csv -> " + p);
      continue;
    }

    // --- Editing commands ---
    if (cmd == "road") {
      if (!EnsureWorld(m_ctx, *this, virtualPath, lineNo)) return false;
      if (t.size() != 3 && t.size() != 4) {
        return fail(virtualPath, lineNo, "road expects: road x y [level]");
      }
      int x = 0, y = 0;
      if (!ParseI32(t[1], &x) || !ParseI32(t[2], &y)) return false;
      int level = 1;
      if (t.size() == 4 && !ParseI32(t[3], &level)) return false;
      ToolApplyResult r = ToolApplyResult::Noop;
      if (!ApplyRoadTile(m_ctx.world, x, y, level, &r)) {
        return fail(virtualPath, lineNo,
                    "road failed at " + std::to_string(x) + ',' + std::to_string(y) + " (" +
                        ToolApplyResultName(r) + ")");
      }
      m_ctx.dirtyDerived = true;
      continue;
    }

    if (cmd == "road_line") {
      if (!EnsureWorld(m_ctx, *this, virtualPath, lineNo)) return false;
      if (t.size() != 5 && t.size() != 6) {
        return fail(virtualPath, lineNo, "road_line expects: road_line x0 y0 x1 y1 [level]");
      }
      int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
      if (!ParseI32(t[1], &x0) || !ParseI32(t[2], &y0) || !ParseI32(t[3], &x1) || !ParseI32(t[4], &y1)) return false;
      int level = 1;
      if (t.size() == 6 && !ParseI32(t[5], &level)) return false;

      bool ok = true;
      Point failP{0, 0};
      ToolApplyResult failR = ToolApplyResult::Noop;
      ForEachLinePoint(Point{x0, y0}, Point{x1, y1}, [&](Point p) {
        if (!ok) return;
        if (!m_ctx.world.inBounds(p.x, p.y)) {
          ok = false;
          failP = p;
          failR = ToolApplyResult::OutOfBounds;
          return;
        }
        ToolApplyResult r = ToolApplyResult::Noop;
        if (!ApplyRoadTile(m_ctx.world, p.x, p.y, level, &r)) {
          ok = false;
          failP = p;
          failR = r;
        }
      });

      if (!ok) {
        return fail(virtualPath, lineNo,
                    "road_line failed at " + std::to_string(failP.x) + ',' + std::to_string(failP.y) + " (" +
                        ToolApplyResultName(failR) + ")");
      }

      m_ctx.dirtyDerived = true;
      continue;
    }

    if (cmd == "road_path") {
      if (!EnsureWorld(m_ctx, *this, virtualPath, lineNo)) return false;
      if (t.size() < 5) {
        return fail(virtualPath, lineNo,
                    "road_path expects: road_path x0 y0 x1 y1 [level] [allowBridges 0|1] [costModel newtiles|money]");
      }
      int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
      if (!ParseI32(t[1], &x0) || !ParseI32(t[2], &y0) || !ParseI32(t[3], &x1) || !ParseI32(t[4], &y1)) return false;

      int level = 1;
      if (t.size() >= 6 && !ParseI32(t[5], &level)) return false;

      bool allowBridges = false;
      if (t.size() >= 7) {
        if (!ParseBool01(t[6], &allowBridges)) return false;
      }

      RoadBuildPathConfig cfg;
      cfg.targetLevel = level;
      cfg.allowBridges = allowBridges;
      cfg.costModel = RoadBuildPathConfig::CostModel::NewTiles;

      if (t.size() >= 8) {
        const std::string cm = ToLower(t[7]);
        if (cm == "newtiles" || cm == "new_tiles") cfg.costModel = RoadBuildPathConfig::CostModel::NewTiles;
        else if (cm == "money") cfg.costModel = RoadBuildPathConfig::CostModel::Money;
        else {
          return fail(virtualPath, lineNo, "road_path costModel must be newtiles|money");
        }
      }

      std::vector<Point> path;
      int cost = 0;
      if (!FindRoadBuildPath(m_ctx.world, Point{x0, y0}, Point{x1, y1}, path, &cost, cfg)) {
        return fail(virtualPath, lineNo, "road_path failed to find a path");
      }

      bool ok = true;
      Point failP{0, 0};
      ToolApplyResult failR = ToolApplyResult::Noop;

      for (const Point& p : path) {
        ToolApplyResult r = ToolApplyResult::Noop;
        if (!ApplyRoadTile(m_ctx.world, p.x, p.y, level, &r)) {
          ok = false;
          failP = p;
          failR = r;
          break;
        }
      }

      if (!ok) {
        return fail(virtualPath, lineNo,
                    "road_path failed at " + std::to_string(failP.x) + ',' + std::to_string(failP.y) + " (" +
                        ToolApplyResultName(failR) + ")");
      }
      m_ctx.dirtyDerived = true;
      continue;
    }

    if (cmd == "zone") {
      if (!EnsureWorld(m_ctx, *this, virtualPath, lineNo)) return false;
      if (t.size() != 4 && t.size() != 5) {
        return fail(virtualPath, lineNo, "zone expects: zone <res|com|ind> x y [level]");
      }
      const std::string type = ToLower(t[1]);
      int x = 0, y = 0;
      if (!ParseI32(t[2], &x) || !ParseI32(t[3], &y)) return false;
      int level = 1;
      if (t.size() == 5 && !ParseI32(t[4], &level)) return false;

      Tool tool = Tool::Residential;
      if (type == "res" || type == "residential") tool = Tool::Residential;
      else if (type == "com" || type == "commercial") tool = Tool::Commercial;
      else if (type == "ind" || type == "industrial") tool = Tool::Industrial;
      else return fail(virtualPath, lineNo, "zone expects type: res|com|ind");

      ToolApplyResult r = ToolApplyResult::Noop;
      if (!ApplyZoneTile(m_ctx.world, tool, x, y, level, &r)) {
        return fail(virtualPath, lineNo,
                    "zone failed at " + std::to_string(x) + ',' + std::to_string(y) + " (" + ToolApplyResultName(r) +
                        ")");
      }
      m_ctx.dirtyDerived = true;
      continue;
    }

    if (cmd == "park") {
      if (!EnsureWorld(m_ctx, *this, virtualPath, lineNo)) return false;
      if (t.size() != 3) {
        return fail(virtualPath, lineNo, "park expects: park x y");
      }
      int x = 0, y = 0;
      if (!ParseI32(t[1], &x) || !ParseI32(t[2], &y)) return false;
      const ToolApplyResult r = m_ctx.world.applyTool(Tool::Park, x, y);
      if (r != ToolApplyResult::Applied && r != ToolApplyResult::Noop) {
        return fail(virtualPath, lineNo,
                    "park failed at " + std::to_string(x) + ',' + std::to_string(y) + " (" + ToolApplyResultName(r) +
                        ")");
      }
      m_ctx.dirtyDerived = true;
      continue;
    }

    if (cmd == "bulldoze") {
      if (!EnsureWorld(m_ctx, *this, virtualPath, lineNo)) return false;
      if (t.size() != 3) {
        return fail(virtualPath, lineNo, "bulldoze expects: bulldoze x y");
      }
      int x = 0, y = 0;
      if (!ParseI32(t[1], &x) || !ParseI32(t[2], &y)) return false;
      const ToolApplyResult r = m_ctx.world.applyTool(Tool::Bulldoze, x, y);
      if (r != ToolApplyResult::Applied && r != ToolApplyResult::Noop) {
        return fail(virtualPath, lineNo,
                    "bulldoze failed at " + std::to_string(x) + ',' + std::to_string(y) + " (" +
                        ToolApplyResultName(r) + ")");
      }
      m_ctx.dirtyDerived = true;
      continue;
    }

    if (cmd == "district") {
      if (!EnsureWorld(m_ctx, *this, virtualPath, lineNo)) return false;
      if (t.size() != 4) {
        return fail(virtualPath, lineNo, "district expects: district x y <id 0..7>");
      }
      int x = 0, y = 0, id = 0;
      if (!ParseI32(t[1], &x) || !ParseI32(t[2], &y) || !ParseI32(t[3], &id)) return false;
      id = std::clamp(id, 0, kDistrictCount - 1);
      const ToolApplyResult r = m_ctx.world.applyDistrict(x, y, id);
      if (r != ToolApplyResult::Applied && r != ToolApplyResult::Noop) {
        return fail(virtualPath, lineNo,
                    "district failed at " + std::to_string(x) + ',' + std::to_string(y) + " (" +
                        ToolApplyResultName(r) + ")");
      }
      m_ctx.dirtyDerived = true;
      continue;
    }

    if (cmd == "district_auto") {
      if (!EnsureWorld(m_ctx, *this, virtualPath, lineNo)) return false;

      AutoDistrictConfig cfg;

      if (t.size() >= 2 && !ParseI32(t[1], &cfg.districts)) {
        return fail(virtualPath, lineNo, "district_auto: invalid districts");
      }
      cfg.districts = std::clamp(cfg.districts, 1, kDistrictCount);

      if (t.size() >= 3 && !ParseBool01(t[2], &cfg.fillAllTiles)) {
        return fail(virtualPath, lineNo, "district_auto: invalid fillAllTiles (use 0|1)");
      }
      if (t.size() >= 4 && !ParseBool01(t[3], &cfg.useTravelTime)) {
        return fail(virtualPath, lineNo, "district_auto: invalid useTravelTime (use 0|1)");
      }
      if (t.size() >= 5 && !ParseBool01(t[4], &cfg.requireOutsideConnection)) {
        return fail(virtualPath, lineNo, "district_auto: invalid requireOutside (use 0|1)");
      }

      // Derived systems might be required for travel-time weights.
      RefreshIfDirty(m_ctx);

      const AutoDistrictResult r = AutoAssignDistricts(m_ctx.world, cfg);
      emitInfo("district_auto: requested=" + std::to_string(r.districtsRequested) + " used=" +
               std::to_string(r.districtsUsed) + " seeds=" + std::to_string(r.seedRoadIdx.size()));

      m_ctx.dirtyDerived = true;
      continue;
    }

    if (cmd == "flood") {
      if (!EnsureWorld(m_ctx, *this, virtualPath, lineNo)) return false;
      if (t.size() < 4 || t.size() > 6) {
        return fail(virtualPath, lineNo,
                    "flood expects: flood <road|park|bulldoze|district|res|com|ind> x y [arg] [includeRoads 0|1]");
      }

      const std::string what = t[1];
      const std::string whatLower = ToLower(what);

      int x = 0, y = 0;
      if (!ParseI32(t[2], &x) || !ParseI32(t[3], &y)) {
        return fail(virtualPath, lineNo, "flood: invalid coordinates");
      }

      int arg = 0;
      bool includeRoads = false;

      const bool isParkOrBulldoze = (whatLower == "park" || whatLower == "bulldoze");
      const bool isDistrict = (whatLower == "district");

      if (isParkOrBulldoze) {
        // No arg. Optional includeRoads.
        if (t.size() >= 5 && !ParseBool01(t[4], &includeRoads)) {
          return fail(virtualPath, lineNo, "flood: invalid includeRoads (use 0|1)");
        }
        if (t.size() == 6) {
          return fail(virtualPath, lineNo, "flood: too many arguments for " + what);
        }
      } else {
        // Arg is optional for road/zones, required for district.
        if (isDistrict && t.size() < 5) {
          return fail(virtualPath, lineNo, "flood district expects: flood district x y <id> [includeRoads 0|1]");
        }

        if (t.size() >= 5 && !ParseI32(t[4], &arg)) {
          return fail(virtualPath, lineNo, "flood: invalid arg");
        }
        if (t.size() == 6 && !ParseBool01(t[5], &includeRoads)) {
          return fail(virtualPath, lineNo, "flood: invalid includeRoads (use 0|1)");
        }
      }

      Point failP{0, 0};
      ToolApplyResult failR = ToolApplyResult::Noop;
      if (!ApplyFlood(m_ctx.world, what, Point{x, y}, arg, includeRoads, &failP, &failR)) {
        return fail(virtualPath, lineNo,
                    "flood failed at " + std::to_string(failP.x) + ',' + std::to_string(failP.y) + " (" +
                        ToolApplyResultName(failR) + ")");
      }
      m_ctx.dirtyDerived = true;
      continue;
    }

    if (cmd == "fill") {
      if (!EnsureWorld(m_ctx, *this, virtualPath, lineNo)) return false;
      if (t.size() < 6 || t.size() > 7) {
        return fail(virtualPath, lineNo, "fill expects: fill <tool> x0 y0 x1 y1 [arg]");
      }

      const std::string what = t[1];
      int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
      if (!ParseI32(t[2], &x0) || !ParseI32(t[3], &y0) || !ParseI32(t[4], &x1) || !ParseI32(t[5], &y1)) {
        return fail(virtualPath, lineNo, "fill: invalid coordinates");
      }

      int arg = 0;
      if (t.size() == 7 && !ParseI32(t[6], &arg)) {
        return fail(virtualPath, lineNo, "fill: invalid arg");
      }

      const std::string whatLower = ToLower(what);
      const bool recognized =
          (whatLower == "road" || whatLower == "park" || whatLower == "bulldoze" || whatLower == "district" ||
           whatLower == "res" || whatLower == "residential" || whatLower == "com" || whatLower == "commercial" ||
           whatLower == "ind" || whatLower == "industrial");
      if (!recognized) {
        return fail(virtualPath, lineNo, "unknown tool for fill: " + what);
      }

      Point failP{0, 0};
      ToolApplyResult failR = ToolApplyResult::Noop;
      if (!ApplyFill(m_ctx.world, what, Point{x0, y0}, Point{x1, y1}, arg, &failP, &failR)) {
        return fail(virtualPath, lineNo,
                    "fill failed at " + std::to_string(failP.x) + ',' + std::to_string(failP.y) + " (" +
                        ToolApplyResultName(failR) + ")");
      }
      m_ctx.dirtyDerived = true;
      continue;
    }

    if (cmd == "outline") {
      if (!EnsureWorld(m_ctx, *this, virtualPath, lineNo)) return false;
      if (t.size() < 6 || t.size() > 7) {
        return fail(virtualPath, lineNo, "outline expects: outline <tool> x0 y0 x1 y1 [arg]");
      }

      const std::string what = t[1];
      int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
      if (!ParseI32(t[2], &x0) || !ParseI32(t[3], &y0) || !ParseI32(t[4], &x1) || !ParseI32(t[5], &y1)) {
        return fail(virtualPath, lineNo, "outline: invalid coordinates");
      }

      int arg = 0;
      if (t.size() == 7 && !ParseI32(t[6], &arg)) {
        return fail(virtualPath, lineNo, "outline: invalid arg");
      }

      const std::string whatLower = ToLower(what);
      const bool recognized =
          (whatLower == "road" || whatLower == "park" || whatLower == "bulldoze" || whatLower == "district" ||
           whatLower == "res" || whatLower == "residential" || whatLower == "com" || whatLower == "commercial" ||
           whatLower == "ind" || whatLower == "industrial");
      if (!recognized) {
        return fail(virtualPath, lineNo, "unknown tool for outline: " + what);
      }

      Point failP{0, 0};
      ToolApplyResult failR = ToolApplyResult::Noop;
      if (!ApplyOutline(m_ctx.world, what, Point{x0, y0}, Point{x1, y1}, arg, &failP, &failR)) {
        return fail(virtualPath, lineNo,
                    "outline failed at " + std::to_string(failP.x) + ',' + std::to_string(failP.y) + " (" +
                        ToolApplyResultName(failR) + ")");
      }
      m_ctx.dirtyDerived = true;
      continue;
    }

    // --- Artifacts / assertions ---
    if (cmd == "export_ppm") {
      if (!EnsureWorld(m_ctx, *this, virtualPath, lineNo)) return false;
      if (t.size() != 3 && t.size() != 4) {
        return fail(virtualPath, lineNo, "export_ppm expects: export_ppm <layer> <out.ppm> [scale]");
      }

      ExportLayer layer = ExportLayer::Overlay;
      if (!ParseExportLayer(t[1], layer)) {
        return fail(virtualPath, lineNo, "unknown export layer: " + t[1]);
      }

      int scale = 1;
      if (t.size() == 4 && (!ParseI32(t[3], &scale) || scale <= 0)) {
        return fail(virtualPath, lineNo, "scale must be > 0");
      }

      RefreshIfDirty(m_ctx);

      // Compute derived fields on demand.
      std::vector<std::uint8_t> roadToEdge;
      const std::vector<std::uint8_t>* roadToEdgePtr = nullptr;
      if (m_ctx.simCfg.requireOutsideConnection) {
        ComputeRoadsConnectedToEdge(m_ctx.world, roadToEdge);
        roadToEdgePtr = &roadToEdge;
      }

      TrafficResult traffic;
      GoodsResult goods;
      LandValueResult lv;

      if (layer == ExportLayer::Traffic || layer == ExportLayer::LandValue || layer == ExportLayer::GoodsTraffic ||
          layer == ExportLayer::GoodsFill) {
        TrafficConfig tc;
        tc.requireOutsideConnection = m_ctx.simCfg.requireOutsideConnection;
        tc.congestionAwareRouting = m_ctx.sim.trafficModel().congestionAwareRouting;
        tc.congestionIterations = m_ctx.sim.trafficModel().congestionIterations;
        tc.congestionAlpha = m_ctx.sim.trafficModel().congestionAlpha;
        tc.congestionBeta = m_ctx.sim.trafficModel().congestionBeta;
        tc.congestionCapacityScale = m_ctx.sim.trafficModel().congestionCapacityScale;
        tc.congestionRatioClamp = m_ctx.sim.trafficModel().congestionRatioClamp;

        float employedShare = 1.0f;
        const int pop = m_ctx.world.stats().population;
        if (pop > 0) {
          employedShare = static_cast<float>(m_ctx.world.stats().employed) / static_cast<float>(pop);
        }

        traffic = ComputeCommuteTraffic(m_ctx.world, tc, employedShare, roadToEdgePtr);
      }

      if (layer == ExportLayer::GoodsTraffic || layer == ExportLayer::GoodsFill || layer == ExportLayer::LandValue) {
        GoodsConfig gc;
        gc.requireOutsideConnection = m_ctx.simCfg.requireOutsideConnection;
        goods = ComputeGoodsFlow(m_ctx.world, gc, roadToEdgePtr);
      }

      if (layer == ExportLayer::LandValue) {
        LandValueConfig lvc;
        lvc.requireOutsideConnection = m_ctx.simCfg.requireOutsideConnection;
        lv = ComputeLandValue(m_ctx.world, lvc, &traffic, roadToEdgePtr);
      }

      const LandValueResult* lvPtr = (layer == ExportLayer::LandValue) ? &lv : nullptr;
      const TrafficResult* trPtr =
          (layer == ExportLayer::Traffic || layer == ExportLayer::LandValue) ? &traffic : nullptr;
      const GoodsResult* gPtr = (layer == ExportLayer::GoodsTraffic || layer == ExportLayer::GoodsFill ||
                                layer == ExportLayer::LandValue)
                                   ? &goods
                                   : nullptr;

      PpmImage img = RenderPpmLayer(m_ctx.world, layer, lvPtr, trPtr, gPtr);
      if (scale > 1) img = ScaleNearest(img, scale);

      const std::string p = expandPathTemplate(t[2]);
      std::string err;
      if (!WriteImageAuto(p, img, err)) {
        return fail(virtualPath, lineNo, "export_ppm failed: " + err);
      }

      emitInfo(std::string("exported ") + ExportLayerName(layer) + " -> " + p);
      continue;
    }

    if (cmd == "export_tiles_csv") {
      if (!EnsureWorld(m_ctx, *this, virtualPath, lineNo)) return false;
      if (t.size() != 2) {
        return fail(virtualPath, lineNo, "export_tiles_csv expects: export_tiles_csv <out.csv>");
      }
      const std::string p = expandPathTemplate(t[1]);
      std::string err;
      if (!WriteTilesCsv(m_ctx.world, p, err)) {
        return fail(virtualPath, lineNo, "export_tiles_csv failed: " + err);
      }
      emitInfo("exported tiles csv -> " + p);
      continue;
    }

    if (cmd == "districts_json") {
      if (!EnsureWorld(m_ctx, *this, virtualPath, lineNo)) return false;
      if (t.size() != 2) {
        return fail(virtualPath, lineNo, "districts_json expects: districts_json <out.json>");
      }
      RefreshIfDirty(m_ctx);
      const std::string p = expandPathTemplate(t[1]);
      if (!WriteDistrictsJsonFile(p, m_ctx.world, m_ctx.simCfg)) {
        return fail(virtualPath, lineNo, "districts_json failed");
      }
      emitInfo("exported districts json -> " + p);
      continue;
    }

    if (cmd == "hash") {
      if (!EnsureWorld(m_ctx, *this, virtualPath, lineNo)) return false;
      RefreshIfDirty(m_ctx);
      const std::uint64_t h = HashWorld(m_ctx.world, true);
      emitPrint(HexU64(h));
      continue;
    }

    if (cmd == "expect_hash") {
      if (!EnsureWorld(m_ctx, *this, virtualPath, lineNo)) return false;
      if (t.size() != 2) {
        return fail(virtualPath, lineNo, "expect_hash expects: expect_hash <u64|0x...>");
      }
      std::uint64_t want = 0;
      if (!ParseU64(t[1], &want)) {
        return fail(virtualPath, lineNo, "invalid hash integer");
      }
      RefreshIfDirty(m_ctx);
      const std::uint64_t got = HashWorld(m_ctx.world, true);
      if (got != want) {
        emitError("expect_hash FAILED");
        emitError("  want: " + HexU64(want));
        emitError("  got:  " + HexU64(got));
        return false;
      }
      continue;
    }

    return fail(virtualPath, lineNo, "unknown command: " + t[0]);
  }

  return true;
}

} // namespace isocity
