#include "isocity/MineExpr.hpp"

#include "isocity/SeedMiner.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <sstream>
#include <string_view>

namespace isocity {
namespace {

static std::string NormalizeKey(std::string_view s)
{
  std::string t;
  t.reserve(s.size());
  for (char c : s) {
    const unsigned char uc = static_cast<unsigned char>(c);
    char out = static_cast<char>(std::tolower(uc));
    if (out == '-' || out == ' ' || out == '.') out = '_';
    t.push_back(out);
  }
  return t;
}

static std::string FormatError(std::string_view src, std::size_t pos, std::string_view msg)
{
  std::ostringstream os;
  os << "mine expr parse error at " << pos << ": " << msg;

  if (!src.empty()) {
    const std::size_t a = (pos > 32) ? (pos - 32) : 0;
    const std::size_t b = std::min(src.size(), pos + 32);
    os << "\n  " << src.substr(a, b - a);
    os << "\n  " << std::string((pos >= a) ? (pos - a) : 0, ' ') << '^';
  }

  return os.str();
}

// -----------------------------------------------------------------------------
// Lexer
// -----------------------------------------------------------------------------

enum class TokType : std::uint8_t {
  End = 0,
  Number,
  Ident,

  Plus,
  Minus,
  Star,
  Slash,
  Caret,

  LParen,
  RParen,
  Comma,

  Less,
  Greater,
  LessEq,
  GreaterEq,
  EqEq,
  NotEq,

  AndAnd,
  OrOr,
  Bang,
};

struct Token {
  TokType type = TokType::End;
  double number = 0.0;
  std::string_view text;
  std::size_t pos = 0;
};

class Lexer {
public:
  explicit Lexer(std::string_view src) : m_src(src)
  {
    next();
  }

  const Token& tok() const { return m_tok; }

  void next()
  {
    // Skip whitespace.
    while (m_pos < m_src.size()) {
      const unsigned char uc = static_cast<unsigned char>(m_src[m_pos]);
      if (!std::isspace(uc)) break;
      ++m_pos;
    }

    m_tok = Token{};
    m_tok.pos = m_pos;

    if (m_pos >= m_src.size()) {
      m_tok.type = TokType::End;
      m_tok.text = std::string_view();
      return;
    }

    const char c = m_src[m_pos];

    // Number.
    if ((c >= '0' && c <= '9') || c == '.') {
      const char* begin = m_src.data() + m_pos;
      char* end = nullptr;
      errno = 0;
      const double v = std::strtod(begin, &end);
      if (errno == 0 && end && end > begin) {
        const std::size_t len = static_cast<std::size_t>(end - begin);
        m_tok.type = TokType::Number;
        m_tok.number = v;
        m_tok.text = m_src.substr(m_pos, len);
        m_pos += len;
        return;
      }
    }

    // Identifier.
    {
      const unsigned char uc = static_cast<unsigned char>(c);
      if (std::isalpha(uc) || c == '_') {
        std::size_t start = m_pos;
        ++m_pos;
        while (m_pos < m_src.size()) {
          const unsigned char uc2 = static_cast<unsigned char>(m_src[m_pos]);
          if (std::isalnum(uc2) || m_src[m_pos] == '_') {
            ++m_pos;
            continue;
          }
          break;
        }
        m_tok.type = TokType::Ident;
        m_tok.text = m_src.substr(start, m_pos - start);
        return;
      }
    }

    // Two-character operators.
    if (m_pos + 1 < m_src.size()) {
      const char n = m_src[m_pos + 1];
      if (c == '<' && n == '=') {
        m_tok.type = TokType::LessEq;
        m_tok.text = m_src.substr(m_pos, 2);
        m_pos += 2;
        return;
      }
      if (c == '>' && n == '=') {
        m_tok.type = TokType::GreaterEq;
        m_tok.text = m_src.substr(m_pos, 2);
        m_pos += 2;
        return;
      }
      if (c == '=' && n == '=') {
        m_tok.type = TokType::EqEq;
        m_tok.text = m_src.substr(m_pos, 2);
        m_pos += 2;
        return;
      }
      if (c == '!' && n == '=') {
        m_tok.type = TokType::NotEq;
        m_tok.text = m_src.substr(m_pos, 2);
        m_pos += 2;
        return;
      }
      if (c == '&' && n == '&') {
        m_tok.type = TokType::AndAnd;
        m_tok.text = m_src.substr(m_pos, 2);
        m_pos += 2;
        return;
      }
      if (c == '|' && n == '|') {
        m_tok.type = TokType::OrOr;
        m_tok.text = m_src.substr(m_pos, 2);
        m_pos += 2;
        return;
      }
    }

    // Single-character tokens.
    switch (c) {
    case '+': m_tok.type = TokType::Plus; break;
    case '-': m_tok.type = TokType::Minus; break;
    case '*': m_tok.type = TokType::Star; break;
    case '/': m_tok.type = TokType::Slash; break;
    case '^': m_tok.type = TokType::Caret; break;
    case '(': m_tok.type = TokType::LParen; break;
    case ')': m_tok.type = TokType::RParen; break;
    case ',': m_tok.type = TokType::Comma; break;
    case '<': m_tok.type = TokType::Less; break;
    case '>': m_tok.type = TokType::Greater; break;
    case '!': m_tok.type = TokType::Bang; break;
    default:
      // Unknown character: leave as End to force a parse error.
      m_tok.type = TokType::End;
      break;
    }

    m_tok.text = m_src.substr(m_pos, 1);
    ++m_pos;
  }

private:
  std::string_view m_src;
  std::size_t m_pos = 0;
  Token m_tok;
};

// -----------------------------------------------------------------------------
// Variables
// -----------------------------------------------------------------------------

enum class Var : std::uint16_t {
  Seed = 0,
  W,
  H,
  Area,
  Day,

  PopDensity,
  RoadDensity,
  ZoneDensity,

  Population,
  Happiness,
  Money,
  AvgLandValue,
  TrafficCongestion,
  GoodsSatisfaction,
  ServicesOverallSatisfaction,

  WaterTiles,
  RoadTiles,
  ResTiles,
  ComTiles,
  IndTiles,
  ParkTiles,

  WaterFrac,
  RoadFrac,
  ZoneFrac,
  ParkFrac,

  SeaFloodFrac,
  SeaMaxDepth,
  PondFrac,
  PondMaxDepth,
  PondVolume,
  FloodRisk,

  Score,
  ObjectiveScore,

  Pi,
  E,
};

static bool LookupVarId(std::string_view ident, std::uint16_t& outId)
{
  const std::string k = NormalizeKey(ident);

  auto set = [&](Var v) {
    outId = static_cast<std::uint16_t>(v);
    return true;
  };

  if (k == "seed") return set(Var::Seed);
  if (k == "w" || k == "width") return set(Var::W);
  if (k == "h" || k == "height") return set(Var::H);
  if (k == "area") return set(Var::Area);
  if (k == "day") return set(Var::Day);

  if (k == "pop_density" || k == "population_density") return set(Var::PopDensity);
  if (k == "road_density") return set(Var::RoadDensity);
  if (k == "zone_density") return set(Var::ZoneDensity);

  if (k == "population" || k == "pop") return set(Var::Population);
  if (k == "happiness") return set(Var::Happiness);
  if (k == "money") return set(Var::Money);
  if (k == "avg_land_value" || k == "avglandvalue" || k == "land_value") return set(Var::AvgLandValue);
  if (k == "traffic_congestion" || k == "traffic" || k == "congestion") return set(Var::TrafficCongestion);
  if (k == "goods_satisfaction" || k == "goods") return set(Var::GoodsSatisfaction);
  if (k == "services_overall_satisfaction" || k == "services_satisfaction" || k == "services") return set(Var::ServicesOverallSatisfaction);

  if (k == "water_tiles") return set(Var::WaterTiles);
  if (k == "road_tiles") return set(Var::RoadTiles);
  if (k == "res_tiles" || k == "residential_tiles") return set(Var::ResTiles);
  if (k == "com_tiles" || k == "commercial_tiles") return set(Var::ComTiles);
  if (k == "ind_tiles" || k == "industrial_tiles") return set(Var::IndTiles);
  if (k == "park_tiles") return set(Var::ParkTiles);

  if (k == "water_frac" || k == "water_fraction") return set(Var::WaterFrac);
  if (k == "road_frac" || k == "road_fraction") return set(Var::RoadFrac);
  if (k == "zone_frac" || k == "zone_fraction") return set(Var::ZoneFrac);
  if (k == "park_frac" || k == "park_fraction") return set(Var::ParkFrac);

  if (k == "sea_flood_frac") return set(Var::SeaFloodFrac);
  if (k == "sea_max_depth") return set(Var::SeaMaxDepth);
  if (k == "pond_frac") return set(Var::PondFrac);
  if (k == "pond_max_depth") return set(Var::PondMaxDepth);
  if (k == "pond_volume") return set(Var::PondVolume);
  if (k == "flood_risk") return set(Var::FloodRisk);

  if (k == "score") return set(Var::Score);
  if (k == "objective_score" || k == "objective") return set(Var::ObjectiveScore);

  if (k == "pi") return set(Var::Pi);
  if (k == "e") return set(Var::E);

  return false;
}

static double GetVarValue(std::uint16_t id, const MineRecord& r)
{
  const Var v = static_cast<Var>(id);
  const double area = std::max(1.0, static_cast<double>(r.w) * static_cast<double>(r.h));

  switch (v) {
  case Var::Seed: return static_cast<double>(r.seed);
  case Var::W: return static_cast<double>(r.w);
  case Var::H: return static_cast<double>(r.h);
  case Var::Area: return static_cast<double>(area);
  case Var::Day: return static_cast<double>(r.stats.day);

  case Var::PopDensity: return static_cast<double>(r.stats.population) / area;
  case Var::RoadDensity: return static_cast<double>(r.roadTiles) / area;
  case Var::ZoneDensity: return r.zoneFrac;

  case Var::Population: return static_cast<double>(r.stats.population);
  case Var::Happiness: return static_cast<double>(r.stats.happiness);
  case Var::Money: return static_cast<double>(r.stats.money);
  case Var::AvgLandValue: return static_cast<double>(r.stats.avgLandValue);
  case Var::TrafficCongestion: return static_cast<double>(r.stats.trafficCongestion);
  case Var::GoodsSatisfaction: return static_cast<double>(r.stats.goodsSatisfaction);
  case Var::ServicesOverallSatisfaction: return static_cast<double>(r.stats.servicesOverallSatisfaction);

  case Var::WaterTiles: return static_cast<double>(r.waterTiles);
  case Var::RoadTiles: return static_cast<double>(r.roadTiles);
  case Var::ResTiles: return static_cast<double>(r.resTiles);
  case Var::ComTiles: return static_cast<double>(r.comTiles);
  case Var::IndTiles: return static_cast<double>(r.indTiles);
  case Var::ParkTiles: return static_cast<double>(r.parkTiles);

  case Var::WaterFrac: return r.waterFrac;
  case Var::RoadFrac: return r.roadFrac;
  case Var::ZoneFrac: return r.zoneFrac;
  case Var::ParkFrac: return r.parkFrac;

  case Var::SeaFloodFrac: return r.seaFloodFrac;
  case Var::SeaMaxDepth: return r.seaMaxDepth;
  case Var::PondFrac: return r.pondFrac;
  case Var::PondMaxDepth: return r.pondMaxDepth;
  case Var::PondVolume: return r.pondVolume;
  case Var::FloodRisk: return MineMetricValue(r, MineMetric::FloodRisk);

  case Var::Score: return r.score;
  case Var::ObjectiveScore: return r.objectiveScore;

  case Var::Pi: return 3.141592653589793238462643383279502884;
  case Var::E: return 2.718281828459045235360287471352662498;
  }
  return 0.0;
}

// -----------------------------------------------------------------------------
// Parser (recursive descent) that emits RPN instructions.
// -----------------------------------------------------------------------------

class Parser {
public:
  Parser(std::string_view src, MineExprProgram& prog, std::string& outErr)
      : m_src(src), m_lex(src), m_prog(prog), m_err(outErr)
  {
    m_err.clear();
  }

  bool parse()
  {
    if (!parseExpression()) return false;
    if (m_lex.tok().type != TokType::End) {
      return failHere("unexpected token");
    }
    if (m_prog.code.empty()) {
      return failAt(0, "empty expression");
    }
    return true;
  }

private:
  std::string_view m_src;
  Lexer m_lex;
  MineExprProgram& m_prog;
  std::string& m_err;

  bool failAt(std::size_t pos, std::string_view msg)
  {
    if (m_err.empty()) m_err = FormatError(m_src, pos, msg);
    return false;
  }

  bool failHere(std::string_view msg)
  {
    return failAt(m_lex.tok().pos, msg);
  }

  bool accept(TokType t)
  {
    if (m_lex.tok().type == t) {
      m_lex.next();
      return true;
    }
    return false;
  }

  bool expect(TokType t, std::string_view what)
  {
    if (m_lex.tok().type != t) {
      std::string msg = "expected ";
      msg += what;
      return failHere(msg);
    }
    m_lex.next();
    return true;
  }

  void emit(MineExprOp op)
  {
    MineExprInstr ins;
    ins.op = op;
    m_prog.code.push_back(ins);
  }

  void emitConst(double v)
  {
    MineExprInstr ins;
    ins.op = MineExprOp::PushConst;
    ins.c = v;
    m_prog.code.push_back(ins);
  }

  bool emitVar(std::string_view ident)
  {
    std::uint16_t id = 0;
    if (!LookupVarId(ident, id)) {
      std::string msg = "unknown variable: ";
      msg.append(ident.begin(), ident.end());
      return failAt(m_lex.tok().pos, msg);
    }
    MineExprInstr ins;
    ins.op = MineExprOp::PushVar;
    ins.id = id;
    m_prog.code.push_back(ins);
    return true;
  }

  // Grammar (precedence from low to high):
  //   expr   := or
  //   or     := and ( '||' and )*
  //   and    := eq  ( '&&' eq  )*
  //   eq     := rel ( ('==' | '!=') rel )*
  //   rel    := add ( (< | <= | > | >=) add )*
  //   add    := mul ( (+|-) mul )*
  //   mul    := pow ( (*|/) pow )*
  //   pow    := unary ( '^' pow )?
  //   unary  := (+|-|!) unary | primary
  //   primary:= number | ident | ident '(' args ')' | '(' expr ')'

  bool parseExpression() { return parseOr(); }

  bool parseOr()
  {
    if (!parseAnd()) return false;
    while (m_lex.tok().type == TokType::OrOr) {
      m_lex.next();
      if (!parseAnd()) return false;
      emit(MineExprOp::Or);
    }
    return true;
  }

  bool parseAnd()
  {
    if (!parseEquality()) return false;
    while (m_lex.tok().type == TokType::AndAnd) {
      m_lex.next();
      if (!parseEquality()) return false;
      emit(MineExprOp::And);
    }
    return true;
  }

  bool parseEquality()
  {
    if (!parseRelational()) return false;
    while (m_lex.tok().type == TokType::EqEq || m_lex.tok().type == TokType::NotEq) {
      const TokType t = m_lex.tok().type;
      m_lex.next();
      if (!parseRelational()) return false;
      emit(t == TokType::EqEq ? MineExprOp::Eq : MineExprOp::NotEq);
    }
    return true;
  }

  bool parseRelational()
  {
    if (!parseAdd()) return false;
    for (;;) {
      const TokType t = m_lex.tok().type;
      MineExprOp op;
      if (t == TokType::Less) op = MineExprOp::Less;
      else if (t == TokType::LessEq) op = MineExprOp::LessEq;
      else if (t == TokType::Greater) op = MineExprOp::Greater;
      else if (t == TokType::GreaterEq) op = MineExprOp::GreaterEq;
      else break;

      m_lex.next();
      if (!parseAdd()) return false;
      emit(op);
    }
    return true;
  }

  bool parseAdd()
  {
    if (!parseMul()) return false;
    while (m_lex.tok().type == TokType::Plus || m_lex.tok().type == TokType::Minus) {
      const TokType t = m_lex.tok().type;
      m_lex.next();
      if (!parseMul()) return false;
      emit(t == TokType::Plus ? MineExprOp::Add : MineExprOp::Sub);
    }
    return true;
  }

  bool parseMul()
  {
    if (!parsePow()) return false;
    while (m_lex.tok().type == TokType::Star || m_lex.tok().type == TokType::Slash) {
      const TokType t = m_lex.tok().type;
      m_lex.next();
      if (!parsePow()) return false;
      emit(t == TokType::Star ? MineExprOp::Mul : MineExprOp::Div);
    }
    return true;
  }

  bool parsePow()
  {
    if (!parseUnary()) return false;
    if (m_lex.tok().type == TokType::Caret) {
      m_lex.next();
      if (!parsePow()) return false; // right-associative
      emit(MineExprOp::Pow);
    }
    return true;
  }

  bool parseUnary()
  {
    if (m_lex.tok().type == TokType::Plus) {
      m_lex.next();
      return parseUnary();
    }
    if (m_lex.tok().type == TokType::Minus) {
      m_lex.next();
      if (!parseUnary()) return false;
      emit(MineExprOp::Neg);
      return true;
    }
    if (m_lex.tok().type == TokType::Bang) {
      m_lex.next();
      if (!parseUnary()) return false;
      emit(MineExprOp::Not);
      return true;
    }
    return parsePrimary();
  }

  bool parsePrimary()
  {
    const Token t = m_lex.tok();

    if (t.type == TokType::Number) {
      emitConst(t.number);
      m_lex.next();
      return true;
    }

    if (t.type == TokType::Ident) {
      const std::string_view ident = t.text;
      m_lex.next();

      // Function call?
      if (accept(TokType::LParen)) {
        std::vector<int> argMarkers; // unused; just counting
        int argc = 0;

        if (!accept(TokType::RParen)) {
          for (;;) {
            if (!parseExpression()) return false;
            ++argc;
            if (accept(TokType::Comma)) continue;
            if (!expect(TokType::RParen, "')'")) return false;
            break;
          }
        }

        const std::string fn = NormalizeKey(ident);

        auto expectArgc = [&](int want) -> bool {
          if (argc != want) {
            std::ostringstream os;
            os << "function '" << std::string(ident) << "' expects " << want << " args";
            return failAt(t.pos, os.str());
          }
          return true;
        };

        // 1-arg funcs.
        if (fn == "abs") {
          if (!expectArgc(1)) return false;
          emit(MineExprOp::Abs);
          return true;
        }
        if (fn == "sqrt") {
          if (!expectArgc(1)) return false;
          emit(MineExprOp::Sqrt);
          return true;
        }
        if (fn == "log") {
          if (!expectArgc(1)) return false;
          emit(MineExprOp::Log);
          return true;
        }
        if (fn == "exp") {
          if (!expectArgc(1)) return false;
          emit(MineExprOp::Exp);
          return true;
        }
        if (fn == "floor") {
          if (!expectArgc(1)) return false;
          emit(MineExprOp::Floor);
          return true;
        }
        if (fn == "ceil" || fn == "ceiling") {
          if (!expectArgc(1)) return false;
          emit(MineExprOp::Ceil);
          return true;
        }
        if (fn == "round") {
          if (!expectArgc(1)) return false;
          emit(MineExprOp::Round);
          return true;
        }

        // 2-arg funcs.
        if (fn == "min") {
          if (!expectArgc(2)) return false;
          emit(MineExprOp::Min);
          return true;
        }
        if (fn == "max") {
          if (!expectArgc(2)) return false;
          emit(MineExprOp::Max);
          return true;
        }
        if (fn == "pow") {
          if (!expectArgc(2)) return false;
          emit(MineExprOp::Pow);
          return true;
        }
        if (fn == "step") {
          if (!expectArgc(2)) return false;
          emit(MineExprOp::Step);
          return true;
        }

        // 3-arg funcs.
        if (fn == "clamp") {
          if (!expectArgc(3)) return false;
          emit(MineExprOp::Clamp);
          return true;
        }
        if (fn == "lerp") {
          if (!expectArgc(3)) return false;
          emit(MineExprOp::Lerp);
          return true;
        }
        if (fn == "smoothstep") {
          if (!expectArgc(3)) return false;
          emit(MineExprOp::Smoothstep);
          return true;
        }

        std::string msg = "unknown function: ";
        msg.append(ident.begin(), ident.end());
        return failAt(t.pos, msg);
      }

      // Variable.
      return emitVar(ident);
    }

    if (accept(TokType::LParen)) {
      if (!parseExpression()) return false;
      if (!expect(TokType::RParen, "')'")) return false;
      return true;
    }

    return failHere("expected number, variable, or '(' expression ')'");
  }
};

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

static double Smoothstep(double e0, double e1, double x)
{
  if (e0 == e1) return (x < e0) ? 0.0 : 1.0;
  const double t0 = (x - e0) / (e1 - e0);
  const double t = std::clamp(t0, 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

} // namespace

bool CompileMineExpr(const std::string& expr, MineExprProgram& outProg, std::string& outError)
{
  outProg = MineExprProgram{};
  outProg.expr = expr;

  Parser p(expr, outProg, outError);
  if (!p.parse()) {
    outProg.code.clear();
    return false;
  }

  outError.clear();
  return true;
}

bool EvalMineExpr(const MineExprProgram& prog, const MineRecord& r, double& outValue, std::string* outError)
{
  auto fail = [&](std::string msg) {
    if (outError) *outError = std::move(msg);
    return false;
  };

  if (prog.code.empty()) {
    return fail("empty program");
  }

  std::vector<double> stack;
  stack.reserve(prog.code.size());

  auto pop1 = [&](double& a) -> bool {
    if (stack.empty()) return false;
    a = stack.back();
    stack.pop_back();
    return true;
  };

  auto pop2 = [&](double& a, double& b) -> bool {
    if (stack.size() < 2) return false;
    b = stack.back();
    stack.pop_back();
    a = stack.back();
    stack.pop_back();
    return true;
  };

  auto pop3 = [&](double& a, double& b, double& c) -> bool {
    if (stack.size() < 3) return false;
    c = stack.back();
    stack.pop_back();
    b = stack.back();
    stack.pop_back();
    a = stack.back();
    stack.pop_back();
    return true;
  };

  for (const MineExprInstr& ins : prog.code) {
    switch (ins.op) {
    case MineExprOp::PushConst:
      stack.push_back(ins.c);
      break;

    case MineExprOp::PushVar:
      stack.push_back(GetVarValue(ins.id, r));
      break;

    case MineExprOp::Add: {
      double a = 0, b = 0;
      if (!pop2(a, b)) return fail("stack underflow for +");
      stack.push_back(a + b);
      break;
    }
    case MineExprOp::Sub: {
      double a = 0, b = 0;
      if (!pop2(a, b)) return fail("stack underflow for -");
      stack.push_back(a - b);
      break;
    }
    case MineExprOp::Mul: {
      double a = 0, b = 0;
      if (!pop2(a, b)) return fail("stack underflow for *");
      stack.push_back(a * b);
      break;
    }
    case MineExprOp::Div: {
      double a = 0, b = 0;
      if (!pop2(a, b)) return fail("stack underflow for /");
      stack.push_back(a / b);
      break;
    }
    case MineExprOp::Pow: {
      double a = 0, b = 0;
      if (!pop2(a, b)) return fail("stack underflow for ^");
      stack.push_back(std::pow(a, b));
      break;
    }

    case MineExprOp::Neg: {
      double a = 0;
      if (!pop1(a)) return fail("stack underflow for unary -");
      stack.push_back(-a);
      break;
    }

    case MineExprOp::Not: {
      double a = 0;
      if (!pop1(a)) return fail("stack underflow for !");
      stack.push_back((a == 0.0) ? 1.0 : 0.0);
      break;
    }

    case MineExprOp::Less: {
      double a = 0, b = 0;
      if (!pop2(a, b)) return fail("stack underflow for <");
      stack.push_back((a < b) ? 1.0 : 0.0);
      break;
    }
    case MineExprOp::LessEq: {
      double a = 0, b = 0;
      if (!pop2(a, b)) return fail("stack underflow for <=");
      stack.push_back((a <= b) ? 1.0 : 0.0);
      break;
    }
    case MineExprOp::Greater: {
      double a = 0, b = 0;
      if (!pop2(a, b)) return fail("stack underflow for >");
      stack.push_back((a > b) ? 1.0 : 0.0);
      break;
    }
    case MineExprOp::GreaterEq: {
      double a = 0, b = 0;
      if (!pop2(a, b)) return fail("stack underflow for >=");
      stack.push_back((a >= b) ? 1.0 : 0.0);
      break;
    }
    case MineExprOp::Eq: {
      double a = 0, b = 0;
      if (!pop2(a, b)) return fail("stack underflow for ==");
      stack.push_back((a == b) ? 1.0 : 0.0);
      break;
    }
    case MineExprOp::NotEq: {
      double a = 0, b = 0;
      if (!pop2(a, b)) return fail("stack underflow for !=");
      stack.push_back((a != b) ? 1.0 : 0.0);
      break;
    }

    case MineExprOp::And: {
      double a = 0, b = 0;
      if (!pop2(a, b)) return fail("stack underflow for &&");
      stack.push_back((a != 0.0 && b != 0.0) ? 1.0 : 0.0);
      break;
    }

    case MineExprOp::Or: {
      double a = 0, b = 0;
      if (!pop2(a, b)) return fail("stack underflow for ||");
      stack.push_back((a != 0.0 || b != 0.0) ? 1.0 : 0.0);
      break;
    }

    case MineExprOp::Min: {
      double a = 0, b = 0;
      if (!pop2(a, b)) return fail("stack underflow for min");
      stack.push_back(std::min(a, b));
      break;
    }

    case MineExprOp::Max: {
      double a = 0, b = 0;
      if (!pop2(a, b)) return fail("stack underflow for max");
      stack.push_back(std::max(a, b));
      break;
    }

    case MineExprOp::Clamp: {
      double x = 0, lo = 0, hi = 0;
      if (!pop3(x, lo, hi)) return fail("stack underflow for clamp");
      stack.push_back(std::clamp(x, lo, hi));
      break;
    }

    case MineExprOp::Abs: {
      double a = 0;
      if (!pop1(a)) return fail("stack underflow for abs");
      stack.push_back(std::abs(a));
      break;
    }
    case MineExprOp::Sqrt: {
      double a = 0;
      if (!pop1(a)) return fail("stack underflow for sqrt");
      stack.push_back(std::sqrt(a));
      break;
    }
    case MineExprOp::Log: {
      double a = 0;
      if (!pop1(a)) return fail("stack underflow for log");
      stack.push_back(std::log(a));
      break;
    }
    case MineExprOp::Exp: {
      double a = 0;
      if (!pop1(a)) return fail("stack underflow for exp");
      stack.push_back(std::exp(a));
      break;
    }

    case MineExprOp::Floor: {
      double a = 0;
      if (!pop1(a)) return fail("stack underflow for floor");
      stack.push_back(std::floor(a));
      break;
    }
    case MineExprOp::Ceil: {
      double a = 0;
      if (!pop1(a)) return fail("stack underflow for ceil");
      stack.push_back(std::ceil(a));
      break;
    }
    case MineExprOp::Round: {
      double a = 0;
      if (!pop1(a)) return fail("stack underflow for round");
      stack.push_back(std::round(a));
      break;
    }

    case MineExprOp::Lerp: {
      double a = 0, b = 0, t = 0;
      if (!pop3(a, b, t)) return fail("stack underflow for lerp");
      stack.push_back(a + (b - a) * t);
      break;
    }

    case MineExprOp::Step: {
      double edge = 0, x = 0;
      if (!pop2(edge, x)) return fail("stack underflow for step");
      stack.push_back((x >= edge) ? 1.0 : 0.0);
      break;
    }

    case MineExprOp::Smoothstep: {
      double e0 = 0, e1 = 0, x = 0;
      if (!pop3(e0, e1, x)) return fail("stack underflow for smoothstep");
      stack.push_back(Smoothstep(e0, e1, x));
      break;
    }
    }
  }

  if (stack.size() != 1) {
    return fail("expression did not reduce to a single value");
  }

  outValue = stack.back();
  if (outError) outError->clear();
  return true;
}

std::string MineExprHelpText()
{
  std::ostringstream os;
  os << "MineExpr variables (case-insensitive):\n";
  os << "  seed,w,h,area,day,pop_density,road_density,zone_density\n";
  os << "  population,happiness,money,avg_land_value,traffic_congestion,goods_satisfaction,services_overall_satisfaction\n";
  os << "  water_tiles,road_tiles,res_tiles,com_tiles,ind_tiles,park_tiles\n";
  os << "  water_frac,road_frac,zone_frac,park_frac\n";
  os << "  sea_flood_frac,sea_max_depth,pond_frac,pond_max_depth,pond_volume,flood_risk\n";
  os << "  score,objective_score\n";
  os << "  pi,e\n\n";

  os << "Functions:\n";
  os << "  min(a,b), max(a,b), clamp(x,lo,hi)\n";
  os << "  abs(x), sqrt(x), log(x), exp(x)\n";
  os << "  floor(x), ceil(x), round(x)\n";
  os << "  lerp(a,b,t), step(edge,x), smoothstep(edge0,edge1,x)\n\n";

  os << "Operators:\n";
  os << "  unary: + - !\n";
  os << "  binary: + - * / ^  comparisons: < <= > >= == !=  boolean: && ||\n";

  return os.str();
}

} // namespace isocity
