#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace isocity {

struct MineRecord;

// -----------------------------------------------------------------------------
// MineExpr: a tiny expression language compiled to a small stack VM.
//
// Supports:
//  - numeric literals (floating point)
//  - variables (case-insensitive; see MineExprHelpText())
//  - unary operators: + - !
//  - binary operators: + - * / ^
//  - comparisons: < <= > >= == !=   (return 1.0 for true, 0.0 for false)
//  - boolean ops: && ||            (0.0=false, nonzero=true)
//  - functions: min,max,clamp,abs,sqrt,log,exp,floor,ceil,round,lerp,step,smoothstep
//
// The VM is deterministic and allocation-free during evaluation (aside from a
// small stack vector). Compilation is intended to happen once per mining run.
// -----------------------------------------------------------------------------

enum class MineExprOp : std::uint8_t {
  PushConst = 0,
  PushVar = 1,

  Add,
  Sub,
  Mul,
  Div,
  Pow,

  Neg,
  Not,

  Less,
  LessEq,
  Greater,
  GreaterEq,
  Eq,
  NotEq,

  And,
  Or,

  Min,
  Max,
  Clamp,

  Abs,
  Sqrt,
  Log,
  Exp,

  Floor,
  Ceil,
  Round,

  Lerp,
  Step,
  Smoothstep,
};

struct MineExprInstr {
  MineExprOp op = MineExprOp::PushConst;
  double c = 0.0;       // used by PushConst
  std::uint16_t id = 0; // used by PushVar
};

struct MineExprProgram {
  // Original expression text.
  std::string expr;

  // Post-order / RPN instruction stream.
  std::vector<MineExprInstr> code;
};

// Compile an expression into a program.
// Returns false on parse errors; outError is a human-readable message.
bool CompileMineExpr(const std::string& expr, MineExprProgram& outProg, std::string& outError);

// Evaluate a compiled program against a MineRecord.
// Returns false on runtime errors (stack underflow, etc.).
// Note: The computed value may be non-finite if the expression produces it.
bool EvalMineExpr(const MineExprProgram& prog,
                  const MineRecord& r,
                  double& outValue,
                  std::string* outError = nullptr);

// Returns a short help text listing supported variables and functions.
std::string MineExprHelpText();

} // namespace isocity
