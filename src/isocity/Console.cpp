#include "isocity/Console.hpp"

#include "raylib.h"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <exception>
#include <sstream>

namespace isocity {

namespace {
constexpr int kFontSize = 18;
constexpr int kLineGap = 4;
constexpr int kPadding = 10;

std::string EllipsizeToWidth(const std::string& s, int maxWidth, int fontSize)
{
  if (maxWidth <= 0) return std::string();
  if (MeasureText(s.c_str(), fontSize) <= maxWidth) return s;

  std::string out = s;
  // Trim until the string fits, leaving room for "...".
  while (!out.empty() && MeasureText((out + "...").c_str(), fontSize) > maxWidth) {
    out.pop_back();
  }
  return out + "...";
}

} // namespace

// Split a single input line into multiple commands separated by ';'.
// Semicolons inside single/double quotes are ignored.
// Quotes and simple backslash escapes are preserved so the tokenizer can handle them.
static std::vector<std::string> SplitCommands(const std::string& line)
{
  enum class Q : unsigned char { None = 0, Single, Double };
  Q q = Q::None;

  std::vector<std::string> out;
  std::string cur;
  cur.reserve(line.size());

  auto push = [&]() {
    out.push_back(cur);
    cur.clear();
  };

  for (std::size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];

    if (q == Q::None) {
      if (c == ';') {
        push();
        continue;
      }
      if (c == '"') {
        q = Q::Double;
        cur.push_back(c);
        continue;
      }
      if (c == '\'') {
        q = Q::Single;
        cur.push_back(c);
        continue;
      }
      cur.push_back(c);
      continue;
    }

    if (q == Q::Double) {
      // Preserve escapes so the tokenizer can interpret them.
      if (c == '\\' && (i + 1) < line.size()) {
        const char n = line[i + 1];
        if (n == '"' || n == '\\') {
          cur.push_back(c);
          cur.push_back(n);
          ++i;
          continue;
        }
      }
      if (c == '"') {
        q = Q::None;
      }
      cur.push_back(c);
      continue;
    }

    // Single quotes.
    if (c == '\\' && (i + 1) < line.size()) {
      const char n = line[i + 1];
      if (n == '\'' || n == '\\') {
        cur.push_back(c);
        cur.push_back(n);
        ++i;
        continue;
      }
    }
    if (c == '\'') {
      q = Q::None;
    }
    cur.push_back(c);
  }

  push();
  return out;
}

DevConsole::DevConsole()
{
  m_lines.reserve(128);
  m_history.reserve(64);
  // A small hint so users discover it.
  print("Console: press F4 to toggle, type 'help' for commands");
}

void DevConsole::toggle()
{
  if (m_open) close();
  else open();
}

void DevConsole::open()
{
  m_open = true;
  m_caretBlink = 0.0f;
  m_historyPos = -1;
  m_scroll = 0;
}

void DevConsole::close()
{
  m_open = false;
  m_scroll = 0;
}

void DevConsole::clearLog()
{
  m_lines.clear();
  m_scroll = 0;
}

void DevConsole::print(const std::string& line)
{
  // Keep log bounded.
  constexpr std::size_t kMaxLines = 250;
  m_lines.push_back(line);
  if (m_lines.size() > kMaxLines) {
    const std::size_t extra = m_lines.size() - kMaxLines;
    m_lines.erase(m_lines.begin(), m_lines.begin() + static_cast<std::ptrdiff_t>(extra));
  }
  // Any new output snaps the view to the bottom.
  m_scroll = 0;
}

void DevConsole::registerCommand(const std::string& name, const std::string& help, CommandFn fn)
{
  const std::string key = toLower(name);
  m_commands[key] = Command{help, std::move(fn)};
  m_commandOrder.push_back(name);
}

bool DevConsole::update(float dt, int screenW, int screenH, float mouseX, float mouseY)
{
  if (!m_open) return false;

  // Caret blink (simple square wave).
  m_caretBlink += dt;
  if (m_caretBlink > 1000.0f) m_caretBlink = 0.0f;

  // Mouse-wheel scroll (only when the mouse is over the console area).
  {
    const int consoleH = std::clamp(screenH / 3, 170, 460);
    const Rectangle rect = Rectangle{0.0f, static_cast<float>(screenH - consoleH), static_cast<float>(screenW),
                                     static_cast<float>(consoleH)};
    const Vector2 mp = Vector2{mouseX, mouseY};
    if (CheckCollisionPointRec(mp, rect)) {
      const float wheel = GetMouseWheelMove();
      if (wheel != 0.0f) {
        m_scroll += static_cast<int>(-wheel * 3.0f);
        if (m_scroll < 0) m_scroll = 0;
        const int maxScroll = std::max(0, static_cast<int>(m_lines.size()) - 1);
        if (m_scroll > maxScroll) m_scroll = maxScroll;
      }
    }
  }

  // Navigation keys.
  if (IsKeyPressed(KEY_UP)) navigateHistory(-1);
  if (IsKeyPressed(KEY_DOWN)) navigateHistory(+1);

  if (IsKeyPressed(KEY_PAGE_UP)) {
    m_scroll += 6;
    const int maxScroll = std::max(0, static_cast<int>(m_lines.size()) - 1);
    if (m_scroll > maxScroll) m_scroll = maxScroll;
  }
  if (IsKeyPressed(KEY_PAGE_DOWN)) {
    m_scroll -= 6;
    if (m_scroll < 0) m_scroll = 0;
  }

  if (IsKeyPressed(KEY_HOME)) {
    m_cursor = 0;
  }
  if (IsKeyPressed(KEY_END)) {
    m_cursor = m_input.size();
  }
  if (IsKeyPressed(KEY_LEFT)) {
    if (m_cursor > 0) m_cursor--;
  }
  if (IsKeyPressed(KEY_RIGHT)) {
    if (m_cursor < m_input.size()) m_cursor++;
  }

  // Text entry (Unicode codepoints).
  int key = GetCharPressed();
  while (key > 0) {
    // Filter out non-printable control chars.
    if (key >= 32 && key != 127) {
      const char c = static_cast<char>(key);
      m_input.insert(m_input.begin() + static_cast<std::ptrdiff_t>(m_cursor), c);
      m_cursor++;
    }
    key = GetCharPressed();
  }

  // Backspace / delete.
  if (IsKeyPressed(KEY_BACKSPACE)) {
    if (m_cursor > 0 && !m_input.empty()) {
      m_input.erase(m_input.begin() + static_cast<std::ptrdiff_t>(m_cursor - 1));
      m_cursor--;
    }
  }
  if (IsKeyPressed(KEY_DELETE)) {
    if (m_cursor < m_input.size()) {
      m_input.erase(m_input.begin() + static_cast<std::ptrdiff_t>(m_cursor));
    }
  }

  // Autocomplete.
  if (IsKeyPressed(KEY_TAB)) {
    autocomplete();
  }

  // Execute.
  if (IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER)) {
    const std::string line = trim(m_input);
    if (!line.empty()) {
      executeLine(line);
      m_history.push_back(line);
      if (m_history.size() > 100) m_history.erase(m_history.begin());
    }
    m_input.clear();
    m_cursor = 0;
    m_historyPos = -1;
    m_historyDraft.clear();
  }

  return true;
}

void DevConsole::draw(int screenW, int screenH)
{
  if (!m_open) return;

  const int consoleH = std::clamp(screenH / 3, 170, 460);
  const int x0 = 0;
  const int y0 = screenH - consoleH;
  const int w = screenW;
  const int h = consoleH;

  DrawRectangle(x0, y0, w, h, Color{0, 0, 0, 200});
  DrawRectangleLines(x0, y0, w, h, Color{255, 255, 255, 60});

  const int lineH = kFontSize + kLineGap;
  const int usableW = w - kPadding * 2;

  // Input line (bottom).
  const int inputY = y0 + h - kPadding - kFontSize;
  const std::string prompt = "> ";
  const std::string inputText = prompt + m_input;
  DrawText(inputText.c_str(), x0 + kPadding, inputY, kFontSize, RAYWHITE);

  // Caret.
  const bool caretOn = (std::fmod(m_caretBlink, 1.0f) < 0.5f);
  if (caretOn) {
    const std::string before = prompt + m_input.substr(0, m_cursor);
    const int caretX = x0 + kPadding + MeasureText(before.c_str(), kFontSize);
    DrawRectangle(caretX, inputY + 2, 8, kFontSize - 4, Color{255, 255, 255, 180});
  }

  // Log area.
  const int logTop = y0 + kPadding;
  const int logBottom = inputY - kPadding;
  const int maxVisibleLines = std::max(0, (logBottom - logTop) / lineH);
  if (maxVisibleLines <= 0) return;

  // Compute which segment of the log to show.
  const int total = static_cast<int>(m_lines.size());
  const int lastIndex = std::max(0, total - 1);
  const int bottomIndex = lastIndex - m_scroll;
  const int firstIndex = std::max(0, bottomIndex - (maxVisibleLines - 1));

  int y = logBottom - lineH;
  for (int i = bottomIndex; i >= firstIndex; --i) {
    if (i < 0 || i >= total) continue;
    const std::string s = EllipsizeToWidth(m_lines[static_cast<std::size_t>(i)], usableW, kFontSize);
    DrawText(s.c_str(), x0 + kPadding, y, kFontSize, Color{220, 220, 220, 255});
    y -= lineH;
  }

  // Scroll hint.
  if (m_scroll > 0) {
    const char* hint = "(scroll: mouse wheel / PgUp/PgDn)";
    const int hw = MeasureText(hint, 14);
    DrawText(hint, x0 + w - hw - 10, y0 + 8, 14, Color{200, 200, 200, 200});
  }
}

void DevConsole::executeLine(const std::string& line)
{
  // Support multiple commands separated by ';' (outside of quotes).
  const std::vector<std::string> cmds = SplitCommands(line);

  auto executeOne = [&](const std::string& cmdLine) {
    const std::string trimmed = trim(cmdLine);
    if (trimmed.empty()) return;

    print("> " + trimmed);

    const Args toks = tokenize(trimmed);
    if (toks.empty()) return;

    const std::string cmdKey = toLower(toks[0]);
    auto it = m_commands.find(cmdKey);
    if (it == m_commands.end()) {
      print("Unknown command: " + toks[0] + " (try 'help')");
      return;
    }

    Args args;
    if (toks.size() > 1) {
      args.assign(toks.begin() + 1, toks.end());
    }

    try {
      it->second.fn(*this, args);
    } catch (const std::exception& e) {
      print(std::string("Error: ") + e.what());
    }
  };

  for (const std::string& cmd : cmds) {
    executeOne(cmd);
  }
}


void DevConsole::autocomplete()
{
  const std::string line = trim(m_input);
  if (line.empty()) return;

  // Only autocomplete the first token (command name).
  const Args toks = tokenize(line);
  if (toks.empty()) return;
  const std::string prefix = toLower(toks[0]);

  std::vector<std::string> matches;
  matches.reserve(8);
  for (const auto& kv : m_commands) {
    if (kv.first.rfind(prefix, 0) == 0) {
      matches.push_back(kv.first);
    }
  }
  if (matches.empty()) return;

  std::sort(matches.begin(), matches.end());
  if (matches.size() == 1) {
    // Replace the command token while preserving any args.
    std::string rest;
    const std::size_t sp = m_input.find_first_of(" \t");
    if (sp != std::string::npos) rest = m_input.substr(sp);
    m_input = matches[0] + rest;
    m_cursor = m_input.size();
  } else {
    std::ostringstream os;
    os << "Matches:";
    for (const std::string& m : matches) os << " " << m;
    print(os.str());
  }
}

void DevConsole::navigateHistory(int delta)
{
  if (m_history.empty()) return;

  // Enter history navigation from the draft.
  if (m_historyPos < 0) {
    m_historyDraft = m_input;
    m_historyPos = static_cast<int>(m_history.size());
  }

  m_historyPos += delta;
  if (m_historyPos < 0) m_historyPos = 0;
  if (m_historyPos > static_cast<int>(m_history.size())) m_historyPos = static_cast<int>(m_history.size());

  if (m_historyPos == static_cast<int>(m_history.size())) {
    m_input = m_historyDraft;
  } else {
    m_input = m_history[static_cast<std::size_t>(m_historyPos)];
  }
  m_cursor = m_input.size();
}

std::string DevConsole::trim(const std::string& s)
{
  std::size_t a = 0;
  while (a < s.size() && std::isspace(static_cast<unsigned char>(s[a]))) a++;
  std::size_t b = s.size();
  while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) b--;
  return s.substr(a, b - a);
}

DevConsole::Args DevConsole::tokenize(const std::string& line)
{
  // A tiny shell-like tokenizer:
  //  - splits on whitespace
  //  - supports single and double quoted strings (quotes are removed)
  //  - supports simple escapes inside quotes: \" and \\ in double quotes, \' and \\ in single quotes
  //
  // IMPORTANT: we intentionally do *not* treat backslashes as escapes outside of quotes so Windows paths
  // like C:\\Users\\Name\\file.txt work as expected.
  Args out;
  out.reserve(8);

  enum class Q : unsigned char { None = 0, Single, Double };
  Q q = Q::None;

  std::string cur;
  cur.reserve(line.size());

  bool tokenStarted = false;

  auto flush = [&]() {
    if (!tokenStarted) return;
    out.push_back(cur);
    cur.clear();
    tokenStarted = false;
  };

  for (std::size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    const unsigned char uc = static_cast<unsigned char>(c);

    if (q == Q::None) {
      if (std::isspace(uc)) {
        flush();
        continue;
      }
      if (c == '"') {
        q = Q::Double;
        tokenStarted = true;
        continue;
      }
      if (c == '\'') {
        q = Q::Single;
        tokenStarted = true;
        continue;
      }
      tokenStarted = true;
      cur.push_back(c);
      continue;
    }

    if (q == Q::Double) {
      if (c == '"') {
        q = Q::None;
        continue;
      }
      if (c == '\\' && (i + 1) < line.size()) {
        const char n = line[i + 1];
        if (n == '"' || n == '\\') {
          cur.push_back(n);
          ++i;
          continue;
        }
      }
      cur.push_back(c);
      continue;
    }

    // Single quotes.
    if (c == '\'') {
      q = Q::None;
      continue;
    }
    if (c == '\\' && (i + 1) < line.size()) {
      const char n = line[i + 1];
      if (n == '\'' || n == '\\') {
        cur.push_back(n);
        ++i;
        continue;
      }
    }
    cur.push_back(c);
  }

  flush();
  return out;
}


std::string DevConsole::toLower(const std::string& s)
{
  std::string out = s;
  for (char& c : out) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return out;
}

} // namespace isocity
