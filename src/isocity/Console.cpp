#include "isocity/Console.hpp"

#include "isocity/Ui.hpp"

#include "isocity/RaylibShim.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <exception>
#include <sstream>
#include <string_view>

namespace isocity {

namespace {
constexpr int kFontSize = 18;
constexpr int kLineGap = 4;
constexpr int kPadding = 12;
constexpr int kMargin = 12;

constexpr int kHeaderH = 34;
constexpr int kInputH = kFontSize + 14;

constexpr int kMaxSuggestionsShown = 8;

inline bool IsWS(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }

// A small layout helper so update() and draw() stay in sync.
struct ConsoleLayout {
  Rectangle panel{};
  Rectangle logBox{};
  Rectangle inputBox{};
  Rectangle suggestBox{}; // {0,0,0,0} if hidden
  int lineH = 0;
};

ConsoleLayout MakeLayout(int screenW, int screenH, int suggestionRows)
{
  ConsoleLayout l{};
  const int consoleH = std::clamp(screenH / 3, 210, 560);

  l.panel = Rectangle{static_cast<float>(kMargin),
                      static_cast<float>(screenH - consoleH - kMargin),
                      static_cast<float>(screenW - kMargin * 2),
                      static_cast<float>(consoleH)};

  l.lineH = kFontSize + kLineGap;

  const float innerX = l.panel.x + static_cast<float>(kPadding);
  const float innerW = l.panel.width - static_cast<float>(kPadding * 2);

  const float inputY = l.panel.y + l.panel.height - static_cast<float>(kPadding + kInputH);
  l.inputBox = Rectangle{innerX, inputY, innerW, static_cast<float>(kInputH)};

  float suggestH = 0.0f;
  if (suggestionRows > 0) {
    suggestH = 12.0f + static_cast<float>(suggestionRows * l.lineH);
    const float suggestY = inputY - 6.0f - suggestH;
    l.suggestBox = Rectangle{innerX, suggestY, innerW, suggestH};
  } else {
    l.suggestBox = Rectangle{0, 0, 0, 0};
  }

  const float logTop = l.panel.y + static_cast<float>(kPadding + kHeaderH);
  const float logBottom = (suggestionRows > 0) ? (l.suggestBox.y - 6.0f) : (inputY - 6.0f);
  l.logBox = Rectangle{innerX, logTop, innerW, std::max(0.0f, logBottom - logTop)};
  return l;
}

std::string EllipsizeToWidthUI(const std::string& s, int maxWidth, int fontSize, bool bold = false)
{
  if (maxWidth <= 0) return std::string();
  if (ui::MeasureTextWidth(s, fontSize, bold, 1) <= maxWidth) return s;

  std::string out = s;
  while (!out.empty() && ui::MeasureTextWidth(out + "...", fontSize, bold, 1) > maxWidth) {
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
  m_suggestions.reserve(16);
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
  m_cursor = m_input.size();
  rebuildSuggestions();
}

void DevConsole::close()
{
  m_open = false;
  m_scroll = 0;
  m_suggestions.clear();
  m_suggestQuery.clear();
  m_suggestIndex = 0;
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

  // If the user has the console open, refresh suggestions so new commands appear immediately.
  if (m_open) {
    m_suggestQuery.clear();
    rebuildSuggestions();
  }
}

std::string DevConsole::commandToken() const
{
  std::size_t i = 0;
  while (i < m_input.size() && (m_input[i] == ' ' || m_input[i] == '\t')) ++i;
  std::size_t j = i;
  while (j < m_input.size() && !IsWS(m_input[j])) ++j;
  return m_input.substr(i, j - i);
}

void DevConsole::rebuildSuggestions()
{
  // Only suggest while the caret is inside the command token (first word).
  std::size_t cmdStart = 0;
  while (cmdStart < m_input.size() && (m_input[cmdStart] == ' ' || m_input[cmdStart] == '\t')) ++cmdStart;
  std::size_t cmdEnd = cmdStart;
  while (cmdEnd < m_input.size() && !IsWS(m_input[cmdEnd])) ++cmdEnd;

  const bool caretInCmd = (m_cursor >= cmdStart && m_cursor <= cmdEnd);
  if (!caretInCmd) {
    m_suggestions.clear();
    m_suggestQuery.clear();
    m_suggestIndex = 0;
    return;
  }

  const std::string tok = toLower(m_input.substr(cmdStart, cmdEnd - cmdStart));
  if (tok.empty()) {
    m_suggestions.clear();
    m_suggestQuery.clear();
    m_suggestIndex = 0;
    return;
  }

  if (tok == m_suggestQuery) {
    // Keep the selection stable while the query is unchanged.
    if (m_suggestIndex < 0) m_suggestIndex = 0;
    if (m_suggestIndex >= static_cast<int>(m_suggestions.size())) m_suggestIndex = 0;
    return;
  }

  m_suggestQuery = tok;
  m_suggestions.clear();
  m_suggestions.reserve(16);

  for (const auto& kv : m_commands) {
    if (kv.first.rfind(tok, 0) == 0) {
      m_suggestions.push_back(kv.first);
    }
  }

  std::sort(m_suggestions.begin(), m_suggestions.end());
  m_suggestIndex = 0;
}

void DevConsole::acceptSuggestion(std::size_t index)
{
  if (index >= m_suggestions.size()) return;
  const std::string& cmd = m_suggestions[index];

  // Preserve everything after the command token.
  std::size_t cmdStart = 0;
  while (cmdStart < m_input.size() && (m_input[cmdStart] == ' ' || m_input[cmdStart] == '\t')) ++cmdStart;
  std::size_t cmdEnd = cmdStart;
  while (cmdEnd < m_input.size() && !IsWS(m_input[cmdEnd])) ++cmdEnd;

  std::string rest;
  if (cmdEnd < m_input.size()) rest = m_input.substr(cmdEnd);

  m_input = cmd + rest;

  // If the caret was inside the command token, snap it to the end of the accepted command.
  if (m_cursor <= cmdEnd) {
    m_cursor = cmd.size();
  } else {
    // Keep caret position stable in the remainder of the line (best-effort).
    const std::ptrdiff_t delta = static_cast<std::ptrdiff_t>(cmd.size()) - static_cast<std::ptrdiff_t>(cmdEnd - cmdStart);
    const std::ptrdiff_t newPos = static_cast<std::ptrdiff_t>(m_cursor) + delta;
    m_cursor = static_cast<std::size_t>(std::clamp<std::ptrdiff_t>(newPos, 0, static_cast<std::ptrdiff_t>(m_input.size())));
  }

  // Force a refresh so the suggestion list reflects the new prefix/exact command.
  m_suggestQuery.clear();
  rebuildSuggestions();
}

bool DevConsole::update(float dt, int screenW, int screenH, float mouseX, float mouseY)
{
  if (!m_open) return false;

  // Cache mouse position for drawing widgets that need it (e.g. scrollbars).
  m_mouseUiX = mouseX;
  m_mouseUiY = mouseY;

  // Close on escape (classic console behavior).
  if (IsKeyPressed(KEY_ESCAPE)) {
    close();
    return true;
  }

  const bool ctrl = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);

  // Small readline-ish conveniences.
  if (ctrl && IsKeyPressed(KEY_L)) clearLog();           // Ctrl+L: clear log
  if (ctrl && IsKeyPressed(KEY_A)) m_cursor = 0;         // Ctrl+A: start of line
  if (ctrl && IsKeyPressed(KEY_E)) m_cursor = m_input.size(); // Ctrl+E: end of line

  if (ctrl && IsKeyPressed(KEY_U)) { // Ctrl+U: delete to start
    if (m_cursor > 0) {
      m_input.erase(0, m_cursor);
      m_cursor = 0;
    }
  }
  if (ctrl && IsKeyPressed(KEY_K)) { // Ctrl+K: delete to end
    if (m_cursor < m_input.size()) {
      m_input.erase(m_cursor);
    }
  }
  if (ctrl && IsKeyPressed(KEY_W)) { // Ctrl+W: delete previous word
    if (m_cursor > 0) {
      std::size_t end = m_cursor;
      std::size_t start = end;
      while (start > 0 && std::isspace(static_cast<unsigned char>(m_input[start - 1]))) --start;
      while (start > 0 && !std::isspace(static_cast<unsigned char>(m_input[start - 1]))) --start;
      m_input.erase(start, end - start);
      m_cursor = start;
    }
  }

  // Clipboard helpers (input line only).
  if (ctrl && IsKeyPressed(KEY_C)) { // Ctrl+C: copy input
    SetClipboardText(m_input.c_str());
  }
  if (ctrl && IsKeyPressed(KEY_X)) { // Ctrl+X: cut input
    SetClipboardText(m_input.c_str());
    m_input.clear();
    m_cursor = 0;
  }
  if (ctrl && IsKeyPressed(KEY_V)) { // Ctrl+V: paste
    const char* clip = GetClipboardText();
    if (clip && clip[0]) {
      std::string clipStr(clip);
      // Normalize newlines to spaces for single-line input.
      clipStr.erase(std::remove(clipStr.begin(), clipStr.end(), '\r'), clipStr.end());
      std::replace(clipStr.begin(), clipStr.end(), '\n', ' ');
      m_input.insert(m_cursor, clipStr);
      m_cursor += clipStr.size();
    }
  }

  // Caret blink (simple square wave).
  m_caretBlink += dt;
  if (m_caretBlink > 1000.0f) m_caretBlink = 0.0f;

  rebuildSuggestions();
  const int suggestRows = std::min(kMaxSuggestionsShown, static_cast<int>(m_suggestions.size()));
  const ConsoleLayout layout = MakeLayout(screenW, screenH, suggestRows);

  // Mouse-wheel scroll (only when the mouse is over the log area).
  {
    const Vector2 mp = Vector2{mouseX, mouseY};
    if (layout.logBox.width > 0.0f && layout.logBox.height > 0.0f && CheckCollisionPointRec(mp, layout.logBox)) {
      const float wheel = GetMouseWheelMove();
      if (wheel != 0.0f) {
        m_scroll += static_cast<int>(-wheel * 3.0f);
        if (m_scroll < 0) m_scroll = 0;
        const int maxScroll = std::max(0, static_cast<int>(m_lines.size()) - 1);
        if (m_scroll > maxScroll) m_scroll = maxScroll;
      }
    }
  }

  // If the user clicks a suggestion, accept it immediately.
  if (suggestRows > 0 && IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
    const Vector2 mp = Vector2{mouseX, mouseY};
    if (layout.suggestBox.width > 0.0f && layout.suggestBox.height > 0.0f &&
        CheckCollisionPointRec(mp, layout.suggestBox)) {
      const float listTop = layout.suggestBox.y + 8.0f;
      const int row = static_cast<int>(std::floor((mp.y - listTop) / static_cast<float>(layout.lineH)));
      if (row >= 0 && row < suggestRows) {
        acceptSuggestion(static_cast<std::size_t>(row));
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
    m_suggestQuery.clear();
    rebuildSuggestions();
  }

  rebuildSuggestions();
  return true;
}

void DevConsole::draw(int screenW, int screenH)
{
  if (!m_open) return;

  const float timeSec = static_cast<float>(GetTime());
  const ui::Theme& th = ui::GetTheme();

  rebuildSuggestions();

  // Hide suggestions when there's only an exact match.
  int suggestRows = 0;
  if (!m_suggestions.empty() && !m_suggestQuery.empty()) {
    if (!(m_suggestions.size() == 1 && m_suggestions[0] == m_suggestQuery)) {
      suggestRows = std::min(kMaxSuggestionsShown, static_cast<int>(m_suggestions.size()));
    }
  }

  const ConsoleLayout layout = MakeLayout(screenW, screenH, suggestRows);

  ui::DrawPanel(layout.panel, timeSec, /*active=*/true);

  // Header (title + active command help preview).
  ui::DrawPanelHeader(Rectangle{layout.panel.x, layout.panel.y, layout.panel.width, static_cast<float>(kHeaderH) + 8.0f},
                      "DEV CONSOLE", timeSec, /*active=*/true, 20);

  // Determine which command to preview help for: exact command token first, otherwise current suggestion.
  std::string activeCmd = toLower(commandToken());
  if (activeCmd.empty() && !m_suggestions.empty()) activeCmd = m_suggestions[0];
  auto hit = m_commands.find(activeCmd);

  if (hit != m_commands.end() && !hit->second.help.empty()) {
    const int helpFont = 14;
    const int x = static_cast<int>(layout.panel.x) + 170;
    const int y = static_cast<int>(layout.panel.y) + 12;
    const int maxW = std::max(0, static_cast<int>(layout.panel.width) - (x - static_cast<int>(layout.panel.x)) - 18);
    const std::string help = EllipsizeToWidthUI(hit->second.help, maxW, helpFont, /*bold=*/false);
    ui::Text(x, y, helpFont, help, th.textDim, /*bold=*/false, /*shadow=*/true, 1);
  } else {
    const int helpFont = 14;
    const int x = static_cast<int>(layout.panel.x) + 170;
    const int y = static_cast<int>(layout.panel.y) + 12;
    ui::Text(x, y, helpFont, "Tab: autocomplete   ;: chain commands   Quotes supported", th.textDim,
             /*bold=*/false, /*shadow=*/true, 1);
  }

  // Log panel.
  ui::DrawPanelInset(layout.logBox, timeSec, /*active=*/true);

  const float sbW = 12.0f;
  const int usableW = static_cast<int>(layout.logBox.width) - 16 - static_cast<int>(sbW) - 4;
  const int logTop = static_cast<int>(layout.logBox.y) + 8;
  const int logBottom = static_cast<int>(layout.logBox.y + layout.logBox.height) - 8;
  const int maxVisibleLines = std::max(0, (logBottom - logTop) / layout.lineH);

  const Rectangle scrollBarR{layout.logBox.x + layout.logBox.width - sbW - 4.0f,
                             static_cast<float>(logTop - 2),
                             sbW,
                             static_cast<float>((logBottom - logTop) + 4)};
  const Vector2 mp{m_mouseUiX, m_mouseUiY};

  if (maxVisibleLines > 0) {
    // Compute which segment of the log to show.
    const int total = static_cast<int>(m_lines.size());
    const int maxScroll = std::max(0, total - maxVisibleLines);
    m_scroll = std::clamp(m_scroll, 0, maxScroll);

    // Scrollbar (maps the log scroll to a "top index" so the thumb behaves normally).
    int topIndex = std::clamp(total - maxVisibleLines - m_scroll, 0, maxScroll);
    if (ui::ScrollbarV(901, scrollBarR, total, maxVisibleLines, topIndex, mp, timeSec, /*enabled=*/true)) {
      m_scroll = std::clamp(total - maxVisibleLines - topIndex, 0, maxScroll);
    }

    const int lastIndex = std::max(0, total - 1);
    const int bottomIndex = lastIndex - m_scroll;
    const int firstIndex = std::max(0, bottomIndex - (maxVisibleLines - 1));

    int y = logBottom - layout.lineH;
    for (int i = bottomIndex; i >= firstIndex; --i) {
      if (i < 0 || i >= total) continue;
      const std::string& raw = m_lines[static_cast<std::size_t>(i)];
      const std::string s = EllipsizeToWidthUI(raw, usableW, kFontSize, /*bold=*/false);

      Color c = Color{220, 220, 220, 255};
      if (!raw.empty() && raw[0] == '>') c = th.accent;

      ui::Text(static_cast<int>(layout.logBox.x) + 8, y, kFontSize, s, c, /*bold=*/false, /*shadow=*/true, 1);
      y -= layout.lineH;
    }
  }

  // Suggestions (optional).
  if (suggestRows > 0) {
    ui::DrawPanelInset(layout.suggestBox, timeSec, /*active=*/true);

    const int sx = static_cast<int>(layout.suggestBox.x) + 8;
    int y = static_cast<int>(layout.suggestBox.y) + 8;
    const int maxW = static_cast<int>(layout.suggestBox.width) - 16;

    ui::Text(sx, y, 14, "Autocomplete:", th.textDim, /*bold=*/false, /*shadow=*/true, 1);
    y += 16;

    const int rowTop = y;
    for (int i = 0; i < suggestRows; ++i) {
      const bool selected = (i == m_suggestIndex);
      const Rectangle rowR{layout.suggestBox.x + 4.0f,
                           static_cast<float>(rowTop + i * layout.lineH - 2),
                           layout.suggestBox.width - 8.0f,
                           static_cast<float>(layout.lineH)};
      if (selected) {
        ui::DrawSelectionHighlight(rowR, timeSec, /*strong=*/false);
      }

      const std::string& cmd = m_suggestions[static_cast<std::size_t>(i)];
      ui::Text(sx, rowTop + i * layout.lineH, 16, cmd, selected ? th.text : th.textDim,
               /*bold=*/selected, /*shadow=*/true, 1);

      // Optional tiny hint on the right: show the start of the help string.
      auto it = m_commands.find(cmd);
      if (it != m_commands.end() && !it->second.help.empty()) {
        const int hintFont = 14;
        const std::string hint = EllipsizeToWidthUI(it->second.help, maxW / 2, hintFont, false);
        const int hintW = ui::MeasureTextWidth(hint, hintFont, false, 1);
        ui::Text(sx + maxW - hintW, rowTop + i * layout.lineH + 1, hintFont, hint, th.textDim,
                 /*bold=*/false, /*shadow=*/true, 1);
      }
    }
  }

  // Input panel.
  ui::DrawPanelInset(layout.inputBox, timeSec, /*active=*/true);

  const int inputX = static_cast<int>(layout.inputBox.x) + 8;
  const int inputY = static_cast<int>(layout.inputBox.y) + 6;

  // Prompt + colored command token.
  const std::string prompt = "> ";
  ui::Text(inputX, inputY, kFontSize, prompt, th.textDim, /*bold=*/false, /*shadow=*/true, 1);

  const int promptW = ui::MeasureTextWidth(prompt, kFontSize, false, 1);
  int cx = inputX + promptW;

  // Split the input into [cmd][rest] so we can tint the command token.
  std::size_t cmdStart = 0;
  while (cmdStart < m_input.size() && (m_input[cmdStart] == ' ' || m_input[cmdStart] == '\t')) ++cmdStart;
  std::size_t cmdEnd = cmdStart;
  while (cmdEnd < m_input.size() && !IsWS(m_input[cmdEnd])) ++cmdEnd;

  const std::string cmd = m_input.substr(cmdStart, cmdEnd - cmdStart);
  const std::string rest = (cmdEnd < m_input.size()) ? m_input.substr(cmdEnd) : std::string();

  if (!cmd.empty()) {
    const std::string cmdKey = toLower(cmd);
    const bool known = (m_commands.find(cmdKey) != m_commands.end());
    const Color cmdColor = known ? th.accent : Color{255, 80, 80, 255};
    ui::Text(cx, inputY, kFontSize, cmd, cmdColor, /*bold=*/known, /*shadow=*/true, 1);
    cx += ui::MeasureTextWidth(cmd, kFontSize, known, 1);
  }

  if (!rest.empty()) {
    ui::Text(cx, inputY, kFontSize, rest, th.text, /*bold=*/false, /*shadow=*/true, 1);
  }

  // Caret.
  const bool caretOn = (std::fmod(m_caretBlink, 1.0f) < 0.5f);
  if (caretOn) {
    const std::string before = prompt + m_input.substr(0, m_cursor);
    const int caretX = inputX + ui::MeasureTextWidth(before, kFontSize, false, 1);
    DrawRectangle(caretX, inputY + 2, 8, kFontSize - 4, Color{255, 255, 255, 200});
  }

  // Scroll hint.
  if (m_scroll > 0) {
    const std::string hint = "(scroll: wheel / PgUp / PgDn)";
    const int hw = ui::MeasureTextWidth(hint, 14, false, 1);
    ui::Text(static_cast<int>(layout.panel.x + layout.panel.width) - hw - 18,
             static_cast<int>(layout.panel.y) + 12, 14, hint, th.textDim, false, true, 1);
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
  rebuildSuggestions();
  if (m_suggestions.empty()) return;

  const bool shift = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);

  if (shift) {
    // Cycle backward.
    if (m_suggestIndex <= 0) m_suggestIndex = static_cast<int>(m_suggestions.size()) - 1;
    else m_suggestIndex--;
    acceptSuggestion(static_cast<std::size_t>(m_suggestIndex));
  } else {
    // Accept current selection, then advance for the next Tab.
    acceptSuggestion(static_cast<std::size_t>(m_suggestIndex));
    m_suggestIndex++;
    if (m_suggestIndex >= static_cast<int>(m_suggestions.size())) m_suggestIndex = 0;
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

  // Rebuild suggestions for the new input.
  m_suggestQuery.clear();
  rebuildSuggestions();
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
