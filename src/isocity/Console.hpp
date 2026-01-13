#pragma once

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace isocity {

// A lightweight in-game developer console.
//
// - Toggleable overlay that captures keyboard input.
// - Command registration + history.
// - Minimal UI rendering (implemented in Console.cpp using raylib).
//
// The console is designed to stay self-contained and platform independent
// (raylib only used in the .cpp).
class DevConsole {
public:
  using Args = std::vector<std::string>;
  using CommandFn = std::function<void(DevConsole&, const Args&)>;

  struct Command {
    std::string help;
    CommandFn fn;
  };

  DevConsole();

  void toggle();
  void open();
  void close();
  bool isOpen() const { return m_open; }

  void clearLog();
  void print(const std::string& line);

  void registerCommand(const std::string& name, const std::string& help, CommandFn fn);

  // Process keyboard input when the console is open.
  // Returns true if input was consumed.
  //
  // NOTE: screenW/screenH and mouse position are provided by the game so the
  // console can correctly handle mouse wheel scrolling even when the UI is
  // rendered with a scaling transform.
  bool update(float dt, int screenW, int screenH, float mouseX, float mouseY);

  // Draw the console overlay (no-op if closed).
  void draw(int screenW, int screenH);

  // Utility helpers for commands.
  const std::vector<std::string>& logLines() const { return m_lines; }
  const std::unordered_map<std::string, Command>& commands() const { return m_commands; }
  const std::vector<std::string>& commandOrder() const { return m_commandOrder; }

private:
  void executeLine(const std::string& line);
  void autocomplete();

  void rebuildSuggestions();
  void acceptSuggestion(std::size_t index);
  std::string commandToken() const;
  void navigateHistory(int delta);

  static std::string trim(const std::string& s);
  static Args tokenize(const std::string& line);
  static std::string toLower(const std::string& s);

  bool m_open = false;

  // Input line state.
  std::string m_input;
  std::size_t m_cursor = 0;
  float m_caretBlink = 0.0f;

  // Command history.
  std::vector<std::string> m_history;
  int m_historyPos = -1;
  std::string m_historyDraft;

  // Output log.
  std::vector<std::string> m_lines;
  int m_scroll = 0; // 0 = bottom, positive = scroll up

  // Last known mouse position in UI coordinates (updated from update()).
  float m_mouseUiX = 0.0f;
  float m_mouseUiY = 0.0f;


  // Autocomplete suggestion state (rebuilt lazily as the input line changes).
  std::string m_suggestQuery;
  std::vector<std::string> m_suggestions;
  int m_suggestIndex = 0;

  // Registered commands.
  std::unordered_map<std::string, Command> m_commands; // lowercase key
  std::vector<std::string> m_commandOrder;             // original names in insertion order
};

} // namespace isocity
