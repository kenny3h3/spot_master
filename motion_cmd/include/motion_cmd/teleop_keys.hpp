#pragma once
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <optional>
#include <string>

namespace motion_cmd {

class TeleopKeys {
public:
  TeleopKeys() : initialized_(false) {}

  bool init() {
    if (initialized_) return true;
    if (tcgetattr(STDIN_FILENO, &old_) < 0) return false;
    new_ = old_;
    new_.c_lflag &= ~(ICANON | ECHO);
    new_.c_cc[VMIN] = 0;
    new_.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &new_) < 0) return false;
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FIL_FILENO, F_SETFL, flags | O_NONBLOCK);
    initialized_ = true;
    return true;
  }

  void shutdown() {
    if (initialized_) tcsetattr(STDIN_FILENO, TCSANOW, &old_);
    initialized_ = false;
  }

  // Arrow: "UP","DOWN","LEFT","RIGHT"; sonst 1-char string (z.B. "s"," ")
  std::optional<std::string> readKey() {
    char c;
    if (read(STDIN_FILENO, &c, 1) != 1) return std::nullopt;
    if (c == '\x1b') {
      char seq[2];
      if (read(STDIN_FILENO, &seq[0], 1) != 1) return std::nullopt;
      if (read(STDIN_FILENO, &seq[1], 1) != 1) return std::nullopt;
      if (seq[0] == '[') {
        switch (seq[1]) {
          case 'A': return std::make_optional(std::string("UP"));
          case 'B': return std::make_optional(std::string("DOWN"));
          case 'C': return std::make_optional(std::string("RIGHT"));
          case 'D': return std::make_optional(std::string("LEFT"));
        }
      }
      return std::nullopt;
    } else {
      return std::make_optional(std::string(1, c));
    }
  }

private:
  bool initialized_;
  struct termios old_{}, new_{};
};

} // namespace motion_cmd
