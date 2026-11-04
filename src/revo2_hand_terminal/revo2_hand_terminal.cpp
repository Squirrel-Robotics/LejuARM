#include <lejusdk-lowlevel/leju_sdk.h>
#include <lejusdk-utils/robot_version.hpp>
#include <lejusdk-utils/time_utils.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace {

std::atomic<bool> g_running{true};

void signalHandler(int) { g_running = false; }

std::string lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

void printHelp() {
  std::cout
      << "Commands:\n"
      << "  status                   show hardware and measured hand state\n"
      << "  left  open|close         set all six left-hand channels to 0 or 100\n"
      << "  right open|close         set all six right-hand channels to 0 or 100\n"
      << "  both  open|close         set both hands to 0 or 100\n"
      << "  left  set <0-100>        set all six left-hand channels\n"
      << "  right set <0-100>        set all six right-hand channels\n"
      << "  both  set <0-100>        set all twelve channels\n"
      << "  left  finger <1-6> <0-100>\n"
      << "  right finger <1-6> <0-100>\n"
      << "  stop                     hold both hands at measured positions\n"
      << "  help\n"
      << "  quit\n\n"
      << "0 means open and 100 means closed, following the existing VR controller.\n"
      << "Default mode requires both hands to report fresh, valid feedback.\n"
      << "Use --right-only only when the left hand is physically disconnected.\n";
}

class HandTerminal {
 public:
  explicit HandTerminal(bool right_only) : right_only_(right_only) {}

  void onHardwareState(const leju::StringDataConstPtr& message) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    hardware_state_ = leju::String2HwState(message->data);
    hardware_state_text_ = message->data;
    received_hardware_state_ = true;
  }

  void onHandState(const leju::HandStateConstPtr& message) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    hand_state_ = *message;
    hand_state_received_at_ = std::chrono::steady_clock::now();
    received_hand_state_ = true;
  }

  bool ready() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return feedbackErrorLocked().empty();
  }

  void printStatus() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    std::cout << "Hardware state: "
              << (received_hardware_state_ ? hardware_state_text_ : "尚未收到")
              << "\n手部反馈: "
              << (received_hand_state_ ? "已收到" : "尚未收到") << "\n";
    if (received_hand_state_) {
      std::cout << "  left valid=" << hand_state_.left_valid
                << " 数据年龄=" << hand_state_.left_sample_age_ms << " ms\n"
                << "  right valid=" << hand_state_.right_valid
                << " 数据年龄=" << hand_state_.right_sample_age_ms << " ms\n";
      if (hand_state_.position.size() == 12) {
        std::cout << "  measured left :";
        for (int i = 0; i < 6; ++i) std::cout << ' ' << hand_state_.position[i];
        std::cout << "\n  measured right:";
        for (int i = 6; i < 12; ++i) std::cout << ' ' << hand_state_.position[i];
        std::cout << '\n';
      }
    }
    if (sent_command_) {
      std::cout << "Last command sent by this program:\n  left :";
      for (int i = 0; i < 6; ++i) std::cout << ' ' << last_command_[i];
      std::cout << "\n  right:";
      for (int i = 6; i < 12; ++i) std::cout << ' ' << last_command_[i];
      std::cout << '\n';
    }
    const auto problem = feedbackErrorLocked();
    std::cout << "Motion commands: " << (problem.empty() ? "available" : "blocked: " + problem)
              << '\n';
  }

  bool setHand(const std::string& side, double value) {
    if ((side != "left" && side != "right" && side != "both") ||
        !std::isfinite(value) || value < 0.0 || value > 100.0) return false;
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!readyOrExplainLocked()) return false;
    if (right_only_ && side != "right") {
      std::cerr << "Right-only mode cannot command the left hand.\n";
      return false;
    }
    auto positions = hand_state_.position;
    if (side == "left" || side == "both") {
      for (int i = 0; i < 6; ++i) positions[i] = value;
    }
    if (side == "right" || side == "both") {
      for (int i = 6; i < 12; ++i) positions[i] = value;
    }
    return publishLocked(positions);
  }

  bool setFinger(const std::string& side, int finger, double value) {
    if (finger < 1 || finger > 6 ||
        (side != "left" && side != "right") ||
        !std::isfinite(value) || value < 0.0 || value > 100.0) {
      std::cerr << "Finger must be from 1 to 6.\n";
      return false;
    }
    const int index = (side == "left" ? 0 : 6) + finger - 1;
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!readyOrExplainLocked()) return false;
    if (right_only_ && side != "right") {
      std::cerr << "Right-only mode cannot command the left hand.\n";
      return false;
    }
    auto positions = hand_state_.position;
    positions[index] = value;
    return publishLocked(positions);
  }

  bool stop() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!readyOrExplainLocked()) return false;
    return publishLocked(hand_state_.position);
  }

  bool nudge(const std::string& side, int channel, double delta) {
    if (channel < 0 || channel > 6 || !std::isfinite(delta) ||
        (side != "left" && side != "right" && side != "both")) return false;
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!readyOrExplainLocked()) return false;
    if (right_only_ && side != "right") {
      std::cerr << "Right-only mode cannot command the left hand.\n";
      return false;
    }
    auto positions = hand_state_.position;
    bool changed = false;
    const auto change = [&](int index) {
      const double next = std::clamp(positions[index] + delta, 0.0, 100.0);
      changed |= next != positions[index];
      positions[index] = next;
    };
    for (int hand = 0; hand < 2; ++hand) {
      if (side != "both" && (side == "left") != (hand == 0)) continue;
      if (channel == 0) {
        for (int finger = 0; finger < 6; ++finger) change(hand * 6 + finger);
      } else {
        change(hand * 6 + channel - 1);
      }
    }
    if (!changed) {
      std::cout << "所选通道已到范围边界。\n";
      return false;
    }
    return publishLocked(positions);
  }

  void renderKeyboardScreen(const std::string& side, int channel,
                            double step, const std::string& last_action) const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    std::cout << "\033[H\033[2J"
              << "Roban2 Revo2 双手键盘控制\n"
              << "Mac 键盘 → SSH 终端 → 下位机 → 灵巧手\n\n"
              << "模式: " << (right_only_ ? "仅右手" : "双手")
              << "\n"
              << "硬件: "
              << (received_hardware_state_ ? hardware_state_text_ : "尚未收到")
              << "\n手部反馈: "
              << (received_hand_state_ ? "已收到" : "尚未收到");
    if (received_hand_state_) {
      std::cout << " | 左手 " << (hand_state_.left_valid ? "有效" : "无效")
                << " 数据年龄=" << hand_state_.left_sample_age_ms << "ms"
                << " | 右手 " << (hand_state_.right_valid ? "有效" : "无效")
                << " 数据年龄=" << hand_state_.right_sample_age_ms << "ms";
    }
    const auto problem = feedbackErrorLocked();
    std::cout << "\n操作: " << (problem.empty() ? "可操作" : "已锁定: " + problem)
              << "\n\n实测位置（0=张开，100=闭合）:\n";
    for (int hand = 0; hand < 2; ++hand) {
      std::cout << (hand == 0 ? "  左手:" : "  右手:");
      for (int finger = 0; finger < 6; ++finger) {
        const int index = hand * 6 + finger;
        if (received_hand_state_ && hand_state_.position.size() == 12) {
          std::cout << ' ' << finger + 1 << '=' << std::fixed
                    << std::setprecision(1) << hand_state_.position[index];
        } else {
          std::cout << ' ' << finger + 1 << "=?";
        }
      }
      std::cout << '\n';
    }
    std::cout << "\n选择: " << (side == "left" ? "左手" : side == "right" ? "右手" : "双手") << " | "
              << (channel < 0 ? "尚未选择" : channel == 0 ? "全部六路" : "通道 " + std::to_string(channel))
              << " | 步长: " << step << "\n"
              << (right_only_ ? "d=右手  0=全部通道  1–6=单通道\n"
                              : "a=左手  d=右手  b=双手  0=全部通道  1–6=单通道\n")
              << "j=张开一小步  k=闭合一小步  -=减小步长  +=增大步长\n"
              << "空格=保持实测位置  q=退出（退出只停止本程序发送）\n"
              << "最近操作: " << last_action << "\n"
              << "保持位置按键不是急停，异常运动时使用实体急停。\n"
              << std::flush;
  }

 private:
  std::string feedbackErrorLocked() const {
    if (!received_hardware_state_ || hardware_state_ != leju::HardwareState::READY_OK) {
      return "硬件未处于 READY_OK";
    }
    if (!received_hand_state_ || hand_state_.position.size() != 12 || !hand_state_.isValid()) {
      return "缺少有效的 12 路手部反馈";
    }
    const auto elapsed = std::chrono::steady_clock::now() - hand_state_received_at_;
    if (elapsed > std::chrono::milliseconds(1000)) {
      return "手部反馈已超过 1 秒未更新";
    }
    if (right_only_) {
      if (hand_state_.left_valid) {
        return "左手恢复在线，请使用双手模式重启本程序";
      }
      if (!hand_state_.right_valid || hand_state_.right_sample_age_ms > 1000) {
        return "右手采样无效或过期";
      }
    } else if (!hand_state_.left_valid || !hand_state_.right_valid ||
               hand_state_.left_sample_age_ms > 1000 ||
               hand_state_.right_sample_age_ms > 1000) {
      return "左手或右手采样无效或过期";
    }
    const int first = right_only_ ? 6 : 0;
    for (int i = first; i < 12; ++i) {
      const double value = hand_state_.position[i];
      if (!std::isfinite(value) || value < 0.0 || value > 100.0) {
        return "手部反馈位置超出范围";
      }
    }
    return {};
  }

  bool readyOrExplainLocked() const {
    const auto problem = feedbackErrorLocked();
    if (!problem.empty()) {
      std::cerr << "Refusing command: " << problem << ".\n";
      return false;
    }
    return true;
  }

  bool publishLocked(const std::vector<double>& positions) {
    if (positions.size() != 12 ||
        std::any_of(positions.begin(), positions.end(), [](double p) {
          return !std::isfinite(p) || p < 0.0 || p > 100.0;
        })) {
      std::cerr << "拒绝发送：目标必须是 12 个范围为 0–100 的有限数值。\n";
      return false;
    }
    leju::HandCmd command;
    command.position = positions;
    if (right_only_) {
      for (int i = 0; i < 6; ++i) command.position[i] = 0.0;
    }
    command.timestamp = leju::common::GetUnixTimestampS();
    if (!leju::GlobalRobot::getInstance().publishHandCmd(command)) {
      std::cerr << "SDK 拒绝发送手部目标。\n";
      return false;
    }
    for (int i = 0; i < 12; ++i) last_command_[i] = command.position[i];
    sent_command_ = true;
    std::cout << "目标已发布；实际到位请查看反馈。\n";
    return true;
  }

  mutable std::mutex state_mutex_;
  bool right_only_{false};
  leju::HardwareState hardware_state_{leju::HardwareState::UNKNOWN};
  std::string hardware_state_text_{"UNKNOWN"};
  bool received_hardware_state_{false};
  leju::HandState hand_state_{};
  std::chrono::steady_clock::time_point hand_state_received_at_{};
  bool received_hand_state_{false};
  std::array<double, 12> last_command_{};
  bool sent_command_{false};
};

bool parseValue(std::istringstream& input, double& value) {
  if (!(input >> value)) {
    std::cerr << "Missing or invalid value. Expected a number from 0 to 100.\n";
    return false;
  }
  if (!std::isfinite(value) || value < 0.0 || value > 100.0) {
    std::cerr << "Value must be from 0 to 100.\n";
    return false;
  }
  return true;
}

class RawTerminal {
 public:
  RawTerminal() {
    if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &original_) != 0) return;
    termios raw = original_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
      active_ = true;
      std::cout << "\033[?25l" << std::flush;
    }
  }

  ~RawTerminal() {
    if (active_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &original_);
      std::cout << "\033[?25h\n" << std::flush;
    }
  }

  bool active() const { return active_; }

 private:
  termios original_{};
  bool active_{false};
};

void runKeyboardMode(HandTerminal& terminal, bool right_only) {
  RawTerminal raw;
  if (!raw.active()) {
    std::cerr << "键盘模式需要交互式终端，请使用普通 SSH 窗口或 ssh -t。\n";
    return;
  }

  std::string side = right_only ? "right" : "left";
  std::string last_action = "尚未发送指令";
  int channel = -1;
  double step = 2.0;
  bool has_last_motion = false;
  auto last_motion = std::chrono::steady_clock::time_point{};
  auto last_draw = std::chrono::steady_clock::time_point{};

  while (g_running) {
    const auto now = std::chrono::steady_clock::now();
    const auto refresh_interval = terminal.ready()
                                      ? std::chrono::milliseconds(500)
                                      : std::chrono::milliseconds(5000);
    if (now - last_draw >= refresh_interval) {
      terminal.renderKeyboardScreen(side, channel, step, last_action);
      last_draw = now;
    }

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);
    timeval timeout{0, 100000};
    const int available = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout);
    if (available <= 0) continue;

    char key = 0;
    const ssize_t count = read(STDIN_FILENO, &key, 1);
    if (count == 0) break;
    if (count != 1) continue;
    if (key == 'q' || key == 'Q') break;
    if (key == 'a' && !right_only) { side = "left"; last_action = "已选择左手"; }
    else if (key == 'd') { side = "right"; last_action = "已选择右手"; }
    else if (key == 'b' && !right_only) { side = "both"; last_action = "已选择双手"; }
    else if (key >= '0' && key <= '6') {
      channel = key - '0';
      last_action = channel == 0 ? "已选择全部通道"
                                 : "已选择通道 " + std::to_string(channel);
    } else if (key == '-') {
      step = std::max(0.5, step / 2.0);
      last_action = "步长已调整";
    } else if (key == '+' || key == '=') {
      step = std::min(10.0, step * 2.0);
      last_action = "步长已调整";
    } else if (key == ' ') {
      last_action = terminal.stop() ? "已发送保持目标" : "保持目标被锁定";
    } else if (key == 'j' || key == 'k') {
      if (channel < 0) {
        last_action = "请先按 1–6 选择通道，或按 0 选择全部";
        last_draw = std::chrono::steady_clock::time_point{};
        continue;
      }
      const auto key_time = std::chrono::steady_clock::now();
      if (has_last_motion &&
          key_time - last_motion < std::chrono::milliseconds(250)) continue;
      has_last_motion = true;
      last_motion = key_time;
      const double delta = key == 'k' ? step : -step;
      last_action = terminal.nudge(side, channel, delta)
                        ? "已发送目标，请观察实测位置"
                        : "指令被锁定或已到范围边界";
    }
    last_draw = std::chrono::steady_clock::time_point{};
  }
}

}  // namespace

int main(int argc, char** argv) {
  bool keyboard_mode = true;
  bool right_only = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      std::cout << "用法: " << argv[0] << " [--commands] [--right-only]\n";
      printHelp();
      return 0;
    }
    if (arg == "--commands") keyboard_mode = false;
    else if (arg == "--right-only") right_only = true;
    else {
      std::cerr << "Usage: " << argv[0]
                << " [--commands] [--right-only]\n";
      return 2;
    }
  }

  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  const auto robot_version = leju::RobotVersion::from_env();
  if (!leju::GlobalRobot::init_env(robot_version)) {
    std::cerr << "Failed to initialize lejusdk-lowlevel. Check ROBOT_VERSION.\n";
    return 1;
  }

  HandTerminal terminal(right_only);
  auto& robot = leju::GlobalRobot::getInstance();
  robot.subscribeHardwareState(
      [&terminal](const leju::StringDataConstPtr& message) {
        terminal.onHardwareState(message);
      });
  robot.subscribeHandState(
      [&terminal](const leju::HandStateConstPtr& message) {
        terminal.onHandState(message);
      });

  std::cout << "Revo2 灵巧手终端控制\n"
            << "等待硬件及手部反馈，最多等待 10 秒"
            << std::flush;
  for (int i = 0; i < 20 && g_running && !terminal.ready(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << '.' << std::flush;
  }
  std::cout << "\n";
  if (keyboard_mode) {
    runKeyboardMode(terminal, right_only);
    leju::GlobalRobot::getInstance().shutdown();
    std::cout << "本程序已退出。\n";
    return 0;
  }

  terminal.printStatus();
  printHelp();

  std::string line;
  while (g_running) {
    std::cout << "hand> " << std::flush;
    if (!std::getline(std::cin, line)) break;
    std::istringstream input(line);
    std::string side;
    input >> side;
    side = lower(side);

    if (side.empty()) continue;
    if (side == "quit" || side == "exit" || side == "q") break;
    if (side == "help" || side == "h") {
      printHelp();
      continue;
    }
    if (side == "status") {
      terminal.printStatus();
      continue;
    }
    if (side == "stop") {
      terminal.stop();
      continue;
    }
    if (side != "left" && side != "right" && side != "both") {
      std::cerr << "Unknown command. Type help.\n";
      continue;
    }

    std::string action;
    input >> action;
    action = lower(action);
    if (action == "open") {
      terminal.setHand(side, 0.0);
    } else if (action == "close") {
      terminal.setHand(side, 100.0);
    } else if (action == "set") {
      double value = 0.0;
      if (parseValue(input, value)) terminal.setHand(side, value);
    } else if (action == "finger" && side != "both") {
      int finger = 0;
      double value = 0.0;
      if (!(input >> finger)) {
        std::cerr << "Missing finger number (1-6).\n";
      } else if (parseValue(input, value)) {
        terminal.setFinger(side, finger, value);
      }
    } else {
      std::cerr << "Unknown action. Type help.\n";
    }
  }

  leju::GlobalRobot::getInstance().shutdown();
  std::cout << "本程序已退出。\n";
  return 0;
}
