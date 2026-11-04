// Right-hand-only Revo 2 terminal controller for classic CAN or CAN FD.
// C ABI declarations match BrainCo's published stark-sdk.h (SDK 2.x).
// https://brainco.tech/docs/revolimb-hand/en/revo2/c_sdk.html

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include <csignal>
#include <termios.h>
#include <unistd.h>

extern "C" {
struct DeviceHandler;
struct CDeviceInfo {
  uint8_t sku_type;
  uint8_t hand_type;
  uint8_t hardware_type;
  const char* serial_number;
  const char* firmware_version;
  const char* hardware_version;
};
struct CMotorStatusData {
  uint16_t positions[6];
  int16_t speeds[6];
  int16_t currents[6];
  uint8_t states[6];
};

int init_socketcan_can(const char* iface);
int init_socketcan_canfd(const char* iface);
void close_socketcan();
DeviceHandler* init_device_handler_can(uint8_t protocol_type, uint8_t master_id,
                                       uint32_t arb_baudrate, uint32_t data_baudrate);
void close_device_handler(DeviceHandler* handle, uint8_t protocol_type);
CDeviceInfo* stark_get_device_info(DeviceHandler* handle, uint8_t slave_id);
void free_device_info(CDeviceInfo* info);
CMotorStatusData* stark_get_motor_status(DeviceHandler* handle, uint8_t slave_id);
void free_motor_status_data(CMotorStatusData* data);
void stark_set_finger_position(DeviceHandler* handle, uint8_t slave_id,
                               uint8_t finger_id, uint16_t position);
void stark_set_finger_positions(DeviceHandler* handle, uint8_t slave_id,
                                const uint16_t* positions, uintptr_t len);
}

namespace {
constexpr uint8_t kCan = 2;
constexpr uint8_t kCanFd = 3;
constexpr std::array<const char*, 6> kNames = {
    "thumb", "thumb aux", "index", "middle", "ring", "pinky"};
std::atomic<bool> g_running{true};

void onSignal(int) { g_running = false; }

bool parseId(const std::string& text, uint8_t& value) {
  try {
    size_t end = 0;
    const unsigned long parsed = std::stoul(text, &end, 0);
    if (end != text.size() || parsed < 1 || parsed > 254) return false;
    value = static_cast<uint8_t>(parsed);
    return true;
  } catch (...) {
    return false;
  }
}

bool validIface(const std::string& name) {
  if (name.empty() || name.size() > 15) return false;
  for (char c : name) {
    if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
          (c >= '0' && c <= '9') || c == '_' || c == '-')) return false;
  }
  return true;
}

class RawTerminal {
 public:
  RawTerminal() {
    if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &saved_) != 0) return;
    termios mode = saved_;
    mode.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    mode.c_cc[VMIN] = 0;
    mode.c_cc[VTIME] = 1;
    active_ = tcsetattr(STDIN_FILENO, TCSANOW, &mode) == 0;
  }
  ~RawTerminal() {
    if (active_) tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
  }
  bool active() const { return active_; }
 private:
  termios saved_{};
  bool active_{false};
};

class HandConnection {
 public:
  ~HandConnection() {
    if (handle_) close_device_handler(handle_, protocol_);
    if (socket_open_) close_socketcan();
  }

  bool open(const std::string& iface, uint8_t protocol,
            const std::array<uint8_t, 2>& ids,
            size_t id_count, bool inspect_only) {
    protocol_ = protocol;
    std::ifstream mtu_file("/sys/class/net/" + iface + "/mtu");
    int mtu = 0;
    if (!(mtu_file >> mtu)) {
      std::cerr << "Cannot read CAN interface " << iface << ".\n";
      return false;
    }
    const int expected_mtu = protocol == kCanFd ? 72 : 16;
    if (mtu != expected_mtu) {
      std::cerr << iface << " has MTU " << mtu
                << "; selected protocol needs MTU " << expected_mtu
                << ". No hand command was sent.\n";
      return false;
    }
    const int opened = protocol == kCanFd
                           ? init_socketcan_canfd(iface.c_str())
                           : init_socketcan_can(iface.c_str());
    if (opened != 0) {
      std::cerr << "BrainCo SDK could not open " << iface << " in "
                << (protocol == kCanFd ? "CAN FD" : "classic CAN") << " mode.\n";
      return false;
    }
    socket_open_ = true;
    // Match BrainCo's built-in SocketCAN example: master ID 1, 1 Mbps
    // arbitration, and 1/5 Mbps data for classic CAN / CAN FD.
    handle_ = init_device_handler_can(
        protocol_, 1, 1000000, protocol_ == kCanFd ? 5000000 : 1000000);
    if (!handle_) {
      std::cerr << "BrainCo SDK could not create a hand handler.\n";
      return false;
    }

    for (size_t i = 0; i < id_count; ++i) {
      const uint8_t candidate = ids[i];
      std::cout << "Reading hand information at ID "
                << static_cast<unsigned>(candidate) << "...\n" << std::flush;
      CDeviceInfo* info = stark_get_device_info(handle_, candidate);
      if (!info) continue;
      std::cout << "ID " << static_cast<unsigned>(candidate)
                << " device info: sku_type=" << static_cast<unsigned>(info->sku_type)
                << " hand_type=" << static_cast<unsigned>(info->hand_type)
                << " hardware_type=" << static_cast<unsigned>(info->hardware_type)
                << ".\n";
      if (inspect_only) {
        free_device_info(info);
        return true;
      }
      const bool right = info->hand_type == 1;
      const bool revo2 = info->hardware_type >= 10 && info->hardware_type <= 19;
      if (right && revo2) {
        id_ = candidate;
        std::cout << "Connected to RIGHT Revo 2 hand, ID "
                  << static_cast<unsigned>(id_) << ".\n";
      } else {
        std::cerr << "ID " << static_cast<unsigned>(candidate)
                  << " replied, but is not identified as a right Revo 2 hand.\n";
      }
      free_device_info(info);
      if (id_ != 0) break;
    }
    if (id_ == 0) {
      std::cerr << "No right Revo 2 hand answered. No motion command was sent.\n";
      return false;
    }
    std::array<uint16_t, 6> measured{};
    if (!readPositions(measured)) {
      std::cerr << "Right-hand position feedback unavailable. Motion locked.\n";
      return false;
    }
    return true;
  }

  bool readPositions(std::array<uint16_t, 6>& positions) const {
    CMotorStatusData* status = stark_get_motor_status(handle_, id_);
    if (!status) return false;
    bool valid = true;
    for (size_t i = 0; i < positions.size(); ++i) {
      positions[i] = status->positions[i];
      valid = valid && positions[i] <= 1000;
    }
    free_motor_status_data(status);
    return valid;
  }

  void sendOne(int channel, uint16_t value) const {
    stark_set_finger_position(handle_, id_, static_cast<uint8_t>(channel), value);
  }
  void sendAll(const std::array<uint16_t, 6>& values) const {
    stark_set_finger_positions(handle_, id_, values.data(), values.size());
  }

 private:
  DeviceHandler* handle_{nullptr};
  bool socket_open_{false};
  uint8_t protocol_{kCanFd};
  uint8_t id_{0};
};

void show(const HandConnection& hand, int channel, int step) {
  std::array<uint16_t, 6> current{};
  if (!hand.readPositions(current)) {
    std::cout << "\nNo fresh right-hand feedback; motion keys will be blocked.\n";
  } else {
    std::cout << "\nRight hand measured positions (0=open, 1000=closed):\n";
    for (size_t i = 0; i < current.size(); ++i) {
      std::cout << "  " << (i + 1) << " " << std::setw(9) << std::left << kNames[i]
                << " " << current[i] << "\n";
    }
  }
  const std::string selected = channel < 0
                                  ? "none (select 0 or 1-6 first)"
                                  : channel == 0 ? "all six channels" : kNames[channel - 1];
  std::cout << "Selected: " << selected
            << " | step: " << step << "/1000\n"
            << "Keys: 0=all, 1-6=select, j=open a step, k=close a step,\n"
            << "      -=smaller step, +=larger step, Space=hold, r=read, q=quit\n"
            << std::flush;
}
}  // namespace

int main(int argc, char** argv) {
  std::string iface;
  uint8_t protocol = kCanFd;
  bool inspect_only = false;
  std::array<uint8_t, 2> ids{0x7f, 0x02};
  size_t id_count = 2;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--iface" && i + 1 < argc) {
      iface = argv[++i];
    } else if (arg == "--protocol" && i + 1 < argc) {
      const std::string value = argv[++i];
      if (value == "can") protocol = kCan;
      else if (value == "canfd") protocol = kCanFd;
      else {
        std::cerr << "Protocol must be can or canfd.\n";
        return 2;
      }
    } else if (arg == "--id" && i + 1 < argc) {
      if (!parseId(argv[++i], ids[0])) {
        std::cerr << "ID must be 1..254 (decimal or 0x hex).\n";
        return 2;
      }
      id_count = 1;
    } else if (arg == "--inspect") {
      inspect_only = true;
    } else {
      std::cerr << "Usage: " << argv[0]
                << " --iface <CAN interface> [--protocol can|canfd]"
                   " [--id 0x7f] [--inspect]\n";
      return 2;
    }
  }
  if (!validIface(iface)) {
    std::cerr << "Specify a Linux CAN interface with --iface, e.g. --iface bcan3.\n";
    return 2;
  }
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  HandConnection hand;
  if (!hand.open(iface, protocol, ids, id_count, inspect_only)) return 1;
  if (inspect_only) return 0;
  RawTerminal terminal;
  if (!terminal.active()) {
    std::cerr << "Open an interactive SSH terminal to use the keyboard.\n";
    return 1;
  }

  // Require an explicit selection before any movement key can send a command.
  int channel = -1;
  int step = 10;
  auto last_motion = std::chrono::steady_clock::time_point{};
  show(hand, channel, step);
  while (g_running) {
    char key = 0;
    const ssize_t count = read(STDIN_FILENO, &key, 1);
    if (count == 0) continue;
    if (count < 0) {
      if (errno == EINTR) continue;
      std::cerr << "Terminal input failed.\n";
      break;
    }
    if (key == 'q' || key == 'Q') break;
    if (key >= '0' && key <= '6') {
      channel = key - '0';
    } else if (key == '-') {
      step = std::max(1, step / 2);
    } else if (key == '+' || key == '=') {
      step = std::min(100, step * 2);
    } else if (key == 'j' || key == 'k' || key == ' ') {
      if (channel < 0) {
        std::cout << "Select one channel (1-6) or all channels (0) before moving.\n";
        show(hand, channel, step);
        continue;
      }
      if (key != ' ') {
        const auto now = std::chrono::steady_clock::now();
        if (last_motion.time_since_epoch().count() != 0 &&
            now - last_motion < std::chrono::milliseconds(250)) {
          continue;
        }
        last_motion = now;
      }
      std::array<uint16_t, 6> target{};
      if (!hand.readPositions(target)) {
        std::cout << "Feedback missing; command blocked.\n";
        show(hand, channel, step);
        continue;
      }
      if (key != ' ') {
        const int delta = key == 'k' ? step : -step;
        const auto change = [&](int index) {
          const int next = static_cast<int>(target[index]) + delta;
          target[index] = static_cast<uint16_t>(std::max(0, std::min(1000, next)));
        };
        if (channel == 0) {
          for (int i = 0; i < 6; ++i) change(i);
        } else {
          change(channel - 1);
        }
      }
      if (channel == 0 || key == ' ') hand.sendAll(target);
      else hand.sendOne(channel, target[channel - 1]);
      std::cout << "Target sent; reading measured positions...\n";
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    } else if (key != 'r' && key != 'R') {
      continue;
    }
    show(hand, channel, step);
  }
  std::cout << "\nController closed.\n";
  return 0;
}
