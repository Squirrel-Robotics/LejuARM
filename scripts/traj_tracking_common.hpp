#pragma once

// 参考 scripts/arm_traj_tracking_test.cpp；只发布头部/腰部轨迹，不发布全身 JointCmd。
// 当前 Roban 接收端：head q=[yaw,pitch] 直接消费；waist q 在 policy 坐标中消费。
// 腰 SDK/policy 坐标转换读取当前 AMP 配置的 joint_direction，不能直接套用头部路径。

#include <lejusdk-vr/lejusdk_vr.h>
#include <lejusdk-lowlevel/leju_sdk.h>
#include <lejusdk-dds-idl/JointCmd.hpp>
#include <lejusdk-dds-idl/JointState.hpp>
#include <lejusdk-topic-pubsub/topic_names.h>
#include <lejusdk-topic-pubsub/topic_subscriber.hpp>
#include <dds/dds.hpp>
#include <yaml-cpp/yaml.h>
#include "tracking_rpc.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

namespace roban_tracking {
namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;
using leju::vr::ControlMode;
using Vec = std::array<double, 2>;
constexpr double kPi = 3.14159265358979323846;
constexpr double kRad2Deg = 180.0 / kPi;
inline volatile sig_atomic_t stop_requested = 0;
inline void onSignal(int) { stop_requested = 1; }

enum class Part { Head, Waist };

struct Spec {
  Part part;
  std::string stem;
  std::size_t dof;
  std::array<int, 2> indices;
  std::array<std::string, 2> names;
  Vec lower_deg;
  Vec upper_deg;
  const char* trajectory_topic;
};

inline Spec specFor(Part part) {
  if (part == Part::Head) {
    return {part, "head", 2, {21, 22}, {"zhead_1_joint", "zhead_2_joint"},
            {-90, -20}, {90, 45}, leju::dds_topics::kHeadTrajectory};
  }
  return {part, "waist", 1, {12, 0}, {"waist_yaw_joint", ""},
          {-150, 0}, {150, 0}, leju::dds_topics::kWaistTrajectory};
}

struct Options {
  fs::path config;
  fs::path project;
  fs::path manager;
  Vec delta_deg{};
  Vec target_deg{};
  bool absolute_target = false;
  double move_sec = 2;
  int hold_ms = 2000;
  int skip_ms = 700;
  int settle_ms = 1000;
  double rate_hz = 100;
  double max_velocity_deg_s = 15;
  int feedback_timeout_ms = 500;
  double max_tracking_error_deg = 15;
  double target_tolerance_deg = 2;
  bool dry_run = false;
  bool inspect = false;
  bool preflight = false;
};

inline void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

// 固定本测试进程的网络通信配置；RPC 样本内存缺陷由 tracking_rpc.hpp 单独修复。
// 网络参数与本台下位机已验证的 roban2-gamepad/dds-network.xml 相同。
// 必须在首次构造 participant、调用 GlobalRobot::init_env 或 VR API initialize 之前执行。
inline constexpr const char* kTrackingDDSConfig = R"xml(<CycloneDDS>
  <Domain Id="any">
    <SharedMemory><Enable>false</Enable></SharedMemory>
    <General>
      <Interfaces><NetworkInterface address="192.168.26.1" multicast="default"/></Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery><EnableTopicDiscoveryEndpoints>true</EnableTopicDiscoveryEndpoints></Discovery>
  </Domain>
</CycloneDDS>)xml";

inline void configureTrackingDDS() {
  require(setenv("CYCLONEDDS_URI", kTrackingDDSConfig, 1) == 0, "设置测试 DDS 网络配置失败");
  std::cout << "DDS: SharedMemory=false，接口=192.168.26.1（本测试进程固定配置）\n";
}

inline Options loadOptions(const Spec& s, int argc, char** argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--dry-run") o.dry_run = true;
    else if (arg == "--inspect") o.inspect = true;
    else if (arg == "--preflight") o.preflight = true;
    else if (arg == "--config") {
      require(i + 1 < argc, "--config 缺少路径");
      o.config = fs::absolute(argv[++i]);
    } else throw std::runtime_error("未知参数: " + arg);
  }
  require(static_cast<int>(o.dry_run) + static_cast<int>(o.inspect) + static_cast<int>(o.preflight) <= 1,
          "--dry-run、--inspect 和 --preflight 请分开使用");
  const fs::path exe = fs::read_symlink("/proc/self/exe");
  const std::string filename = s.stem + "_traj_tracking.yaml";
  // 优先读取项目 scripts，避免 build 中的旧 YAML 掩盖用户修改。
  const std::vector<fs::path> candidates = {
      fs::current_path() / "scripts" / filename,
      exe.parent_path().parent_path() / "scripts" / filename,
      exe.parent_path() / filename};
  if (o.config.empty()) {
    for (const auto& candidate : candidates) {
      if (fs::is_regular_file(candidate)) { o.config = candidate; break; }
    }
  }
  require(!o.config.empty() && fs::is_regular_file(o.config),
          "找不到 scripts/" + filename + "；可用 --config PATH 指定");
  o.config = fs::canonical(o.config);
  o.project = exe.parent_path().parent_path();
  if (!fs::is_directory(o.project / "installed")) {
    o.project = o.config.parent_path().parent_path();
  }
  const YAML::Node root = YAML::LoadFile(o.config.string());
  require(root.IsMap(), "测试 YAML 必须是键值映射");
  const std::vector<std::string> keys = {"delta_deg", "target_deg", "move_sec", "hold_ms", "skip_ms",
      "settle_ms", "rate_hz", "max_velocity_deg_s", "feedback_timeout_ms",
      "max_tracking_error_deg", "target_tolerance_deg", "controller_manager_config"};
  for (const auto& entry : root) {
    const std::string key = entry.first.as<std::string>();
    require(std::find(keys.begin(), keys.end(), key) != keys.end(), "未知 YAML 字段: " + key);
  }
  require(static_cast<bool>(root["delta_deg"]) != static_cast<bool>(root["target_deg"]),
          "target_deg（绝对目标）和 delta_deg（相对步进）必须且只能填写一项");
  o.absolute_target = static_cast<bool>(root["target_deg"]);
  const std::string angle_key = o.absolute_target ? "target_deg" : "delta_deg";
  const YAML::Node angles = root[angle_key];
  require(angles.IsSequence() && angles.size() == s.dof,
          angle_key + " 必须包含 " + std::to_string(s.dof) + " 个角度（deg）");
  for (std::size_t i = 0; i < s.dof; ++i) {
    const double value = angles[i].as<double>();
    require(std::isfinite(value), angle_key + " 必须是有限值");
    if (o.absolute_target) {
      require(value >= s.lower_deg[i] && value <= s.upper_deg[i], s.names[i] + " target_deg 超出模型限位");
      o.target_deg[i] = value;
    } else {
      require(std::abs(value) <= 60, "单次相对步进绝对值不得超过 60 deg");
      o.delta_deg[i] = value;
    }
  }
  if (root["move_sec"]) o.move_sec = root["move_sec"].as<double>();
  if (root["hold_ms"]) o.hold_ms = root["hold_ms"].as<int>();
  if (root["skip_ms"]) o.skip_ms = root["skip_ms"].as<int>();
  if (root["settle_ms"]) o.settle_ms = root["settle_ms"].as<int>();
  if (root["rate_hz"]) o.rate_hz = root["rate_hz"].as<double>();
  if (root["max_velocity_deg_s"]) o.max_velocity_deg_s = root["max_velocity_deg_s"].as<double>();
  if (root["feedback_timeout_ms"]) o.feedback_timeout_ms = root["feedback_timeout_ms"].as<int>();
  if (root["max_tracking_error_deg"]) o.max_tracking_error_deg = root["max_tracking_error_deg"].as<double>();
  if (root["target_tolerance_deg"]) o.target_tolerance_deg = root["target_tolerance_deg"].as<double>();
  o.manager = root["controller_manager_config"]
      ? fs::path(root["controller_manager_config"].as<std::string>())
      : fs::path("src/leju-controllers/leju-rl-controller/config/17/controller_manager.yaml");
  if (o.manager.is_relative()) o.manager = o.project / o.manager;
  require(std::isfinite(o.move_sec) && o.move_sec >= 0.5 && o.move_sec <= 30,
          "move_sec 范围为 0.5..30 s");
  require(o.hold_ms >= 500 && o.hold_ms <= 60000 && o.skip_ms >= 0 && o.skip_ms < o.hold_ms,
          "hold_ms 范围 500..60000，且 0 <= skip_ms < hold_ms");
  require(o.settle_ms >= 1000 && o.settle_ms <= 10000, "settle_ms 范围 1000..10000");
  require(std::isfinite(o.rate_hz) && o.rate_hz >= 20 && o.rate_hz <= 250, "rate_hz 范围 20..250 Hz");
  require(std::isfinite(o.max_velocity_deg_s) && o.max_velocity_deg_s > 0 && o.max_velocity_deg_s <= 30,
          "max_velocity_deg_s 范围 (0,30] deg/s");
  require(o.feedback_timeout_ms >= 100 && o.feedback_timeout_ms <= 500, "feedback_timeout_ms 范围 100..500");
  require(std::isfinite(o.max_tracking_error_deg) && o.max_tracking_error_deg > 0 &&
          o.max_tracking_error_deg <= 30, "max_tracking_error_deg 范围 (0,30] deg");
  require(std::isfinite(o.target_tolerance_deg) && o.target_tolerance_deg >= 0.1 &&
          o.target_tolerance_deg <= 5, "target_tolerance_deg 范围 0.1..5 deg");
  return o;
}

inline Vec prepareTarget(const Spec& s, Options& o, const Vec& origin) {
  Vec target{};
  const double configured_sec = o.move_sec;
  for (std::size_t i = 0; i < s.dof; ++i) {
    target[i] = o.absolute_target ? o.target_deg[i] / kRad2Deg : origin[i] + o.delta_deg[i] / kRad2Deg;
    // 更新实际位移，供结果验收；absolute 模式下位移由启动反馈决定。
    o.delta_deg[i] = (target[i] - origin[i]) * kRad2Deg;
    const double required_sec = std::abs(o.delta_deg[i]) * kPi / (2 * o.max_velocity_deg_s);
    o.move_sec = std::max(o.move_sec, required_sec);
  }
  if (o.move_sec > configured_sec) {
    std::cout << "按当前位置与 " << o.max_velocity_deg_s << " deg/s 限速，移动时间延长为 "
              << o.move_sec << " s\n";
  }
  return target;
}

struct Snapshot {
  Vec q{};
  Vec kp{{NAN, NAN}};
  Vec kd{{NAN, NAN}};
  std::array<int, 2> modes{{-1, -1}};
  Clock::time_point received{};
  std::uint64_t seq = 0;
};

inline bool fresh(const Snapshot& snap, int timeout_ms) {
  return snap.seq != 0 && Clock::now() - snap.received < std::chrono::milliseconds(timeout_ms);
}

struct Shared {
  std::mutex mu;
  Snapshot joint_cmd, motor_cmd, joint_state, motor_state;
  std::array<double, 23> body_q{}, body_v{};
  Clock::time_point body_received{};
  std::uint64_t body_seq = 0;
};

// 已安装 TopicSubscriber 的析构未解除 reader 的裸 listener 指针。
// 在底层 listener 仍存活时显式停止回调并关闭 reader，覆盖正常/异常退出。
template <typename T>
class SafeSubscriber : public leju::dds_common::TopicSubscriber<T> {
 public:
  using Base = leju::dds_common::TopicSubscriber<T>;
  using Base::Base;
  ~SafeSubscriber() {
    try {
      if (this->is_valid()) {
        this->reader().listener(nullptr, dds::core::status::StatusMask::none());
        this->reader().close();
      }
    } catch (const std::exception& e) {
      std::cerr << "关闭 DDS 订阅异常: " << e.what() << '\n';
    }
  }
};

inline bool extract(const Spec& s, const std::vector<double>& src, Vec& out) {
  if (src.size() != 23) return false;
  for (std::size_t i = 0; i < s.dof; ++i) {
    out[i] = src[s.indices[i]];
    if (!std::isfinite(out[i])) return false;
  }
  return true;
}

inline void onState(Shared& shared, Snapshot Shared::* slot, const Spec& s,
                    const leju::msgs::JointState& msg) {
  Snapshot snap;
  if (!extract(s, msg.q(), snap.q)) return;
  snap.received = Clock::now();
  std::lock_guard<std::mutex> lock(shared.mu);
  snap.seq = (shared.*slot).seq + 1;
  shared.*slot = snap;
  if (slot == &Shared::joint_state && msg.q().size() == 23 && msg.v().size() == 23 &&
      std::all_of(msg.q().begin(), msg.q().end(), [](double x) { return std::isfinite(x); }) &&
      std::all_of(msg.v().begin(), msg.v().end(), [](double x) { return std::isfinite(x); })) {
    std::copy(msg.q().begin(), msg.q().end(), shared.body_q.begin());
    std::copy(msg.v().begin(), msg.v().end(), shared.body_v.begin());
    shared.body_received = snap.received;
    ++shared.body_seq;
  }
}

inline void onCmd(Shared& shared, Snapshot Shared::* slot, const Spec& s,
                  const leju::msgs::JointCmd& msg) {
  Snapshot snap;
  if (!extract(s, msg.q(), snap.q)) return;
  // 缺少增益时报告 '-'，不把缺失字段统计为 0。
  Vec values{};
  if (extract(s, msg.kp(), values)) snap.kp = values;
  if (extract(s, msg.kd(), values)) snap.kd = values;
  if (msg.modes().size() == 23) {
    for (std::size_t i = 0; i < s.dof; ++i) snap.modes[i] = msg.modes()[s.indices[i]];
  }
  snap.received = Clock::now();
  std::lock_guard<std::mutex> lock(shared.mu);
  snap.seq = (shared.*slot).seq + 1;
  shared.*slot = snap;
}

struct Frame { Snapshot cmd, state, joint_state, joint_cmd; };

inline Frame frame(Shared& shared, bool motor) {
  std::lock_guard<std::mutex> lock(shared.mu);
  return {motor ? shared.motor_cmd : shared.joint_cmd,
          motor ? shared.motor_state : shared.joint_state, shared.joint_state, shared.joint_cmd};
}

inline bool waitStreams(Shared& shared, int timeout_ms, bool* motor) {
  const auto end = Clock::now() + std::chrono::seconds(3);
  while (Clock::now() < end && !stop_requested) {
    const auto m = frame(shared, true);
    if (fresh(m.joint_state, timeout_ms) && fresh(m.joint_cmd, timeout_ms) &&
        fresh(m.cmd, timeout_ms) && fresh(m.state, timeout_ms)) {
      *motor = true; return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  const auto j = frame(shared, false);
  *motor = false;
  return !stop_requested && fresh(j.cmd, timeout_ms) && fresh(j.state, timeout_ms);
}

inline void checkAngles(const Spec& s, const Vec& sdk_q, const char* stage) {
  for (std::size_t i = 0; i < s.dof; ++i) {
    const double deg = sdk_q[i] * kRad2Deg;
    require(std::isfinite(deg) && deg >= s.lower_deg[i] && deg <= s.upper_deg[i],
            std::string(stage) + " " + s.names[i] + "=" + std::to_string(deg) +
            " deg 超出模型限位 [" + std::to_string(s.lower_deg[i]) + "," +
            std::to_string(s.upper_deg[i]) + "]，未发布新轨迹");
  }
}

// 以下解析与匹配函数只处理输入数据，便于离线测试，均不接触 DDS 或模式接口。
inline bool parseProcUnsigned(const std::string& text, std::uint64_t* value) {
  if (text.empty()) return false;
  std::uint64_t result = 0;
  for (const char character : text) {
    if (character < '0' || character > '9') return false;
    const auto digit = static_cast<std::uint64_t>(character - '0');
    if (result > (std::numeric_limits<std::uint64_t>::max() - digit) / 10) return false;
    result = result * 10 + digit;
  }
  *value = result;
  return true;
}

struct ProcStart {
  std::uint64_t pid = 0;
  std::uint64_t ticks = 0;
};

inline bool parseProcStart(const std::string& text, ProcStart* result) {
  // comm 可以包含空格或括号，不能直接按空格拆整条 /proc/PID/stat。
  const auto open = text.find('(');
  const auto close = text.rfind(')');
  if (open == std::string::npos || close == std::string::npos || close <= open) return false;
  std::istringstream prefix(text.substr(0, open));
  std::string pid, extra;
  if (!(prefix >> pid) || (prefix >> extra)) return false;
  ProcStart parsed;
  if (!parseProcUnsigned(pid, &parsed.pid) || parsed.pid == 0) return false;
  std::istringstream suffix(text.substr(close + 1));
  std::string field;
  for (int index = 3; index <= 22; ++index) {
    if (!(suffix >> field)) return false;
    if (index == 3 && field.size() != 1) return false;
  }
  if (!parseProcUnsigned(field, &parsed.ticks)) return false;
  *result = parsed;
  return true;
}

inline bool parseProcBootTime(const std::string& text, std::uint64_t* boot_time) {
  std::istringstream lines(text);
  std::string line;
  bool found = false;
  std::uint64_t parsed = 0;
  while (std::getline(lines, line)) {
    std::istringstream fields(line);
    std::string key, value, extra;
    if (!(fields >> key) || key != "btime") continue;
    if (found || !(fields >> value) || (fields >> extra) ||
        !parseProcUnsigned(value, &parsed) || parsed == 0) return false;
    found = true;
  }
  if (found) *boot_time = parsed;
  return found;
}

inline std::vector<std::string> parseProcCmdline(const std::string& text) {
  std::vector<std::string> arguments;
  std::size_t begin = 0;
  while (begin < text.size()) {
    const auto end = text.find('\0', begin);
    arguments.push_back(text.substr(begin, end == std::string::npos ? end : end - begin));
    if (end == std::string::npos) break;
    begin = end + 1;
  }
  return arguments;
}

inline bool pathWithinProject(const fs::path& path, const fs::path& project) {
  const auto normalized_path = path.lexically_normal();
  const auto normalized_project = project.lexically_normal();
  if (!normalized_path.is_absolute() || !normalized_project.is_absolute()) return false;
  auto position = normalized_path.begin();
  for (const auto& component : normalized_project) {
    if (position == normalized_path.end() || *position != component) return false;
    ++position;
  }
  return true;
}

inline bool matchesControllerCommand(const fs::path& executable,
                                     const std::vector<std::string>& arguments,
                                     const fs::path& cwd, const fs::path& project,
                                     const fs::path& manager) {
  if (executable.filename() != "run_rl_controller" || !pathWithinProject(executable, project) ||
      !pathWithinProject(manager, project)) return false;
  fs::path process_manager;
  bool found = false;
  for (std::size_t index = 1; index < arguments.size(); ++index) {
    if (arguments[index] != "-c") continue;
    if (found || index + 1 >= arguments.size() || arguments[index + 1].empty()) return false;
    process_manager = arguments[++index];
    found = true;
  }
  if (!found) return false;
  if (process_manager.is_relative()) {
    if (!cwd.is_absolute()) return false;
    process_manager = cwd / process_manager;
  }
  return pathWithinProject(process_manager, project) &&
      process_manager.lexically_normal() == manager.lexically_normal();
}

inline bool matchesControllerProcess(const fs::path& executable,
                                     const std::vector<std::string>& arguments,
                                     const fs::path& cwd, const fs::path& project,
                                     const fs::path& manager,
                                     const std::string& own_network_namespace,
                                     const std::string& process_network_namespace) {
  return !own_network_namespace.empty() && own_network_namespace == process_network_namespace &&
      matchesControllerCommand(executable, arguments, cwd, project, manager);
}

enum class ConfigProcessRelation { Unknown, PredatesProcess, ModifiedAfterStart };

inline ConfigProcessRelation compareConfigModification(std::int64_t mtime_seconds,
    long mtime_nanoseconds, std::uint64_t boot_time, std::uint64_t start_ticks,
    long ticks_per_second) {
  if (mtime_seconds < 0 || mtime_nanoseconds < 0 || mtime_nanoseconds >= 1000000000L ||
      boot_time == 0 || ticks_per_second <= 0) return ConfigProcessRelation::Unknown;
  const long double started = static_cast<long double>(boot_time) +
      static_cast<long double>(start_ticks) / ticks_per_second;
  const long double modified = static_cast<long double>(mtime_seconds) +
      static_cast<long double>(mtime_nanoseconds) / 1000000000L;
  return modified > started ? ConfigProcessRelation::ModifiedAfterStart :
      ConfigProcessRelation::PredatesProcess;
}

inline bool readProcFile(const fs::path& path, std::string* text) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) return false;
  std::ostringstream contents;
  contents << stream.rdbuf();
  if (stream.bad()) return false;
  *text = contents.str();
  return true;
}

inline fs::path normalizedExistingPath(const fs::path& path) {
  if (path.empty()) return {};
  std::error_code error;
  const auto result = fs::weakly_canonical(path, error);
  return error ? path.lexically_normal() : result;
}

inline void checkWaistConfigProcess(const Options& o, const fs::path& config) {
  // 同一个项目可能在隔离网络命名空间中运行离线模拟，不能把宿主控制器算进去。
  std::error_code error;
  const auto own_namespace = fs::read_symlink("/proc/self/ns/net", error).string();
  std::string proc_stat;
  std::uint64_t boot_time = 0;
  struct stat config_stat{};
  const long ticks_per_second = sysconf(_SC_CLK_TCK);
  const auto unavailable = [](const std::string& reason) {
    std::cerr << "未验证当前控制器是否重新加载 AMP 配置：" << reason
              << "。文件中的 enable_waist_controller 值不代表运行实例已生效。\n";
  };
  if (error || own_namespace.empty() || !readProcFile("/proc/stat", &proc_stat) ||
      !parseProcBootTime(proc_stat, &boot_time) || ticks_per_second <= 0 ||
      stat(config.c_str(), &config_stat) != 0) {
    unavailable("无法读取本机进程时间、网络命名空间或 AMP 文件时间");
    return;
  }
  const auto project = normalizedExistingPath(o.project);
  const auto manager = normalizedExistingPath(o.manager);
  bool verified = false;
  bool unreadable_match = false;
  std::vector<std::uint64_t> modified_after_pids;
  bool modified_namespace_unknown = false;
  fs::directory_iterator processes("/proc", error);
  if (error) { unavailable("无法遍历 /proc"); return; }
  const fs::directory_iterator end;
  for (; processes != end; processes.increment(error)) {
    if (error) break;
    const auto process_dir = processes->path();
    std::uint64_t pid = 0;
    if (!parseProcUnsigned(process_dir.filename().string(), &pid) || pid == 0) continue;
    std::string cmdline, process_stat;
    if (!readProcFile(process_dir / "cmdline", &cmdline)) continue;
    const auto arguments = parseProcCmdline(cmdline);
    if (arguments.empty()) continue;
    std::error_code process_error;
    auto executable = fs::read_symlink(process_dir / "exe", process_error);
    // lab 账号不能 readlink root 控制器的 exe/ns，但通常能读取其 cmdline/stat。
    // 仅以项目内的绝对启动路径作为候选，不能把该回退声称为已验证加载。
    const bool executable_verified = !process_error;
    if (process_error) {
      executable = arguments.front();
      if (!executable.is_absolute()) continue;
    }
    // 内核为已被替换但仍运行的可执行文件附加此后缀。
    std::string executable_name = executable.string();
    const std::string deleted_suffix = " (deleted)";
    if (executable_name.size() >= deleted_suffix.size() &&
        executable_name.compare(executable_name.size() - deleted_suffix.size(),
                                deleted_suffix.size(), deleted_suffix) == 0) {
      executable = executable_name.substr(0, executable_name.size() - deleted_suffix.size());
    }
    executable = normalizedExistingPath(executable);
    if (executable.filename() != "run_rl_controller" || !pathWithinProject(executable, project)) continue;
    const auto network_namespace = fs::read_symlink(process_dir / "ns/net", process_error).string();
    const bool namespace_verified = !process_error;
    if (namespace_verified && network_namespace != own_namespace) continue;
    auto cwd = fs::read_symlink(process_dir / "cwd", process_error);
    if (process_error) cwd.clear();
    if (!matchesControllerCommand(executable, arguments, normalizedExistingPath(cwd), project,
                                  manager)) continue;
    if (!namespace_verified || !executable_verified) unreadable_match = true;
    ProcStart start;
    if (!readProcFile(process_dir / "stat", &process_stat) ||
        !parseProcStart(process_stat, &start) || start.pid != pid) {
      unreadable_match = true; continue;
    }
    const auto relation = compareConfigModification(config_stat.st_mtim.tv_sec,
        config_stat.st_mtim.tv_nsec, boot_time, start.ticks, ticks_per_second);
    if (relation == ConfigProcessRelation::Unknown) { unreadable_match = true; continue; }
    verified = verified || (namespace_verified && executable_verified);
    if (relation == ConfigProcessRelation::ModifiedAfterStart) {
      modified_after_pids.push_back(pid);
      modified_namespace_unknown = modified_namespace_unknown || !namespace_verified;
    }
    std::cout << "AMP 配置时间检查: PID=" << pid << "，配置"
              << (relation == ConfigProcessRelation::ModifiedAfterStart ? "晚于" : "未晚于")
              << "匹配启动命令的 run_rl_controller 启动时间"
              << (namespace_verified ? "" : "（进程网络命名空间不可读）") << '\n';
  }
  if (error) unreadable_match = true;
  if (!modified_after_pids.empty()) {
    std::ostringstream message;
    message << "AMP 配置 " << config << " 在匹配本项目启动命令的 run_rl_controller 启动后修改（PID";
    for (const auto pid : modified_after_pids) message << ' ' << pid;
    message << "）。文件值不能作为当前实例已启用腰部控制的依据。";
    if (modified_namespace_unknown) message << "部分候选进程的网络命名空间不可读，配置加载未验证。";
    message
            << "请按原有机器人初始化/START 流程重新初始化控制器，再运行腰部测试；"
            << "不要用运行时 reload 或本测试工具重启机器人";
    if (o.inspect) std::cerr << "警告：" << message.str() << "。\n";
    else throw std::runtime_error(message.str());
  } else if (!verified || unreadable_match) {
    unavailable(verified ? "部分匹配控制器的进程信息不可读" :
        "未找到同一网络命名空间中、使用本项目 manager 配置的可读 run_rl_controller 进程");
  }
}

inline bool waistConfig(const Options& o, const std::string& controller, double* direction) {
  require(controller == "amp", "腰部测试只核验了 amp，请先按原操作流程使用 amp 控制器");
  const auto manager = YAML::LoadFile(o.manager.string());
  fs::path config;
  for (const auto& entry : manager["controllers"]) {
    if (entry["name"].as<std::string>() == controller) {
      config = entry["config"].as<std::string>(); break;
    }
  }
  require(!config.empty(), "controller_manager_config 中找不到 amp 配置");
  if (config.is_relative()) config = o.manager.parent_path() / config;
  const auto robot = YAML::LoadFile(config.string())["HumanoidRobotCfg"]["env"]["robot"];
  require(robot.IsMap(), "AMP YAML 缺少 HumanoidRobotCfg.env.robot");
  const auto names = robot["joint_names"];
  const auto directions = robot["joint_direction"];
  require(names.IsSequence() && directions.IsSequence() && names.size() == directions.size(),
          "AMP joint_names/joint_direction 维度不匹配");
  bool found = false;
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (names[i].as<std::string>() == "waist_yaw_joint") {
      *direction = directions[i].as<double>(); found = true; break;
    }
  }
  require(found && (*direction == 1 || *direction == -1), "AMP 腰 joint_direction 必须为 +1 或 -1");
  const bool enabled = robot["enable_waist_controller"] && robot["enable_waist_controller"].as<bool>();
  std::cout << "AMP 配置: " << config << "\n腰 joint_direction=" << *direction
            << "，enable_waist_controller=" << std::boolalpha << enabled << '\n';
  checkWaistConfigProcess(o, config);
  return enabled;
}

// VR API 的析构不执行 shutdown。显式清理覆盖查询失败、配置拒绝、模式失败等出口。
// 此 guard 必须先于 ModeGuard 创建，确保恢复模式后才关闭 RPC 通信。
struct ApiGuard {
  leju::vr::RobanVRAPI& api;
  ~ApiGuard() noexcept {
    try { api.shutdown(); }
    catch (const std::exception& e) { std::cerr << "关闭 VR SDK 异常: " << e.what() << '\n'; }
    catch (...) { std::cerr << "关闭 VR SDK 出现未知异常。\n"; }
  }
};

struct ModeGuard {
  TrackingControlRpc& api;
  ControlMode original;
  bool armed = false;
  bool restore() {
    if (!armed) return true;
    if (!api.setWaistMode(original, 1500)) return false;
    const auto deadline = Clock::now() + std::chrono::seconds(2);
    while (Clock::now() < deadline) {
      leju::vr::ControllerState state;
      if (api.getControllerState(state, 500) && state.waist_mode == original) {
        armed = false; return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }
  ~ModeGuard() {
    if (armed) {
      try {
        if (!restore()) std::cerr << "腰部模式恢复失败，请现场检查控制模式和急停。\n";
      } catch (...) { std::cerr << "腰部模式恢复异常，请现场检查。\n"; }
    }
  }
};

struct Lock {
  int fd = -1;
  explicit Lock(const Spec& s) {
    const std::string path = "/tmp/roban_" + s.stem + "_traj_tracking.lock";
    fd = open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    require(fd >= 0, "无法创建测试锁: " + path);
    if (flock(fd, LOCK_EX | LOCK_NB) != 0) {
      close(fd); fd = -1;
      throw std::runtime_error("已有同部位测试正在运行");
    }
  }
  ~Lock() { if (fd >= 0) close(fd); }
};

struct Accum {
  int n = 0, kp_n = 0, kd_n = 0;
  double cmd = 0, fb = 0, abs_error = 0, max_error = 0, kp = 0, kd = 0;
  double expected_sdk = 0, target_error = 0;
  double target_cmd_error = 0;
  int mode = -1;
};

inline void addSample(Accum& a, const Snapshot& cmd, const Snapshot& state,
                      const Snapshot& joint_state, const Snapshot& joint_cmd,
                      std::size_t i, double expected_sdk) {
  ++a.n;
  a.cmd += cmd.q[i]; a.fb += state.q[i];
  const double error = std::abs(cmd.q[i] - state.q[i]);
  a.abs_error += error; a.max_error = std::max(a.max_error, error);
  a.expected_sdk += expected_sdk;
  a.target_error += std::abs(expected_sdk - joint_state.q[i]);
  a.target_cmd_error += std::abs(expected_sdk - joint_cmd.q[i]);
  if (std::isfinite(cmd.kp[i])) { ++a.kp_n; a.kp += cmd.kp[i]; }
  if (std::isfinite(cmd.kd[i])) { ++a.kd_n; a.kd += cmd.kd[i]; }
  a.mode = cmd.modes[i];
}

inline void report(const Spec& s, bool motor, const std::array<Accum, 2>& stats) {
  std::cout << "\n执行层稳态误差: " << (motor ? "/rt/motor_cmd − /rt/motor_state" : "/rt/joint_cmd − /rt/joint_state")
            << "（角度为 SDK/电机坐标；误差正值表示指令角大于反馈角）\n";
  std::cout << std::left << std::setw(20) << "joint" << std::right
            << std::setw(10) << "cmd_deg" << std::setw(10) << "fb_deg"
            << std::setw(10) << "err_deg" << std::setw(10) << "abs_deg"
            << std::setw(10) << "max_deg" << std::setw(10) << "kp"
            << std::setw(10) << "kd" << std::setw(7) << "mode" << std::setw(8) << "samples" << '\n';
  for (std::size_t i = 0; i < s.dof; ++i) {
    const auto& a = stats[i];
    require(a.n > 0, "保持段未取得新的有效指令/反馈样本，不能认定测试成功");
    std::cout << std::left << std::setw(20) << s.names[i] << std::right << std::fixed << std::setprecision(3)
              << std::setw(10) << a.cmd / a.n * kRad2Deg << std::setw(10) << a.fb / a.n * kRad2Deg
              << std::setw(10) << (a.cmd - a.fb) / a.n * kRad2Deg
              << std::setw(10) << a.abs_error / a.n * kRad2Deg
              << std::setw(10) << a.max_error * kRad2Deg;
    if (a.kp_n) std::cout << std::setw(10) << a.kp / a.kp_n; else std::cout << std::setw(10) << "-";
    if (a.kd_n) std::cout << std::setw(10) << a.kd / a.kd_n; else std::cout << std::setw(10) << "-";
    std::cout << std::setw(7) << a.mode << std::setw(8) << a.n << '\n';
    std::cout << "  轨迹目标换算到 SDK=" << a.expected_sdk / a.n * kRad2Deg
              << " deg，目标与 /rt/joint_cmd 的平均绝对差=" << a.target_cmd_error / a.n * kRad2Deg
              << " deg，与 /rt/joint_state 的平均绝对差=" << a.target_error / a.n * kRad2Deg << " deg\n";
  }
  std::cout << "kp/kd 仅观察，不修改。头部可能无有效 PD 字段；mode=2 为 CSP。\n"
            << "目标与反馈的差包含控制器接入/滤波；cmd−feedback 用于观察实际执行误差。\n";
}

inline void validateResult(const Spec& s, const Options& o, const std::array<Accum, 2>& stats) {
  for (std::size_t i = 0; i < s.dof; ++i) {
    const auto& a = stats[i];
    require(a.n > 0, "保持段没有有效样本");
    // 阈值小于非零请求步进，避免整段完全没动却被宽松跟踪阈值判为成功。
    const double tolerance = o.delta_deg[i] == 0 ? o.target_tolerance_deg :
        std::min(o.target_tolerance_deg, std::abs(o.delta_deg[i]) * 0.5);
    require(a.target_cmd_error / a.n * kRad2Deg <= tolerance,
            s.names[i] + " 实际关节命令未达到轨迹目标；测试指令可能未被消费或被其他来源覆盖");
    require(a.target_error / a.n * kRad2Deg <= tolerance,
            s.names[i] + " 反馈未达到目标容差；请结合已输出的 kp/kd、mode 和稳态误差检查");
  }
}

class Session {
 public:
  Session(const Spec& spec, const Options& options, Shared& shared,
          leju::vr::RobanVRAPI& api, bool motor, double direction,
          std::function<void()> protect_body = {})
      : s_(spec), o_(options), shared_(shared), api_(api), motor_(motor), direction_(direction),
        protect_body_(std::move(protect_body)) {}

  Vec sdk(const Vec& q) const {
    Vec result = q;
    for (std::size_t i = 0; i < s_.dof; ++i) result[i] *= direction_;
    return result;
  }

  bool publish(const Vec& q, const Vec& v) {
    if (protect_body_) protect_body_();
    checkAngles(s_, sdk(q), "发布目标");
    const auto f = frame(shared_, motor_);
    require(fresh(f.joint_state, o_.feedback_timeout_ms) && fresh(f.cmd, o_.feedback_timeout_ms) &&
            fresh(f.state, o_.feedback_timeout_ms) && fresh(f.joint_cmd, o_.feedback_timeout_ms),
            "指令或反馈超时，停止测试轨迹");
    checkAngles(s_, f.joint_state.q, "当前反馈");
    // 对实际命令与反馈做误差保护；目标处于平滑移动时不使用终点误差误判。
    for (std::size_t i = 0; i < s_.dof; ++i) {
      require(std::abs(f.cmd.q[i] - f.state.q[i]) * kRad2Deg <= o_.max_tracking_error_deg,
              s_.names[i] + " 实际指令−反馈超过 max_tracking_error_deg，停止测试轨迹");
    }
    leju::vr::JointTrajectoryPoint cmd;
    cmd.q.assign(q.begin(), q.begin() + s_.dof);
    cmd.v.assign(v.begin(), v.begin() + s_.dof);
    cmd.acc.assign(s_.dof, 0);
    return s_.part == Part::Head ? api_.publishHeadJointCmd(cmd) : api_.publishWaistJointCmd(cmd);
  }

  void move(const Vec& from, const Vec& to) {
    const auto period = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1 / o_.rate_hz));
    const auto start = Clock::now();
    auto next = start;
    Vec previous = from;
    auto previous_time = start;
    while (!stop_requested) {
      const auto now = Clock::now();
      const double elapsed = std::chrono::duration<double>(now - start).count();
      const double phase = std::min(1.0, elapsed / o_.move_sec);
      const double blend = 0.5 * (1 - std::cos(kPi * phase));
      const double slope = 0.5 * kPi / o_.move_sec * std::sin(kPi * phase);
      Vec q{}, v{};
      for (std::size_t i = 0; i < s_.dof; ++i) {
        const double limit = o_.max_velocity_deg_s / kRad2Deg *
            std::chrono::duration<double>(now - previous_time).count();
        const double wanted = from[i] + (to[i] - from[i]) * blend;
        q[i] = previous[i] + std::clamp(wanted - previous[i], -limit, limit);
        v[i] = (to[i] - from[i]) * slope;
      }
      require(publish(q, v), std::string("发布失败: ") + s_.trajectory_topic);
      previous = q; previous_time = now;
      bool arrived = phase >= 1;
      for (std::size_t i = 0; i < s_.dof; ++i) arrived = arrived && std::abs(q[i] - to[i]) < 1e-9;
      if (arrived) return;
      // 避免线程超时后连续补发旧周期，造成命令突发。
      next = std::max(next + period, Clock::now());
      std::this_thread::sleep_until(next);
    }
    throw std::runtime_error("收到 Ctrl+C/SIGTERM，停止测试轨迹");
  }

  void hold(const Vec& q, int duration_ms, int skip_ms, std::array<Accum, 2>* stats) {
    const auto period = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1 / o_.rate_hz));
    const auto start = Clock::now();
    const auto end = start + std::chrono::milliseconds(duration_ms);
    auto next = start;
    std::uint64_t last_state_seq = 0;
    while (Clock::now() < end && !stop_requested) {
      require(publish(q, Vec{}), std::string("发布失败: ") + s_.trajectory_topic);
      if (stats && Clock::now() - start >= std::chrono::milliseconds(skip_ms)) {
        const auto f = frame(shared_, motor_);
        // 每份反馈只计一次，不能把缓存的旧样本重复计作新的观测。
        if (f.state.seq != last_state_seq) {
          last_state_seq = f.state.seq;
          const auto expected = sdk(q);
          for (std::size_t i = 0; i < s_.dof; ++i) {
            addSample((*stats)[i], f.cmd, f.state, f.joint_state, f.joint_cmd, i, expected[i]);
          }
        }
      }
      next = std::max(next + period, Clock::now());
      std::this_thread::sleep_until(next);
    }
    require(!stop_requested, "收到 Ctrl+C/SIGTERM，停止测试轨迹");
  }

 private:
  const Spec& s_;
  const Options& o_;
  Shared& shared_;
  leju::vr::RobanVRAPI& api_;
  bool motor_;
  double direction_;
  std::function<void()> protect_body_;
};

inline void checkSdkNames(const Spec& s, const leju::RobotVersion& version) {
  leju::GlobalRobot::init_env(version);
  const auto& robot = leju::GlobalRobot::getInstance();
  const auto names = robot.getMotorNames();
  require(robot.getMotorNumber() == 23 && names.size() == 23, "SDK 本体关节数不是 23，拒绝套用 v17 下标");
  const std::array<std::string, 2> expected_head{{"zhead_1_link", "zhead_2_link"}};
  for (std::size_t i = 0; i < s.dof; ++i) {
    const auto& name = names[s.indices[i]];
    const auto& expected = s.part == Part::Head ? expected_head[i] : s.names[i];
    require(name == expected || name == s.names[i], "SDK 关节名称与 v17 预期不符: " + name);
    std::cout << "SDK[" << s.indices[i] << "]=" << name << "，URDF=" << s.names[i] << '\n';
  }
}

inline int run(Part part, int argc, char** argv) {
  const Spec s = specFor(part);
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "用法: build/" << s.stem << "_traj_tracking_test [--config PATH] [--dry-run | --inspect | --preflight]\n"
                << "--dry-run: 仅校验 YAML 和轨迹峰值速度，不初始化 DDS。\n"
                << "--inspect: 只查询 runtime/模式并订阅反馈，不切模式、不发布运动目标。\n"
                << "--preflight: 完整启动检查，到模式切换前退出；不切模式、不发布运动目标。\n"
                << "默认运行一次绝对目标、保持统计、返回起点；角度配置单位 deg。\n"
                << "YAML 用 target_deg 指定绝对目标，或用 delta_deg 指定相对步进，两项互斥。\n"
                << "ROBOT_VERSION 未设置时使用 17；已设为其他版本时拒绝运行。\n"
                << "Ctrl+C 停止测试轨迹；腰部恢复启动前模式。急停请使用现场急停。\n";
      return 0;
    }
  }
  try {
    std::cout << std::unitbuf;
    stop_requested = 0;
    std::signal(SIGINT, onSignal); std::signal(SIGTERM, onSignal);
    if (!std::getenv("ROBOT_VERSION")) setenv("ROBOT_VERSION", "17", 0);
    const auto version = leju::RobotVersion::from_env();
    require(version.to_string() == "17", "测试仅支持 ROBOT_VERSION=17");
    Options o = loadOptions(s, argc, argv);
    std::cout << "=== ROBAN v17 " << s.stem << " trajectory tracking ===\n"
              << "配置: " << o.config << "\n话题: " << s.trajectory_topic
              << (o.absolute_target ? "\n绝对目标 deg:" : "\n相对步进 deg:");
    for (std::size_t i = 0; i < s.dof; ++i) std::cout << ' ' << (o.absolute_target ? o.target_deg[i] : o.delta_deg[i]);
    std::cout << "；move=" << o.move_sec << " s，hold=" << o.hold_ms << " ms\n";
    if (o.dry_run) {
      std::cout << "离线配置校验通过；运行时将依据当前反馈校验起点/终点并自动延长移动时间以满足限速。\n";
      return 0;
    }

    configureTrackingDDS();
    Lock lock(s);
    // Shared 的生命周期长于所有 subscriber；SafeSubscriber 析构显式停止回调。
    Shared shared;
    dds::domain::DomainParticipant participant(0);
    dds::sub::qos::DataReaderQos qos;
    qos << dds::core::policy::Reliability::BestEffort();
    SafeSubscriber<leju::msgs::JointCmd> jc(participant, leju::dds_topics::kJointCmd,
        [&](const auto& msg) { onCmd(shared, &Shared::joint_cmd, s, msg); }, nullptr, qos);
    SafeSubscriber<leju::msgs::JointCmd> mc(participant, leju::dds_topics::kMotorCmd,
        [&](const auto& msg) { onCmd(shared, &Shared::motor_cmd, s, msg); }, nullptr, qos);
    SafeSubscriber<leju::msgs::JointState> js(participant, leju::dds_topics::kJointState,
        [&](const auto& msg) { onState(shared, &Shared::joint_state, s, msg); }, nullptr, qos);
    SafeSubscriber<leju::msgs::JointState> ms(participant, leju::dds_topics::kMotorState,
        [&](const auto& msg) { onState(shared, &Shared::motor_state, s, msg); }, nullptr, qos);
    checkSdkNames(s, version);
    leju::vr::RobanVRAPI api(version);
    require(api.initialize(), "VR SDK 初始化失败");
    ApiGuard api_cleanup{api};
    TrackingControlRpc rpc;
    leju::vr::RuntimeState runtime;
    leju::vr::ControllerState controller;
    require(rpc.getRuntimeState(runtime, 1500), "GetRuntimeState 查询超时");
    require(rpc.getControllerState(controller, 1500), "GetControllerState 查询超时");
    std::cout << "runtime: data_ready=" << std::boolalpha << runtime.data_ready
              << "，hardware=" << static_cast<int>(runtime.hardware_state)
              << "，lifecycle=" << static_cast<int>(runtime.lifecycle_state)
              << "\ncontroller=" << controller.current_controller
              << "，transitioning=" << controller.controller_transitioning
              << "，waist_mode=" << static_cast<int>(controller.waist_mode) << '\n';
    double direction = 1;
    bool waist_enabled = true;
    if (part == Part::Waist) waist_enabled = waistConfig(o, controller.current_controller, &direction);
    bool motor = false;
    require(waitStreams(shared, o.feedback_timeout_ms, &motor),
            "3s 内未收到完整且新鲜的 23 维指令/反馈；检查硬件、控制器与 CYCLONEDDS_URI");
    const auto initial = frame(shared, motor);
    Vec origin = initial.joint_state.q;
    for (std::size_t i = 0; i < s.dof; ++i) origin[i] *= direction;
    Vec target = prepareTarget(s, o, origin);
    std::cout << "诊断来源: " << (motor ? "motor_cmd/motor_state" : "joint_cmd/joint_state (电机话题不齐)") << '\n';
    for (std::size_t i = 0; i < s.dof; ++i) {
      std::cout << s.names[i] << "：当前 SDK=" << initial.joint_state.q[i] * kRad2Deg
                << " deg，轨迹起点=" << origin[i] * kRad2Deg << " deg，轨迹目标="
                << target[i] * kRad2Deg << " deg，预期 SDK 目标=" << direction * target[i] * kRad2Deg << " deg\n";
    }
    if (o.inspect) {
      std::cout << "只读检查结束。" << (waist_enabled ? "" : " 腰部未启用，不能进行腰轨迹测试。") << '\n';
      return 0;
    }
    require(runtime.data_ready && runtime.hardware_state == leju::vr::HardwareState::kReadyOk &&
            runtime.lifecycle_state == leju::vr::LifecycleState::kRunning && !controller.controller_transitioning,
            "需要 data_ready、ReadyOk、Running 且控制器未切换；本程序不启动硬件或 runtime");
    require(waist_enabled, "enable_waist_controller=false。先将 AMP 此项设为 true，并按原流程重新初始化控制器；仅改 YAML 不会使当前实例生效");
    leju::vr::MotionState motion;
    std::cout << "[preflight] 查询动作播放状态...\n";
    require(rpc.getMotionState(motion, 1500), "GetMotionState 查询超时，无法排除动作播放冲突");
    std::cout << "[preflight] motion_supported=" << motion.supported
              << "，motion_playing=" << motion.motion_playing << '\n';
    require(!motion.motion_playing, "当前正在播放 motion，请完成或停止原动作后再测试");
    require(!stop_requested, "测试启动被中断");
    Session session(s, o, shared, api, motor, direction);
    checkAngles(s, session.sdk(origin), "起点");
    checkAngles(s, session.sdk(target), "终点");
    if (part == Part::Waist) {
      require(controller.waist_mode == ControlMode::kAuto,
              "腰部当前不是 Auto，请先确认原外部控制程序已退出并恢复 Auto");
    }
    if (o.preflight) {
      std::cout << "完整只读启动检查通过；未切换模式、未发布运动轨迹。\n";
      return 0;
    }
    ModeGuard mode{rpc, controller.waist_mode, false};
    if (part == Part::Waist) {
      mode.armed = true;  // RPC 超时也可能已被服务端接收，所有后续出口尝试恢复。
      std::cout << "[mode] 请求腰部 External...\n";
      require(rpc.setWaistMode(ControlMode::kExternal, 1500), "SetWaistMode(External) 失败");
      std::cout << "[mode] External 请求已接收，等待实际模式...\n";
      const auto end = Clock::now() + std::chrono::seconds(2);
      bool confirmed = false;
      while (Clock::now() < end && !stop_requested) {
        leju::vr::ControllerState current;
        if (rpc.getControllerState(current, 500) && current.current_controller == controller.current_controller &&
            !current.controller_transitioning && current.waist_mode == ControlMode::kExternal) {
          confirmed = true; break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      require(confirmed, "External 模式未实际生效；检查运行中的控制器是否启用了腰部控制对象");
    }
    // RPC 可能等待较久，发布前重新读当前反馈，避免用旧起点。
    const auto before = frame(shared, motor);
    require(fresh(before.joint_state, o.feedback_timeout_ms), "接入前关节反馈超时");
    origin = session.sdk(before.joint_state.q);  // direction 为 ±1，正反转换相同。
    target = prepareTarget(s, o, origin);
    checkAngles(s, session.sdk(origin), "接入起点");
    checkAngles(s, session.sdk(target), "接入终点");
    std::cout << "保持起点 -> 平滑移动 -> 保持并采样 -> 平滑返回起点\n";
    session.hold(origin, o.settle_ms, 0, nullptr);
    session.move(origin, target);
    std::array<Accum, 2> stats{};
    session.hold(target, o.hold_ms, o.skip_ms, &stats);
    // 输出或报告失败不应使正常完成的目标保持在终点。
    session.move(target, origin);
    session.hold(origin, o.settle_ms, 0, nullptr);
    const auto returned = frame(shared, motor);
    bool returned_ok = fresh(returned.joint_state, o.feedback_timeout_ms);
    for (std::size_t i = 0; i < s.dof; ++i) {
      returned_ok = returned_ok && std::abs(returned.joint_state.q[i] - direction * origin[i]) * kRad2Deg <= o.target_tolerance_deg;
    }
    require(mode.restore(), "腰部原模式恢复失败");
    report(s, motor, stats);
    validateResult(s, o, stats);
    require(returned_ok, "已发布回程，但反馈未回到起点容差，请现场检查");
    std::cout << "测试完成，已发布返回起点轨迹。\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "测试失败: " << e.what() << '\n'
              << "已停止测试轨迹。中断/异常时不强行执行回程；Ctrl+C 不等于硬件急停。\n";
    return 1;
  }
}
}  // namespace roban_tracking
