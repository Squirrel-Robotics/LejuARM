#include <lejusdk-lowlevel/leju_sdk.h>
#include <lejusdk-utils/robot_version.hpp>
#include <lejusdk-vr/lejusdk_vr.h>

#include <atomic>
#include <chrono>
#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <sys/wait.h>
#include <unistd.h>

namespace {

using Clock = std::chrono::steady_clock;

constexpr char kTmuxSessionName[] = "lejulab_runtime";
constexpr char kWorkspaceRootEnv[] = "LEJULAB_WS_ROOT";
constexpr char kActiveProfileLink[] = "/root/.config/lejulab/auto_start_config/current";
constexpr char kActiveControllerManagerConfig[] =
    "/root/.config/lejulab/auto_start_config/current/controller_manager.yaml";
constexpr int kRpcTimeoutMs = 1000;
constexpr int kDebounceMs = 150;
// 必须大于 launcher _cleanup 的 5s 优雅退出宽限 (硬件节点掉使能耗时),
// 否则外层 tmux kill-session 会把掉使能流程连锅端
constexpr int kStopGraceMs = 8000;
constexpr int kLoopIntervalMs = 100;

std::atomic<bool> g_shutdown_requested{false};

void HandleSignal(int) {
  g_shutdown_requested.store(true);
}

void Log(const std::string& message) {
  std::cout << "[leju_joy_autostart_manager] " << message << std::endl;
}

void LogError(const std::string& message) {
  std::cerr << "[leju_joy_autostart_manager] " << message << std::endl;
}

bool StartsWith(const std::string& value, const std::string& prefix) {
  return value.rfind(prefix, 0) == 0;
}

struct Config {
  std::string workspace_root;
  std::string runtime_cmd;
  std::chrono::milliseconds rpc_timeout{kRpcTimeoutMs};
  std::chrono::milliseconds debounce{kDebounceMs};
  std::chrono::milliseconds stop_grace{kStopGraceMs};
  std::chrono::milliseconds loop_interval{kLoopIntervalMs};
};

bool LoadConfig(Config* config) {
  const char* workspace_root = std::getenv(kWorkspaceRootEnv);
  if (workspace_root == nullptr || workspace_root[0] == '\0') {
    LogError(std::string("未设置环境变量 ") + kWorkspaceRootEnv);
    return false;
  }

  config->workspace_root = workspace_root;
  config->runtime_cmd =
      "exec bash \"" + config->workspace_root +
      "/src/leju_launch/scripts/launch_real.sh\" --auto-start "
      "--config " + std::string(kActiveControllerManagerConfig);
  return true;
}

bool ValidateActiveControllerManagerConfig(std::string* resolved_dir) {
  namespace fs = std::filesystem;

  std::error_code ec;
  const fs::path current_link(kActiveProfileLink);
  const fs::file_status current_status = fs::symlink_status(current_link, ec);
  if (ec || current_status.type() == fs::file_type::not_found) {
    LogError(std::string("未找到 active 配置入口: ") + kActiveProfileLink);
    return false;
  }
  if (!fs::is_symlink(current_status)) {
    LogError(std::string("active 配置入口不是符号链接: ") + kActiveProfileLink);
    return false;
  }

  const fs::path resolved = fs::canonical(current_link, ec);
  if (ec || !fs::is_directory(resolved)) {
    LogError(std::string("active 配置目录无法解析: ") + kActiveProfileLink);
    return false;
  }

  const fs::path controller_config(kActiveControllerManagerConfig);
  if (!fs::exists(controller_config, ec) || ec) {
    LogError(std::string("active 配置缺少 controller_manager.yaml: ") +
             kActiveControllerManagerConfig);
    return false;
  }

  if (resolved_dir != nullptr) {
    *resolved_dir = resolved.string();
  }
  return true;
}

int RunCommand(const std::vector<std::string>& args, bool quiet) {
  std::vector<char*> argv;
  argv.reserve(args.size() + 1);
  for (const auto& arg : args) {
    argv.push_back(const_cast<char*>(arg.c_str()));
  }
  argv.push_back(nullptr);

  pid_t pid = fork();
  if (pid < 0) {
    LogError("fork 失败");
    return -1;
  }

  if (pid == 0) {
    if (quiet) {
      int devnull = open("/dev/null", O_RDWR);
      if (devnull >= 0) {
        dup2(devnull, STDOUT_FILENO);
        dup2(devnull, STDERR_FILENO);
        if (devnull > STDERR_FILENO) {
          close(devnull);
        }
      }
    }

    execvp(argv[0], argv.data());
    _exit(127);
  }

  int status = 0;
  while (waitpid(pid, &status, 0) < 0) {
    if (errno == EINTR) {
      continue;
    }
    LogError("waitpid 失败");
    return -1;
  }

  if (WIFEXITED(status)) {
    return WEXITSTATUS(status);
  }
  return -1;
}

class TmuxSession {
 public:
  TmuxSession() = default;

  bool Exists() const {
    return RunCommand({"tmux", "has-session", "-t", kTmuxSessionName}, true) == 0;
  }

  bool Start(const std::string& runtime_cmd) const {
    int rc = RunCommand(
        {"tmux", "new-session", "-d", "-s", kTmuxSessionName, "/bin/bash", "-lc",
         runtime_cmd},
        true);
    return rc == 0;
  }

  bool SendCtrlC() const {
    int rc = RunCommand({"tmux", "send-keys", "-t", kTmuxSessionName, "C-c"}, true);
    return rc == 0;
  }

  bool Kill() const {
    int rc = RunCommand({"tmux", "kill-session", "-t", kTmuxSessionName}, true);
    return rc == 0;
  }
};

struct ButtonSnapshot {
  bool start_pressed = false;
  bool back_pressed = false;
  bool pre_start_recovery_pressed = false;
};

class JoyAutostartManager {
 public:
  JoyAutostartManager(const Config& config, const leju::RobotVersion& robot_version)
      : config_(config),
        robot_version_(robot_version),
        vr_api_(robot_version) {}

  bool Initialize() {
    if (!leju::GlobalRobot::init_env(robot_version_)) {
      LogError("GlobalRobot::init_env() 失败");
      return false;
    }

    if (!vr_api_.initialize()) {
      LogError("VRBaseAPI initialize 失败");
      return false;
    }

    auto& robot = leju::GlobalRobot::getInstance();
    robot.subscribeJoyData([this](const leju::JoyDataConstPtr& joy) {
      std::lock_guard<std::mutex> lock(button_mutex_);
      buttons_.start_pressed = joy->buttons.start != 0;
      buttons_.back_pressed = joy->buttons.back != 0;
      buttons_.pre_start_recovery_pressed =
          joy->buttons.left_shoulder != 0 &&
          joy->buttons.right_shoulder != 0 && joy->buttons.west != 0;
    });

    Log(std::string("初始化完成，tmux session=") + kTmuxSessionName);
    return true;
  }

  void Run() {
    while (!g_shutdown_requested.load()) {
      Tick();
      std::this_thread::sleep_for(config_.loop_interval);
    }
  }

  void Shutdown() {
    vr_api_.shutdown();
    try {
      leju::GlobalRobot::getInstance().shutdown();
    } catch (...) {
    }
  }

 private:
  void Tick() {
    const auto now = Clock::now();
    const bool session_exists = session_.Exists();
    const ButtonSnapshot buttons = ReadButtons();

    if (stop_in_progress_) {
      if (!session_exists) {
        stop_in_progress_ = false;
        Log("runtime 已退出，tmux session 已清理");
        return;
      }

      if (now >= stop_deadline_) {
        Log("优雅退出超时，强制关闭 tmux session");
        session_.Kill();
        if (!session_.Exists()) {
          stop_in_progress_ = false;
        }
      }
      prev_buttons_ = buttons;
      return;
    }

    const bool back_edge = buttons.back_pressed && !prev_buttons_.back_pressed;
    const bool start_edge = buttons.start_pressed && !prev_buttons_.start_pressed;
    const bool pre_start_recovery_edge =
        buttons.pre_start_recovery_pressed &&
        !prev_buttons_.pre_start_recovery_pressed;

    prev_buttons_ = buttons;

    // 同帧 combo：任意一边沿命中且当前两键都按下 → quit 流程
    if ((back_edge || start_edge) && buttons.back_pressed && buttons.start_pressed) {
      if (PassesDebounce(now)) {
        Log("BACK+START combo => Quit");
        HandleBack(session_exists, now);
      }
      return;
    }

    // 单按 BACK：忽略，无日志
    if (back_edge && !buttons.start_pressed) {
      return;
    }

    // runtime 尚未启动时，LB+RB+X 既是启动请求，也是 START 前倒地起身的
    // 第一次按键。通过启动参数把这个按下沿可靠地交给新进程，避免启动期间
    // 用户松键导致 run_rl_controller 永远看不到原始 Joy 边沿。
    if (pre_start_recovery_edge && PassesDebounce(now)) {
      HandlePreStartRecovery(session_exists);
      return;
    }

    if (start_edge && !buttons.back_pressed && PassesDebounce(now)) {
      HandleStart(session_exists);
    }
  }

  ButtonSnapshot ReadButtons() {
    std::lock_guard<std::mutex> lock(button_mutex_);
    return buttons_;
  }

  bool PassesDebounce(Clock::time_point now) {
    if (now - last_button_event_time_ < config_.debounce) {
      return false;
    }
    last_button_event_time_ = now;
    return true;
  }

  bool HasExternalRuntime(leju::vr::RuntimeState* runtime_state = nullptr) {
    leju::vr::RuntimeState probed_state;
    if (!vr_api_.getRuntimeState(probed_state,
                                 static_cast<int>(config_.rpc_timeout.count()))) {
      return false;
    }

    if (runtime_state != nullptr) {
      *runtime_state = probed_state;
    }
    return true;
  }

  void HandleStart(bool session_exists) {
    if (!session_exists) {
      leju::vr::RuntimeState external_runtime_state;
      if (HasExternalRuntime(&external_runtime_state)) {
        Log("检测到外部 runtime 正在运行，autostart manager 忽略本次 START");
        return;
      }

      std::string active_profile_dir;
      if (!ValidateActiveControllerManagerConfig(&active_profile_dir)) {
        LogError("active 配置无效，本次 START 已忽略");
        return;
      }
      Log("本次启动使用 active 配置: " + active_profile_dir);
      Log("检测到第一次 START，拉起 tmux runtime");
      if (!session_.Start(config_.runtime_cmd)) {
        LogError("tmux runtime 启动失败");
      }
      return;
    }

    leju::vr::RuntimeState runtime_state;
    if (!vr_api_.getRuntimeState(runtime_state,
                                 static_cast<int>(config_.rpc_timeout.count()))) {
      Log("runtime RPC 尚不可达，忽略本次 START");
      return;
    }

    if (runtime_state.lifecycle_state == leju::vr::LifecycleState::kRunning) {
      Log("runtime 已在运行中，忽略本次 START");
      return;
    }

    if (runtime_state.lifecycle_state == leju::vr::LifecycleState::kExiting) {
      Log("runtime 正在退出中，忽略本次 START");
      return;
    }

    std::string message;
    if (vr_api_.startRuntime(&message, static_cast<int>(config_.rpc_timeout.count()))) {
      Log("第二次 START 已触发 startRuntime()");
    } else {
      LogError("startRuntime() 失败: " + message);
    }
  }

  void HandlePreStartRecovery(bool session_exists) {
    if (session_exists) {
      // runtime 已存在时由它直接消费原始 JoyData，不能在这里重复注入事件。
      return;
    }

    leju::vr::RuntimeState external_runtime_state;
    if (HasExternalRuntime(&external_runtime_state)) {
      Log("检测到外部 runtime 正在运行，LB+RB+X 交由 runtime 直接处理");
      return;
    }

    std::string active_profile_dir;
    if (!ValidateActiveControllerManagerConfig(&active_profile_dir)) {
      LogError("active 配置无效，本次 LB+RB+X 已忽略");
      return;
    }

    Log("本次启动使用 active 配置: " + active_profile_dir);
    Log("检测到 START 前 LB+RB+X，拉起 runtime 并保留倒地起身事件");
    if (!session_.Start(config_.runtime_cmd + " --pre-start-fall-recovery")) {
      LogError("tmux runtime 启动失败");
    }
  }

  void HandleBack(bool session_exists, Clock::time_point now) {
    if (!session_exists) {
      if (HasExternalRuntime()) {
        Log("检测到外部 runtime 正在运行，autostart manager 不处理本次 BACK");
      } else {
        Log("未检测到 tmux session，无需进一步清理");
      }
      return;
    }

    // 不再立即 stopRuntime()/SendCtrl-C：runtime 的 ControlLoop 收到 QuitSquat 后会
    // 先下蹲再自行退出（先下蹲再结束程序）。立即 stopRuntime() 会让 ExternalInterface
    // 直接 push Quit 跳过下蹲（issue #3937 现象）；SendCtrl-C 会 SIGINT 整个 tmux session。
    // 这里只登记优雅停止窗口，超时（stop_grace=8s）仍强制清理兜底。
    Log("BACK+START：由 runtime 执行先下蹲再退出，monitor 登记优雅停止窗口");
    stop_in_progress_ = true;
    stop_deadline_ = now + config_.stop_grace;
  }

  Config config_;
  TmuxSession session_;
  leju::RobotVersion robot_version_;
  leju::vr::VRBaseAPI vr_api_;
  std::mutex button_mutex_;
  ButtonSnapshot buttons_{};
  ButtonSnapshot prev_buttons_{};
  Clock::time_point last_button_event_time_{Clock::now() - std::chrono::seconds(1)};
  bool stop_in_progress_ = false;
  Clock::time_point stop_deadline_{};
};

bool ValidateArgs(int argc, char* argv[]) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (StartsWith(arg, "__") || arg.find(":=") != std::string::npos) {
      continue;
    }
    Log("忽略未使用参数: " + arg);
  }
  return true;
}

}  // namespace

int main(int argc, char* argv[]) {
  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);

  ValidateArgs(argc, argv);

  leju::RobotVersion robot_version;
  try {
    robot_version = leju::RobotVersion::from_env();
  } catch (const std::exception& ex) {
    LogError(std::string("读取 ROBOT_VERSION 失败: ") + ex.what());
    return 1;
  }

  Config config;
  if (!LoadConfig(&config)) {
    return 1;
  }

  JoyAutostartManager manager(config, robot_version);
  if (!manager.Initialize()) {
    return 1;
  }

  manager.Run();
  manager.Shutdown();
  return 0;
}
