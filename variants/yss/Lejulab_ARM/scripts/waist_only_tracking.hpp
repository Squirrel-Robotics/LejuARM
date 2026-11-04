#pragma once

#include "traj_tracking_common.hpp"
#include <lejusdk-dds-idl/StringData.hpp>
#include <lejusdk-topic-pubsub/topic_publisher.hpp>

namespace roban_tracking {
using Body = std::array<double, 23>;
inline bool isWaistOnly(const std::string& name) { return name.rfind("waist_only_", 0) == 0; }

struct BodyFrame { Body q, v; Clock::time_point received; std::uint64_t seq; };
inline BodyFrame bodyFrame(Shared& shared) {
  std::lock_guard<std::mutex> lock(shared.mu);
  return {shared.body_q, shared.body_v, shared.body_received, shared.body_seq};
}
inline void checkBodyFresh(const BodyFrame& f, int timeout_ms) {
  require(f.seq && Clock::now() - f.received < std::chrono::milliseconds(timeout_ms),
          "全身 23 维位置/速度反馈无效或超时，停止腰部轨迹");
}
inline double bodyDeviation(const Body& q, const Body& held, const std::vector<std::string>& names) {
  double largest = 0;
  for (std::size_t i = 0; i < q.size(); ++i) {
    if (i == 12) continue;
    const double error = std::abs(q[i] - held[i]) * kRad2Deg;
    require(std::isfinite(error) && error <= 3.0,
            names.at(i) + " 偏离固定姿态 " + std::to_string(error) + " deg（上限 3 deg），停止腰部轨迹");
    largest = std::max(largest, error);
  }
  return largest;
}
inline Body captureStationaryBody(Shared& shared, int timeout_ms) {
  const auto end = Clock::now() + std::chrono::seconds(3);
  auto since = Clock::now();
  while (Clock::now() < end && !stop_requested) {
    const auto f = bodyFrame(shared);
    checkBodyFresh(f, timeout_ms);
    const bool stationary = std::all_of(f.v.begin(), f.v.end(),
        [](double v) { return std::abs(v) * kRad2Deg <= 5.0; });
    if (!stationary) since = Clock::now();
    if (Clock::now() - since >= std::chrono::milliseconds(500)) return f.q;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  throw std::runtime_error("全身未连续静止 500ms（速度需 <=5 deg/s），未切换控制器");
}

struct FixedConfig { YAML::Node root; double direction; fs::path source; };
inline FixedConfig makeFixedConfig(const Options& o, const Body& held,
                                   const std::vector<std::string>& sdk_names) {
  const auto manager = YAML::LoadFile(o.manager.string());
  fs::path source;
  for (const auto& entry : manager["controllers"]) {
    if (entry["name"].as<std::string>() == "amp") {
      require(entry["type"].as<std::string>() == "GenericRLController", "AMP 控制器类型与已核验源码不符");
      source = entry["config"].as<std::string>(); break;
    }
  }
  require(!source.empty(), "manager 中找不到 AMP 配置");
  if (source.is_relative()) source = o.manager.parent_path() / source;
  YAML::Node root = YAML::LoadFile(source.string());
  auto cfg = root["HumanoidRobotCfg"];
  auto env = cfg["env"];
  auto robot = env["robot"];
  const auto names = robot["joint_names"].as<std::vector<std::string>>();
  const auto directions = robot["joint_direction"].as<std::vector<double>>();
  require(names.size() == 21 && directions.size() == 21 && sdk_names.size() == 23,
          "固定模式需要 v17 的 21 个策略关节和 23 个 SDK 关节");
  std::array<bool, 23> covered{};
  std::vector<double> defaults, scales(21, 0);
  double waist_direction = 0;
  for (std::size_t i = 0; i < names.size(); ++i) {
    const auto it = std::find(sdk_names.begin(), sdk_names.end(), names[i]);
    require(it != sdk_names.end() && (directions[i] == 1 || directions[i] == -1),
            "无法映射固定关节或方向非法: " + names[i]);
    const auto index = static_cast<std::size_t>(it - sdk_names.begin());
    require(index <= 20 && !covered[index] && std::isfinite(held[index]), "策略关节重复、越界或姿态无效");
    covered[index] = true;
    defaults.push_back(held[index] * directions[i]);
    if (index == 12) waist_direction = directions[i];
  }
  require(waist_direction != 0 && std::all_of(covered.begin(), covered.begin() + 21, [](bool x) { return x; }),
          "配置未完整覆盖腰、腿、双臂");
  for (const char* key : {"actuator_kp", "actuator_kd", "joint_torque_limit", "actuator_control_mode"}) {
    require(robot[key].IsSequence() && robot[key].size() == 21, std::string("固定配置维度错误: ") + key);
  }
  // 现有 GenericRLController 在 CST 分支以这些固定目标计算 PD 力矩。
  // 不直接抢占 /rt/joint_cmd；保持已核验的增益/力矩限幅。
  robot["joint_default_pos"] = defaults;
  robot["start_stand_pos"] = defaults;
  robot["action_scale"] = scales;
  robot["actuator_control_mode"] = std::vector<int>(21, 0);
  robot["head_default_pos"] = std::vector<double>{held[21], held[22]};
  // 双臂使用 KeepPose，保留现有控制器的 URDF 力矩补偿；不让 AMP 输出接管。
  robot["enable_arm_controller"] = true;
  robot["enable_waist_controller"] = true;
  for (const char* key : {"enable_amp_arm_enhance_v17", "enable_back_enhance_v17",
                         "enable_standup_enhance_v17", "enable_turn_in_place_enhance_v17"}) robot[key] = false;
  env["enable_roll_compensation_closed_loop"] = false;
  env["enable_lateral_yaw_compensation_closed_loop"] = false;
  env["squat_posture_defense"]["enabled"] = false;
  env["amp_hand_posture_axis"]["enabled"] = false;
  cfg["limp_when_idle"] = false;
  cfg.remove("motion");
  cfg.remove("prone_policy_path"); cfg.remove("supine_policy_path");
  fs::path model = cfg["policy_path"].as<std::string>();
  if (model.is_relative()) model = source.parent_path() / model;
  require(fs::is_regular_file(model), "固定配置的策略模型不存在: " + model.string());
  cfg["policy_path"] = fs::canonical(model).string();
  return {root, waist_direction, source};
}

inline std::string fileText(const fs::path& path) {
  std::ifstream input(path);
  require(input.good(), "无法读取文件: " + path.string());
  std::ostringstream contents; contents << input.rdbuf();
  return contents.str();
}
inline void atomicText(const fs::path& path, const std::string& text) {
  const fs::path temp = path.string() + ".waist-tmp-" + std::to_string(getpid());
  try {
    std::ofstream output(temp, std::ios::trunc);
    require(output.good(), "无法创建临时文件: " + temp.string());
    output << text; output.close();
    require(output.good(), "写文件失败: " + temp.string());
    if (fs::exists(path)) fs::permissions(temp, fs::status(path).permissions());
    fs::rename(temp, path);
  } catch (...) { std::error_code ignored; fs::remove(temp, ignored); throw; }
}
// reload 只增加未加载的名字，不能更新既有实例。每次用新名字捕获本次姿态。
// 注册后立即恢复 manager 原文：避免重启时加载上一次的姿态配置。
struct ManagerRegistration {
  fs::path path;
  std::string original, staged;
  bool changed = false;
  void restore() {
    if (!changed) return;
    require(fileText(path) == staged, "manager 在注册期间被其他程序修改，保留现场文件及备份，请人工检查");
    atomicText(path, original); changed = false;
  }
  ~ManagerRegistration() {
    if (changed) try { restore(); } catch (const std::exception& e) { std::cerr << "manager 恢复失败: " << e.what() << '\n'; }
  }
};

inline std::string registerFixedController(const Options& o, const FixedConfig& config,
                                           TrackingControlRpc& rpc,
                                           const dds::domain::DomainParticipant& participant,
                                           const leju::vr::ControllerState& original_state) {
  const auto loaded = std::count_if(original_state.available_controllers.begin(), original_state.available_controllers.end(), isWaistOnly);
  require(loaded < 8, "本次控制器进程已加载 8 个腰测试实例，请按原流程重新初始化后再测");
  const auto stamp = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  const std::string name = "waist_only_" + std::to_string(stamp) + "_" + std::to_string(getpid());
  const auto directory = o.project / "build/tracking_debug/waist_only";
  fs::create_directories(directory);
  const auto config_path = directory / (name + ".yaml");
  YAML::Emitter yaml; yaml.SetDoublePrecision(17); yaml << config.root;
  require(yaml.good(), "固定配置 YAML 序列化失败");
  atomicText(config_path, yaml.c_str());
  ManagerRegistration registration;
  registration.path = o.manager;
  registration.original = fileText(o.manager);
  atomicText(directory / (name + "_manager_backup.yaml"), registration.original);
  registration.staged = registration.original + "\n  - name: \"" + name +
      "\"\n    type: \"GenericRLController\"\n    config: \"" + config_path.string() + "\"\n    enabled: true\n";
  require(YAML::Load(registration.staged)["controllers"].IsSequence(), "manager 注册内容无效");
  dds::pub::qos::DataWriterQos reload_qos;
  reload_qos << dds::core::policy::Reliability::Reliable()
             << dds::core::policy::History::KeepLast(1);
  leju::dds_common::TopicPublisher<leju::msgs::StringData> reload(participant, "/rt/teleop/reload_controllers", reload_qos);
  const auto discovery_deadline = Clock::now() + std::chrono::seconds(3);
  while (reload.writer().publication_matched_status().current_count() == 0 &&
         Clock::now() < discovery_deadline && !stop_requested) std::this_thread::sleep_for(std::chrono::milliseconds(20));
  require(reload.writer().publication_matched_status().current_count() > 0 && !stop_requested,
          "未发现控制器 reload 接收端，未改 manager、未切换控制器");
  atomicText(o.manager, registration.staged);
  registration.changed = true;
  leju::msgs::StringData trigger; trigger.data(name);
  require(reload.publish(trigger), "注册控制器 reload 发布失败");
  std::cout << "[fixed] 注册专用固定姿态控制器: " << name << '\n';
  const auto deadline = Clock::now() + std::chrono::seconds(15);
  auto next_trigger = Clock::now() + std::chrono::seconds(1);
  bool found = false;
  while (Clock::now() < deadline && !stop_requested) {
    if (Clock::now() >= next_trigger) {
      require(reload.publish(trigger), "注册控制器 reload 重试失败");
      next_trigger = Clock::now() + std::chrono::seconds(1);
    }
    leju::vr::ControllerState current;
    if (rpc.getControllerState(current, 1000)) {
      require(current.current_controller == original_state.current_controller && !current.controller_transitioning,
              "注册期间当前控制器发生变化，未启动腰部测试");
      if (std::find(current.available_controllers.begin(), current.available_controllers.end(), name) != current.available_controllers.end()) {
        found = true; break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  registration.restore();
  require(found, "专用固定姿态控制器未成功加载，未切换/未发布腰轨迹；查看 run_rl_controller 日志");
  std::cout << "[fixed] 配置已加载；manager 已恢复原文。姿态配置: " << config_path << '\n';
  return name;
}

struct FixedModeGuard {
  TrackingControlRpc& rpc;
  std::string name;
  bool armed = false;
  bool keep() {
    if (!armed) return true;
    leju::vr::ControllerState state;
    const auto end = Clock::now() + std::chrono::seconds(6);
    bool settled = false;
    while (Clock::now() < end) {
      if (rpc.getControllerState(state, 700) && !state.controller_transitioning) { settled = true; break; }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (!settled) return false;
    if (state.current_controller != name) { armed = false; return false; }
    if (!rpc.setArmMode(ControlMode::kKeepPose, 1500)) return false;
    if (!rpc.setWaistMode(ControlMode::kKeepPose, 1500)) return false;
    const auto confirm_end = Clock::now() + std::chrono::seconds(2);
    while (Clock::now() < confirm_end) {
      if (rpc.getControllerState(state, 500) && state.current_controller == name &&
          state.waist_mode == ControlMode::kKeepPose && state.arm_mode == ControlMode::kKeepPose && !state.controller_transitioning) {
        armed = false; return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }
  ~FixedModeGuard() {
    if (armed) try { if (!keep()) std::cerr << "腰 KeepPose 未确认，请现场检查。\n"; }
    catch (...) { std::cerr << "腰 KeepPose 调用异常，请现场检查。\n"; }
  }
};

inline int runWaistOnly(int argc, char** argv) {
  const auto spec = specFor(Part::Waist);
  try {
    std::cout << std::unitbuf;
    bool register_only = false;
    std::vector<char*> args{argv[0]};
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--register-only") register_only = true;
      else if (arg == "--help" || arg == "-h") {
        std::cout << "build/waist_traj_tracking_test [--config PATH] [--dry-run | --inspect | --preflight | --register-only]\n"
                  << "默认：捕获当前姿态，固定腿/双臂/头，腰到目标并返回；结束保留固定控制器及腰 KeepPose。\n"
                  << "需要吊架可靠支撑，腿部不做 AMP 平衡补偿；全身为关节姿态保持，随腰转动的上身会一起转动。\n"
                  << "preflight/inspect 只读；register-only 仅注册未激活的固定配置，不切模式、不发布部位指令。\n";
        return 0;
      } else args.push_back(argv[i]);
    }
    auto o = loadOptions(spec, static_cast<int>(args.size()), args.data());
    require(!register_only || (!o.dry_run && !o.inspect && !o.preflight), "register-only 请单独使用");
    std::cout << "=== ROBAN v17 腰部独立测试：其余22关节固定 ===\n配置: " << o.config << '\n';
    if (o.dry_run) { std::cout << "离线 YAML 校验通过；未初始化 DDS。\n"; return 0; }
    stop_requested = 0;
    std::signal(SIGINT, onSignal); std::signal(SIGTERM, onSignal);
    if (!std::getenv("ROBOT_VERSION")) setenv("ROBOT_VERSION", "17", 0);
    const auto version = leju::RobotVersion::from_env();
    require(version.to_string() == "17", "仅支持 ROBOT_VERSION=17");
    configureTrackingDDS();
    Lock lock(spec);
    Shared shared;
    dds::domain::DomainParticipant participant(0);
    dds::sub::qos::DataReaderQos qos; qos << dds::core::policy::Reliability::BestEffort();
    SafeSubscriber<leju::msgs::JointCmd> jc(participant, leju::dds_topics::kJointCmd,
        [&](const auto& m) { onCmd(shared, &Shared::joint_cmd, spec, m); }, nullptr, qos);
    SafeSubscriber<leju::msgs::JointCmd> mc(participant, leju::dds_topics::kMotorCmd,
        [&](const auto& m) { onCmd(shared, &Shared::motor_cmd, spec, m); }, nullptr, qos);
    SafeSubscriber<leju::msgs::JointState> js(participant, leju::dds_topics::kJointState,
        [&](const auto& m) { onState(shared, &Shared::joint_state, spec, m); }, nullptr, qos);
    SafeSubscriber<leju::msgs::JointState> ms(participant, leju::dds_topics::kMotorState,
        [&](const auto& m) { onState(shared, &Shared::motor_state, spec, m); }, nullptr, qos);
    checkSdkNames(spec, version);
    const auto names = leju::GlobalRobot::getInstance().getMotorNames();
    leju::vr::RobanVRAPI api(version);
    require(api.initialize(), "VR SDK 初始化失败");
    ApiGuard cleanup{api};
    TrackingControlRpc rpc;
    leju::vr::RuntimeState runtime; leju::vr::ControllerState controller;
    require(rpc.getRuntimeState(runtime, 1500) && rpc.getControllerState(controller, 1500), "运行状态查询失败");
    std::cout << "controller=" << controller.current_controller << "，waist_mode=" << static_cast<int>(controller.waist_mode)
              << "，hardware=" << static_cast<int>(runtime.hardware_state) << "，lifecycle=" << static_cast<int>(runtime.lifecycle_state) << '\n';
    bool motor = false;
    require(waitStreams(shared, o.feedback_timeout_ms, &motor), "指令/反馈话题不完整或超时");
    const auto body = bodyFrame(shared); checkBodyFresh(body, o.feedback_timeout_ms);
    const auto config = makeFixedConfig(o, body.q, names);
    if (o.inspect) { std::cout << "只读检查结束；腰 joint_direction=" << config.direction << '\n'; return 0; }
    require(runtime.data_ready && runtime.hardware_state == leju::vr::HardwareState::kReadyOk &&
            runtime.lifecycle_state == leju::vr::LifecycleState::kRunning && !controller.controller_transitioning,
            "需要 ReadyOk、Running、反馈就绪且未切换控制器");
    require(controller.current_controller == "amp" || isWaistOnly(controller.current_controller), "请先使用 amp 或本测试的固定控制器");
    require(controller.waist_mode != ControlMode::kExternal, "腰仍为 External，请先退出其他腰控制程序，再恢复 Auto/KeepPose");
    leju::vr::MotionState motion;
    require(rpc.getMotionState(motion, 1500) && !motion.motion_playing, "动作播放状态查询失败或已有 motion 正在播放");
    const auto held = captureStationaryBody(shared, o.feedback_timeout_ms);
    checkAngles(specFor(Part::Head), {held[21], held[22]}, "头部固定姿态");
    const auto fixed = makeFixedConfig(o, held, names);
    Vec origin{held[12] * fixed.direction, 0};
    const auto target = prepareTarget(spec, o, origin);
    checkAngles(spec, {held[12], 0}, "起点");
    checkAngles(spec, {target[0] * fixed.direction, 0}, "终点");
    std::cout << "固定: 腿12关节、双臂8关节、头2关节；腰目标=" << target[0] * kRad2Deg
              << " deg，SDK目标=" << fixed.direction * target[0] * kRad2Deg << " deg，move=" << o.move_sec << " s\n";
    if (o.preflight) { std::cout << "完整只读检查通过；未改配置、未切模式、未发布部位指令。\n"; return 0; }
    const auto name = registerFixedController(o, fixed, rpc, participant, controller);
    if (register_only) { std::cout << "仅注册验证通过；当前控制器保持 " << controller.current_controller << "。\n"; return 0; }
    require(!stop_requested, "测试被中断，未切换控制器");
    const auto before_switch = bodyFrame(shared);
    checkBodyFresh(before_switch, o.feedback_timeout_ms);
    bodyDeviation(before_switch.q, held, names);
    FixedModeGuard guard{rpc, name, true};
    // 在切换前清除旧的全局头部目标，值等于刚捕获的姿态，速度为零。
    leju::vr::JointTrajectoryPoint head;
    head.q = {held[21], held[22]}; head.v = {0, 0}; head.acc = {0, 0};
    require(api.publishHeadJointCmd(head), "头部固定目标发布失败");
    require(rpc.switchController(name, 2000), "固定控制器切换请求失败");
    const auto end = Clock::now() + std::chrono::seconds(6);
    bool active = false;
    while (Clock::now() < end && !stop_requested) {
      leju::vr::ControllerState state;
      if (rpc.getControllerState(state, 700) && state.current_controller == name && !state.controller_transitioning) { active = true; break; }
      checkBodyFresh(bodyFrame(shared), o.feedback_timeout_ms);
      bodyDeviation(bodyFrame(shared).q, held, names);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    require(active, "固定控制器未实际接入，未发布腰轨迹");
    require(rpc.setArmMode(ControlMode::kKeepPose, 1500), "双臂 KeepPose 请求失败");
    require(rpc.setWaistMode(ControlMode::kExternal, 1500), "固定控制器腰 External 请求失败");
    const auto mode_end = Clock::now() + std::chrono::seconds(2);
    bool external = false;
    while (Clock::now() < mode_end && !stop_requested) {
      if (rpc.getControllerState(controller, 500) && controller.current_controller == name &&
          !controller.controller_transitioning && controller.arm_mode == ControlMode::kKeepPose &&
          controller.waist_mode == ControlMode::kExternal) { external = true; break; }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    require(external, "固定控制器/External 未确认，未发布腰轨迹");
    double largest_deviation = 0;
    auto next_rpc = Clock::now();
    auto protect = [&] {
      const auto f = bodyFrame(shared); checkBodyFresh(f, o.feedback_timeout_ms);
      largest_deviation = std::max(largest_deviation, bodyDeviation(f.q, held, names));
      if (Clock::now() >= next_rpc) {
        require(rpc.getControllerState(controller, 500) && controller.current_controller == name &&
                !controller.controller_transitioning && controller.waist_mode == ControlMode::kExternal &&
                controller.arm_mode == ControlMode::kKeepPose,
                "固定控制器、双臂 KeepPose 或腰 External 被其他来源改变，停止腰轨迹");
        require(rpc.getRuntimeState(runtime, 500) && runtime.data_ready &&
                runtime.hardware_state == leju::vr::HardwareState::kReadyOk &&
                runtime.lifecycle_state == leju::vr::LifecycleState::kRunning, "运行状态异常，停止腰轨迹");
        next_rpc = Clock::now() + std::chrono::milliseconds(500);
      }
      require(api.publishHeadJointCmd(head), "头部固定目标发布失败");
    };
    Session session(spec, o, shared, api, motor, fixed.direction, protect);
    // 切换期间可能有微小漂移，重新捕获腰起点；其余22关节仍使用 held。
    const auto before = frame(shared, motor);
    require(fresh(before.joint_state, o.feedback_timeout_ms), "腰接入反馈超时");
    origin = session.sdk(before.joint_state.q);
    const auto endpoint = prepareTarget(spec, o, origin);
    session.hold(origin, o.settle_ms, 0, nullptr);
    session.move(origin, endpoint);
    std::array<Accum, 2> stats{};
    session.hold(endpoint, o.hold_ms, o.skip_ms, &stats);
    session.move(endpoint, origin);
    session.hold(origin, o.settle_ms, 0, nullptr);
    require(guard.keep(), "腰 KeepPose 未实际生效");
    report(spec, motor, stats); validateResult(spec, o, stats);
    const auto returned = frame(shared, motor);
    require(fresh(returned.joint_state, o.feedback_timeout_ms) &&
            std::abs(returned.joint_state.q[0] - fixed.direction * origin[0]) * kRad2Deg <= o.target_tolerance_deg,
            "腰反馈未回到起点容差");
    std::cout << "测试完成；其余22关节最大偏移=" << largest_deviation
              << " deg。保留固定控制器及腰 KeepPose，不自动切回 AMP。\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "测试失败: " << e.what() << "\n停止腰轨迹；已接入固定模式时尝试腰 KeepPose，保留其余关节固定。\n"
              << "异常不强行回程；Ctrl+C 不等于硬件急停。\n";
    return 1;
  }
}
}  // namespace roban_tracking
