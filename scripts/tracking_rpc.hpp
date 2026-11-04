#pragma once

// 测试专用 RPC 客户端：沿用 SDK 的 IDL、话题名和 QoS，但使用已构造的回复对象。
// 此 SDK Requester 的 16 样本 loan 与 CycloneDDS C++ 0.10.2 的单样本分配回调不兼容。
// 不修改已安装 SDK 或控制器；所有调用都在测试主线程同步执行。
#include <dds_rpc/rpc_types.hpp>
#include <rpc_idl/ControllerService.hpp>
#include <rpc_idl/StdSrvs.hpp>
#include <lejusdk-vr/data_types.h>
#include <dds/topic/TopicTraits.hpp>
#include <org/eclipse/cyclonedds/topic/datatopic.hpp>

#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>
#include <thread>

namespace roban_tracking {
namespace tracking_rpc_detail {

struct Entity {
  dds_entity_t handle = 0;
  Entity() = default;
  Entity(const Entity&) = delete;
  Entity& operator=(const Entity&) = delete;
  ~Entity() { if (handle > 0) dds_delete(handle); }
  void reset(dds_entity_t value, const std::string& operation) {
    if (value <= 0) throw std::runtime_error(operation + " DDS error=" + std::to_string(value));
    handle = value;
  }
};

// 非空的 sample 指针指向已经构造的 C++ 对象：DDS 复制到该对象，不申请 loan。
// 每次只取一条，既不依赖 C++ 类型的多样本 allocator，也不用 dds_return_loan。
template<class Reply>
int takeOwnedReply(dds_entity_t reader, Reply& reply, dds_sample_info_t& info) {
  void* samples[1]{&reply};
  const int count = dds_take(reader, samples, &info, 1, 1);
  if (count < 0) throw std::runtime_error("RPC dds_take error=" + std::to_string(count));
  return count;
}

template<class Request, class Reply>
class Requester {
 public:
  Requester(dds_entity_t participant, std::string service) : service_(std::move(service)) {
    std::random_device random;
    for (auto& byte : guid_) byte = static_cast<uint8_t>(random());
    using org::eclipse::cyclonedds::topic::TopicTraits;
    auto* request_type = TopicTraits<Request>::getSerType();
    request_topic_.reset(dds_create_topic_sertype(participant,
        dds_rpc::request_topic_name(service_).c_str(), &request_type, nullptr, nullptr, nullptr), service_ + " request topic");
    auto* reply_type = TopicTraits<Reply>::getSerType();
    reply_topic_.reset(dds_create_topic_sertype(participant,
        dds_rpc::reply_topic_name(service_).c_str(), &reply_type, nullptr, nullptr, nullptr), service_ + " reply topic");
    const dds_rpc::RpcQosConfig config;
    using Qos = std::unique_ptr<dds_qos_t, decltype(&dds_delete_qos)>;
    Qos writer_qos(config.create_writer_qos(), &dds_delete_qos);
    Qos reader_qos(config.create_reply_reader_qos(), &dds_delete_qos);
    writer_.reset(dds_create_writer(participant, request_topic_.handle, writer_qos.get(), nullptr), service_ + " writer");
    reader_.reset(dds_create_reader(participant, reply_topic_.handle, reader_qos.get(), nullptr), service_ + " reader");
  }

  bool call(Request request, Reply& result, int timeout_ms) {
    using Clock = std::chrono::steady_clock;
    const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
    bool matched = false;
    while (Clock::now() < deadline) {
      dds_publication_matched_status_t publication{};
      dds_subscription_matched_status_t subscription{};
      if (dds_get_publication_matched_status(writer_.handle, &publication) == DDS_RETCODE_OK &&
          dds_get_subscription_matched_status(reader_.handle, &subscription) == DDS_RETCODE_OK &&
          publication.current_count > 0 && subscription.current_count > 0) {
        matched = true; break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!matched) return timedOut("服务发现");
    request.header().request_id().writer_guid(guid_);
    request.header().request_id().sequence_number(++sequence_);
    request.header().instance_name(service_);
    const auto identity = request.header().request_id();
    const auto written = dds_write(writer_.handle, &request);
    if (written != DDS_RETCODE_OK)
      throw std::runtime_error(service_ + " dds_write error=" + std::to_string(written));
    Reply sample{};
    while (Clock::now() < deadline) {
      dds_sample_info_t info{};
      if (takeOwnedReply(reader_.handle, sample, info) == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        continue;
      }
      if (!info.valid_data || sample.header().related_request_id() != identity) continue;
      if (sample.header().remote_ex() != dds_rpc::RemoteExceptionCode::REMOTE_EX_OK)
        throw std::runtime_error(service_ + " remote exception=" +
            std::to_string(static_cast<int>(sample.header().remote_ex())));
      result = sample;
      return true;
    }
    return timedOut("等待回复");
  }

 private:
  bool timedOut(const char* stage) const {
    std::cerr << "RPC 超时: " << service_ << "（" << stage << "）\n";
    return false;
  }
  std::string service_;
  std::array<uint8_t, 16> guid_{};
  int64_t sequence_ = 0;
  Entity request_topic_, reply_topic_, writer_, reader_;
};
}  // namespace tracking_rpc_detail

class TrackingControlRpc {
 public:
  TrackingControlRpc()
      : runtime_(participant(), "ControllerService/GetRuntimeState"),
        controller_(participant_.handle, "ControllerService/GetControllerState"),
        motion_(participant_.handle, "ControllerService/GetMotionState"),
        waist_(participant_.handle, "ControllerService/SetWaistMode"),
        switch_(participant_.handle, "ControllerService/SwitchController"),
        arm_(participant_.handle, "ControllerService/SetArmMode") {}

  bool getRuntimeState(leju::vr::RuntimeState& state, int timeout_ms) {
    leju::srv::GetRuntimeStateReply reply;
    if (!runtime_.call({}, reply, timeout_ms)) return false;
    state.data_ready = reply.data_ready();
    state.hardware_state = static_cast<leju::vr::HardwareState>(reply.hardware_state());
    state.lifecycle_state = static_cast<leju::vr::LifecycleState>(reply.lifecycle_state());
    return true;
  }
  bool getControllerState(leju::vr::ControllerState& state, int timeout_ms) {
    leju::srv::GetControllerStateReply reply;
    if (!controller_.call({}, reply, timeout_ms)) return false;
    state.current_controller = reply.current_controller();
    state.arm_mode = static_cast<leju::vr::ControlMode>(reply.arm_mode());
    state.head_mode = static_cast<leju::vr::ControlMode>(reply.head_mode());
    state.waist_mode = static_cast<leju::vr::ControlMode>(reply.waist_mode());
    state.controller_transitioning = reply.controller_transitioning();
    state.available_controllers = reply.available_controllers();
    return true;
  }
  bool getMotionState(leju::vr::MotionState& state, int timeout_ms) {
    leju::srv::GetMotionStateReply reply;
    if (!motion_.call({}, reply, timeout_ms)) return false;
    state.supported = reply.supported();
    state.motion_playing = reply.motion_playing();
    state.current_motion_name = reply.current_motion_name();
    state.available_motion_names = reply.available_motion_names();
    return true;
  }
  bool setWaistMode(leju::vr::ControlMode mode, int timeout_ms) {
    std_srvs::SetInt32Request request;
    std_srvs::SetInt32Reply reply;
    request.data(static_cast<int32_t>(mode));
    if (!waist_.call(request, reply, timeout_ms)) return false;
    if (!reply.success()) std::cerr << "SetWaistMode 被拒绝: " << reply.message() << '\n';
    return reply.success();
  }
  bool switchController(const std::string& name, int timeout_ms) {
    leju::srv::SwitchControllerRequest request;
    leju::srv::SwitchControllerReply reply;
    request.controller_name(name);
    if (!switch_.call(request, reply, timeout_ms)) return false;
    if (!reply.success()) std::cerr << "SwitchController 被拒绝: " << reply.message() << '\n';
    return reply.success();
  }
  bool setArmMode(leju::vr::ControlMode mode, int timeout_ms) {
    std_srvs::SetInt32Request request;
    std_srvs::SetInt32Reply reply;
    request.data(static_cast<int32_t>(mode));
    if (!arm_.call(request, reply, timeout_ms)) return false;
    if (!reply.success()) std::cerr << "SetArmMode 被拒绝: " << reply.message() << '\n';
    return reply.success();
  }

 private:
  dds_entity_t participant() {
    participant_.reset(dds_create_participant(0, nullptr, nullptr), "tracking RPC participant");
    return participant_.handle;
  }
  // participant 先创建、最后销毁；构造中途失败也释放已经创建的 DDS 实体。
  tracking_rpc_detail::Entity participant_;
  tracking_rpc_detail::Requester<leju::srv::GetRuntimeStateRequest, leju::srv::GetRuntimeStateReply> runtime_;
  tracking_rpc_detail::Requester<leju::srv::GetControllerStateRequest, leju::srv::GetControllerStateReply> controller_;
  tracking_rpc_detail::Requester<leju::srv::GetMotionStateRequest, leju::srv::GetMotionStateReply> motion_;
  tracking_rpc_detail::Requester<std_srvs::SetInt32Request, std_srvs::SetInt32Reply> waist_;
  tracking_rpc_detail::Requester<leju::srv::SwitchControllerRequest, leju::srv::SwitchControllerReply> switch_;
  tracking_rpc_detail::Requester<std_srvs::SetInt32Request, std_srvs::SetInt32Reply> arm_;
};
}  // namespace roban_tracking
