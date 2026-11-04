#include "lejusdk-lowlevel/leju_sdk.h"
#include <nlohmann/json.hpp>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <set>
#include <string>

namespace {
using Json = nlohmann::json;
struct Client {
    std::mutex mutex;
    std::mutex recovery_mutex;
    std::condition_variable condition;
    Json state = nullptr;
    Json target = Json::object();
    uint64_t pending_recovery = 0;
    Json recovery_result = nullptr;
};
using Handle = std::shared_ptr<Client>;
std::once_flag init_flag;
std::atomic<uint64_t> next_request{static_cast<uint64_t>(
    std::chrono::steady_clock::now().time_since_epoch().count())};
thread_local std::string response;

const char* reply(const Json& value) {
    response = value.dump();
    return response.c_str();
}
}

// C 接口仅适配已有 C++ DDS SDK，不初始化或直接访问硬件。
extern "C" void* leju_claw_create() noexcept {
    try {
        std::call_once(init_flag, [] {
            if (!leju::GlobalRobot::init_env(leju::RobotVersion::from_env()))
                throw std::runtime_error("DDS SDK initialization failed");
        });
        auto client = std::make_shared<Client>();
        std::weak_ptr<Client> weak = client;
        auto& robot = leju::GlobalRobot::getInstance();
        robot.subscribeClawState([weak](const leju::ClawStateConstPtr& state) {
            if (auto client = weak.lock()) {
                if (!state) return;
                std::lock_guard<std::mutex> lock(client->mutex);
                client->state = {{"name", state->name}, {"position", state->position},
                    {"velocity", state->velocity}, {"effort", state->effort},
                    {"state", state->state}, {"is_high_freq", state->is_high_freq}};
            }
        });
        robot.subscribeClawTarget([weak](const leju::ClawTargetConstPtr& target) {
            if (auto client = weak.lock()) {
                if (!target) return;
                std::lock_guard<std::mutex> lock(client->mutex);
                // CAN 目标逐电机上报，按名称合并，不能把右爪帧当作双爪全量状态。
                for (size_t i = 0; i < target->name.size(); ++i) {
                    if (i >= target->position.size() || i >= target->velocity.size() ||
                        i >= target->effort.size() || i >= target->kp.size() || i >= target->kd.size()) continue;
                    client->target[target->name[i]] = {{"position", target->position[i]},
                        {"velocity", target->velocity[i]}, {"effort", target->effort[i]},
                        {"kp", target->zero_kp ? 0 : target->kp[i]}, {"kd", target->kd[i]},
                        {"zero_kp", target->zero_kp}};
                }
            }
        });
        robot.subscribeClawRecoveryResult([weak](const leju::ClawRecoveryResultConstPtr& result) {
            if (auto client = weak.lock()) {
                if (!result) return;
                std::lock_guard<std::mutex> lock(client->mutex);
                if (result->request_id != client->pending_recovery) return;
                client->recovery_result = {{"request_id", result->request_id},
                    {"success", result->success}, {"message", result->message}};
                client->condition.notify_all();
            }
        });
        return new Handle(std::move(client));
    } catch (...) {
        return nullptr;
    }
}

extern "C" void leju_claw_destroy(void* handle) noexcept {
    delete static_cast<Handle*>(handle);
}

extern "C" const char* leju_claw_request(void* handle, const char* input) noexcept {
    try {
        if (!handle || !input) return reply({{"ok", false}, {"error", "invalid client"}});
        auto client = *static_cast<Handle*>(handle);
        const auto request = Json::parse(input);
        const auto op = request.at("op").get<std::string>();
        if (op == "state" || op == "target") {
            std::lock_guard<std::mutex> lock(client->mutex);
            return reply({{"ok", true}, {"data", op == "state" ? client->state : client->target}});
        }
        auto& robot = leju::GlobalRobot::getInstance();
        if (op == "control") {
            leju::ClawCmd cmd;
            cmd.name = request.at("name").get<std::vector<std::string>>();
            cmd.position = request.at("position").get<std::vector<double>>();
            cmd.velocity = request.value("velocity", std::vector<double>{});
            cmd.effort = request.value("effort", std::vector<double>{});
            cmd.is_high_freq = request.value("is_high_freq", false);
            // C ABI 可能被非 Python 调用方直接使用，必须在这里拒绝错位或重复参数。
            if (cmd.name.empty() || cmd.position.size() != cmd.name.size() ||
                (!cmd.velocity.empty() && cmd.velocity.size() != cmd.name.size()) ||
                (!cmd.effort.empty() && cmd.effort.size() != cmd.name.size())) {
                return reply({{"ok", false}, {"error", "name/position/velocity/effort length mismatch"}});
            }
            if (std::set<std::string>(cmd.name.begin(), cmd.name.end()).size() != cmd.name.size()) {
                return reply({{"ok", false}, {"error", "duplicate claw name"}});
            }
            return reply({{"ok", robot.publishClawCmd(cmd)}});
        }
        if (op == "recover") {
            // 同一客户端串行等待恢复结果；跨客户端接管由原控制器判断。
            std::lock_guard<std::mutex> serial(client->recovery_mutex);
            leju::ClawRecovery cmd;
            cmd.request_id = ++next_request;
            cmd.direction = request.at("direction").get<std::string>();
            cmd.current = request.value("current", 1.5);
            cmd.duration_ms = request.value("duration_ms", 100);
            cmd.repeat = request.value("repeat", 1);
            {
                std::lock_guard<std::mutex> lock(client->mutex);
                client->pending_recovery = cmd.request_id;
                client->recovery_result = nullptr;
            }
            if (!robot.publishClawRecovery(cmd)) return reply({{"ok", false}, {"error", "DDS publish failed"}});
            std::unique_lock<std::mutex> lock(client->mutex);
            if (!client->condition.wait_for(lock, std::chrono::seconds(10), [&] {
                return !client->recovery_result.is_null();
            })) return reply({{"ok", false}, {"error", "recovery result timeout"}});
            return reply({{"ok", true}, {"data", client->recovery_result}});
        }
        return reply({{"ok", false}, {"error", "unknown operation"}});
    } catch (const std::exception& error) {
        return reply({{"ok", false}, {"error", error.what()}});
    } catch (...) {
        return reply({{"ok", false}, {"error", "claw client failed"}});
    }
}
