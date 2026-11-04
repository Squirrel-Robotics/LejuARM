#pragma once

#include <lejusdk-lowlevel/robot_api/robot_base.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <optional>
#include <chrono>
#include <mutex>

namespace leju::vr_control {

// 两个 node 共用同一末端判定口径：首次确认夹爪后不回退；夹爪状态短暂缺失时
// 只暂停夹爪输出，不能切换到灵巧手。此状态机不依赖 DDS publish 的返回值。
class QuestClawRouting {
public:
    explicit QuestClawRouting(bool lock_hand = false) : lock_hand_(lock_hand) {}

    void set_lock_hand(bool lock) { lock_hand_.store(lock); }
    bool lock_hand() const { return lock_hand_.load(); }

    // 只在首次由上升沿（laser）确认双爪后锁定末端类型；TSM 的任一部位故障
    // 属于瞬时或单侧故障，继续只停 ClawCmd，不能误切到 HandCmd。
    void on_claw_state(const ClawStateConstPtr& state) {
        const bool valid = state && !state->name.empty() && state->name.size() != 0 &&
                           state->name.size() == state->position.size() &&
                           std::all_of(state->name.begin(), state->name.end(),
                                       [](const std::string& name) {
                                           return name == "left_claw" || name == "right_claw";
                                       });
        const bool both = valid && state->name.size() == 2 &&
                          state->name[0] != state->name[1];
        if (valid) {
            std::lock_guard<std::mutex> lock(mutex_);
            last_valid_ = std::chrono::steady_clock::now();
            if (both) ever_confirmed_ = true;
        }
        available_.store(both);
    }

    // 只用于提示反馈中断，不参与末端回退；一帧空状态不会改变已确认的夹爪身份。
    bool feedback_stale(std::chrono::milliseconds timeout) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return ever_confirmed_ && last_valid_ != std::chrono::steady_clock::time_point{} &&
               std::chrono::steady_clock::now() - last_valid_ >= timeout;
    }

    // VR 有夹爪时只输出 ClawCmd；已确认过的会话绝不因一帧空状态改发 HandCmd。
    bool should_send_claw_cmd() const { return !lock_hand() && available_.load(); }
    bool should_send_hand_cmd() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return !lock_hand() && !ever_confirmed_;
    }

private:
    std::atomic<bool> lock_hand_{false};
    std::atomic<bool> available_{false};
    std::atomic<bool> ever_confirmed_{false};
    mutable std::mutex mutex_;
    std::chrono::steady_clock::time_point last_valid_{};
};


inline std::optional<ClawCmd> makeQuestClawCommand(float left_trigger, float right_trigger) {
    // 浮点非法值不能转换为整数位置，也不应刷新高频租约。
    if (!std::isfinite(left_trigger) || !std::isfinite(right_trigger)) return std::nullopt;
    ClawCmd cmd;
    cmd.name = {"left_claw", "right_claw"};
    // 对齐 kuavo-ros-control 的 JoyStickHandler::processClawDataWithJoystick：
    // 扳机乘 100 后截断取整、限幅到 0~100，不额外加死区。保持原有 static_cast
    // 语义，只修正端点：0/1 不能因浮点误差送成 -1/101 后误钳到 5/95。
    const auto to_percent = [](float trigger) {
        const float percent = std::clamp(trigger, 0.0f, 1.0f) * 100.0f;
        return static_cast<double>(static_cast<int>(std::clamp(percent, 0.0f, 100.0f)));
    };
    cmd.position = {to_percent(left_trigger), to_percent(right_trigger)};
    cmd.velocity = {90.0, 90.0};
    cmd.effort = {0.0, 0.0};  // 沿用源输入，控制器按原规则取默认 1A。
    cmd.is_high_freq = true;
    return cmd;
}

}  // namespace leju::vr_control
