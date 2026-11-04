#pragma once

#include <chrono>
#include <cmath>
#include <string>
#include <vector>

namespace leju::vr_control {

struct ThumbHandFeedback {
  bool hardware_ready = false;
  bool valid = false;
  bool left_valid = false;
  bool right_valid = false;
  unsigned left_age_ms = 0;
  unsigned right_age_ms = 0;
  std::vector<double> position;
  std::chrono::steady_clock::time_point received_at{};
};

struct ThumbHandInput {
  bool x = false;
  bool y = false;
  float left_trigger = 0;
  float right_trigger = 0;
};

struct ThumbHandResult {
  bool published = false;
  bool toggled = false;
  bool closed = false;
  std::string error;
};

// Called under the node's hand mutex. No command is sent on startup/reconnect.
class YThumbHandControl {
 public:
  using Clock = std::chrono::steady_clock;

  template <typename Publish>
  ThumbHandResult update(const ThumbHandInput& input,
                         const ThumbHandFeedback& feedback,
                         Clock::time_point now, Publish&& publish) {
    const bool baseline = !primed_ || now - last_joy_ > std::chrono::milliseconds(500);
    const bool y_pressed = input.y && !previous_y_;
    const bool left_changed = input.left_trigger != previous_left_trigger_;
    const bool right_changed = input.right_trigger != previous_right_trigger_;
    previous_y_ = input.y;
    previous_left_trigger_ = input.left_trigger;
    previous_right_trigger_ = input.right_trigger;
    last_joy_ = now;
    primed_ = true;

    if (baseline) {
      targets_valid_ = false;
      return {};
    }
    // X is a modifier for arm/exit combinations, not a hand-motion command.
    if (input.x) return {};

    const bool requested = y_pressed || left_changed || right_changed;
    std::string problem = feedbackError(feedback, now);
    if (!std::isfinite(input.left_trigger) || !std::isfinite(input.right_trigger) ||
        input.left_trigger < 0 || input.left_trigger > 1 ||
        input.right_trigger < 0 || input.right_trigger > 1) {
      problem = "扳机数据无效";
    }
    if (!problem.empty()) {
      targets_valid_ = false;
      return {false, false, closed_, requested ? problem : std::string{}};
    }
    if (!requested) return {};

    auto positions = targets_valid_ ? targets_ : feedback.position;
    // The left trigger still controls all six channels. A new Y press
    // overrides channel 1 for this frame; the next trigger change takes over.
    if (left_changed) {
      for (int i = 0; i < 6; ++i) positions[i] = input.left_trigger * 100.0;
    }
    if (right_changed) {
      for (int i = 6; i < 12; ++i) positions[i] = input.right_trigger * 100.0;
    }
    const bool next_closed = y_pressed ? !closed_ : closed_;
    if (y_pressed) positions[0] = next_closed ? 100.0 : 0.0;
    if (!publish(positions)) {
      targets_valid_ = false;
      return {false, false, closed_, "SDK 拒绝发送手部目标"};
    }
    targets_ = std::move(positions);
    targets_valid_ = true;
    if (y_pressed) closed_ = next_closed;
    return {true, y_pressed, closed_, {}};
  }

 private:
  static std::string feedbackError(const ThumbHandFeedback& feedback,
                                   Clock::time_point now) {
    if (!feedback.hardware_ready) return "硬件未处于 READY_OK";
    if (!feedback.valid || feedback.position.size() != 12) {
      return "缺少有效的 12 路手部反馈";
    }
    if (now - feedback.received_at > std::chrono::milliseconds(1000)) {
      return "手部反馈已超过 1 秒未更新";
    }
    if (!feedback.left_valid || !feedback.right_valid ||
        feedback.left_age_ms > 1000 || feedback.right_age_ms > 1000) {
      return "左手或右手采样无效或过期";
    }
    for (double value : feedback.position) {
      if (!std::isfinite(value) || value < 0 || value > 100) {
        return "手部反馈位置超出范围";
      }
    }
    return {};
  }

  bool primed_ = false;
  bool previous_y_ = false;
  bool closed_ = false;
  bool targets_valid_ = false;
  float previous_left_trigger_ = 0;
  float previous_right_trigger_ = 0;
  Clock::time_point last_joy_{};
  std::vector<double> targets_;
};

}  // namespace leju::vr_control
