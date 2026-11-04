/**
 * @file multi_topic_test.cpp
 * @brief 多 topic 延迟测试，模拟机器人控制循环
 *
 * 模拟真实机器人场景：在同一循环迭代中发布 JointState + ImuData。
 * 测量两个 topic 的端到端延迟。
 */

#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <mutex>

#include <dds/dds.hpp>
#include <lejusdk-dds-idl/JointState.hpp>
#include <lejusdk-dds-idl/ImuData.hpp>
#include <lejusdk-topic-pubsub/topic_publisher.hpp>
#include <lejusdk-topic-pubsub/topic_subscriber.hpp>

#include "perf_utils.hpp"

static std::atomic<bool> g_running{true};
constexpr size_t kJointDim = 28;

void signal_handler(int) {
    g_running = false;
}

// Subscriber that measures latency for both topics
class MultiTopicSubscriber {
public:
    MultiTopicSubscriber(dds::domain::DomainParticipant& participant,
                         const std::string& joint_topic,
                         const std::string& imu_topic,
                         size_t warmup = 100)
        : warmup_(warmup)
        , joint_sub_(participant, joint_topic,
            [this](const leju::msgs::JointState& msg) { on_joint_state(msg); },
            [](const std::string& err) { std::cerr << "JointState error: " << err << "\n"; })
        , imu_sub_(participant, imu_topic,
            [this](const leju::msgs::ImuData& msg) { on_imu_data(msg); },
            [](const std::string& err) { std::cerr << "ImuData error: " << err << "\n"; })
    {
    }

    bool is_valid() const {
        return joint_sub_.is_valid() && imu_sub_.is_valid();
    }

    void print_results(double rate_hz) {
        std::cout << "\n";
        joint_stats_.print_summary("JointState End-to-End Latency", "/rt/joint_state", 0, rate_hz);
        joint_stats_.print_histogram(10);

        std::cout << "\n";
        imu_stats_.print_summary("ImuData End-to-End Latency", "/rt/imu_state", 0, rate_hz);
        imu_stats_.print_histogram(10);
    }

    size_t joint_count() const { return joint_recv_count_; }
    size_t imu_count() const { return imu_recv_count_; }

private:
    void on_joint_state(const leju::msgs::JointState& msg) {
        int64_t recv_time = leju::perf::now_ns();
        int64_t send_time = static_cast<int64_t>(msg.header_sec()) * 1000000000LL +
                            static_cast<int64_t>(msg.header_nanosec());
        int64_t latency = recv_time - send_time;

        std::lock_guard<std::mutex> lock(mutex_);
        joint_recv_count_++;
        // Skip warmup
        if (joint_recv_count_ > warmup_) {
            joint_stats_.add_sample(latency);
        }
    }

    void on_imu_data(const leju::msgs::ImuData& msg) {
        int64_t recv_time = leju::perf::now_ns();
        int64_t send_time = static_cast<int64_t>(msg.header_sec()) * 1000000000LL +
                            static_cast<int64_t>(msg.header_nanosec());
        int64_t latency = recv_time - send_time;

        std::lock_guard<std::mutex> lock(mutex_);
        imu_recv_count_++;
        // Skip warmup
        if (imu_recv_count_ > warmup_) {
            imu_stats_.add_sample(latency);
        }
    }

    size_t warmup_;
    size_t joint_recv_count_{0};
    size_t imu_recv_count_{0};

    leju::dds_common::TopicSubscriber<leju::msgs::JointState> joint_sub_;
    leju::dds_common::TopicSubscriber<leju::msgs::ImuData> imu_sub_;

    std::mutex mutex_;
    leju::perf::LatencyStats joint_stats_;
    leju::perf::LatencyStats imu_stats_;
};

// Publisher that simulates robot control loop
class MultiTopicPublisher {
public:
    MultiTopicPublisher(dds::domain::DomainParticipant& participant,
                        const std::string& joint_topic,
                        const std::string& imu_topic)
        : joint_pub_(participant, joint_topic)
        , imu_pub_(participant, imu_topic)
    {
    }

    bool is_valid() const {
        return joint_pub_.is_valid() && imu_pub_.is_valid();
    }

    void run(const leju::perf::PerfTestArgs& args) {
        // Prepare JointState message
        leju::msgs::JointState joint_msg;
        joint_msg.q(std::vector<double>(kJointDim, 0.0));
        joint_msg.v(std::vector<double>(kJointDim, 0.0));
        joint_msg.vd(std::vector<double>(kJointDim, 0.0));
        joint_msg.tau(std::vector<double>(kJointDim, 0.0));

        // Prepare ImuData message
        leju::msgs::ImuData imu_msg;
        std::array<double, 3> zeros3 = {0.0, 0.0, 0.0};
        std::array<double, 4> zeros4 = {1.0, 0.0, 0.0, 0.0};
        imu_msg.gyro(zeros3);
        imu_msg.acc(zeros3);
        imu_msg.free_acc(zeros3);
        imu_msg.quat(zeros4);

        auto period = std::chrono::nanoseconds(
            static_cast<int64_t>(1e9 / args.rate_hz));

        std::cout << "Publishing JointState + ImuData at " << args.rate_hz << " Hz...\n";

        size_t total = args.warmup + args.count;
        bool is_warmup = true;

        for (size_t i = 0; i < total && g_running; ++i) {
            auto loop_start = std::chrono::steady_clock::now();
            is_warmup = (i < args.warmup);

            // Publish JointState - 时间戳在 publish 前一刻设置
            int64_t t1 = leju::perf::now_ns();
            int32_t sec1 = static_cast<int32_t>(t1 / 1000000000LL);
            uint32_t nsec1 = static_cast<uint32_t>(t1 % 1000000000LL);
            joint_msg.header_sec(sec1);
            joint_msg.header_nanosec(nsec1);
            joint_pub_.publish(joint_msg);
            int64_t t2 = leju::perf::now_ns();

            // Publish ImuData - 时间戳在 publish 前一刻设置
            int64_t t3 = leju::perf::now_ns();
            int32_t sec3 = static_cast<int32_t>(t3 / 1000000000LL);
            uint32_t nsec3 = static_cast<uint32_t>(t3 % 1000000000LL);
            imu_msg.header_sec(sec3);
            imu_msg.header_nanosec(nsec3);
            imu_pub_.publish(imu_msg);
            int64_t t4 = leju::perf::now_ns();

            // Record publish time (skip warmup)
            if (!is_warmup) {
                joint_publish_stats_.add_sample(t2 - t1);
                imu_publish_stats_.add_sample(t4 - t3);
            }

            // Progress
            if ((i + 1) % 1000 == 0) {
                std::cout << "Published " << (i + 1) << "/" << total << " cycles\r" << std::flush;
            }

            // Rate control
            auto elapsed = std::chrono::steady_clock::now() - loop_start;
            if (elapsed < period) {
                std::this_thread::sleep_for(period - elapsed);
            }
        }

        std::cout << "\nPublishing complete.\n";
    }

    void print_publish_stats() {
        std::cout << "\n";
        joint_publish_stats_.print_summary("JointState publish() Execution Time", "/rt/joint_state");
        joint_publish_stats_.print_histogram(10);

        std::cout << "\n";
        imu_publish_stats_.print_summary("ImuData publish() Execution Time", "/rt/imu_state");
        imu_publish_stats_.print_histogram(10);
    }

private:
    leju::dds_common::TopicPublisher<leju::msgs::JointState> joint_pub_;
    leju::dds_common::TopicPublisher<leju::msgs::ImuData> imu_pub_;

    leju::perf::LatencyStats joint_publish_stats_;
    leju::perf::LatencyStats imu_publish_stats_;
};

int main(int argc, char* argv[]) {
    auto args = leju::perf::PerfTestArgs::parse(argc, argv);
    args.print();

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    const std::string joint_topic = "/rt/joint_state";
    const std::string imu_topic = "/rt/imu_state";

    std::cout << "JointState topic: " << joint_topic << "\n";
    std::cout << "ImuData topic: " << imu_topic << "\n\n";

    try {
        dds::domain::DomainParticipant participant(0);
        std::cout << "DDS participant created\n";

        // Create subscriber first (with warmup count)
        MultiTopicSubscriber subscriber(participant, joint_topic, imu_topic, args.warmup);
        if (!subscriber.is_valid()) {
            std::cerr << "Failed to create subscriber\n";
            return 1;
        }

        // Create publisher
        MultiTopicPublisher publisher(participant, joint_topic, imu_topic);
        if (!publisher.is_valid()) {
            std::cerr << "Failed to create publisher\n";
            return 1;
        }

        // Wait for discovery
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Run test
        publisher.run(args);

        // Wait for remaining messages
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Print results
        std::cout << "\n";
        std::cout << "Received: JointState=" << subscriber.joint_count()
                  << ", ImuData=" << subscriber.imu_count() << "\n";

        // Print publish() execution time stats
        publisher.print_publish_stats();

        // Print end-to-end latency stats
        subscriber.print_results(args.rate_hz);

    } catch (const dds::core::Exception& e) {
        std::cerr << "DDS exception: " << e.what() << "\n";
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
