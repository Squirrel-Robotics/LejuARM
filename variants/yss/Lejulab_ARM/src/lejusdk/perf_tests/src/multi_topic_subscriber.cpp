/**
 * @file multi_topic_subscriber.cpp
 * @brief 多 topic 订阅者，用于跨进程延迟测试
 *
 * 订阅 JointState + ImuData 并测量端到端延迟。
 * 需要与 multi_topic_publisher 在独立进程中配合运行。
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
#include <lejusdk-topic-pubsub/topic_subscriber.hpp>

#include "perf_utils.hpp"

static std::atomic<bool> g_running{true};
static std::mutex g_mutex;
static leju::perf::LatencyStats g_joint_stats;
static leju::perf::LatencyStats g_imu_stats;
static size_t g_joint_count = 0;
static size_t g_imu_count = 0;
static size_t g_warmup = 100;
static size_t g_target_count = 0;

void signal_handler(int) {
    g_running = false;
}

void on_joint_state(const leju::msgs::JointState& msg) {
    int64_t recv_time = leju::perf::now_ns();
    int64_t send_time = static_cast<int64_t>(msg.header_sec()) * 1000000000LL +
                        static_cast<int64_t>(msg.header_nanosec());
    int64_t latency = recv_time - send_time;

    std::lock_guard<std::mutex> lock(g_mutex);
    g_joint_count++;
    if (g_joint_count > g_warmup) {
        g_joint_stats.add_sample(latency);
    }

    // Progress
    if (g_joint_count % 1000 == 0) {
        std::cout << "Received JointState: " << g_joint_count
                  << ", latency: " << (latency / 1000.0) << " us\r" << std::flush;
    }
}

void on_imu_data(const leju::msgs::ImuData& msg) {
    int64_t recv_time = leju::perf::now_ns();
    int64_t send_time = static_cast<int64_t>(msg.header_sec()) * 1000000000LL +
                        static_cast<int64_t>(msg.header_nanosec());
    int64_t latency = recv_time - send_time;

    std::lock_guard<std::mutex> lock(g_mutex);
    g_imu_count++;
    if (g_imu_count > g_warmup) {
        g_imu_stats.add_sample(latency);
    }
}

int main(int argc, char* argv[]) {
    auto args = leju::perf::PerfTestArgs::parse(argc, argv);
    g_warmup = args.warmup;
    g_target_count = args.count;
    args.print();

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    const std::string joint_topic = "/perf/joint_state";
    const std::string imu_topic = "/perf/imu_state";

    std::cout << "JointState topic: " << joint_topic << "\n";
    std::cout << "ImuData topic: " << imu_topic << "\n\n";

    try {
        dds::domain::DomainParticipant participant(0);
        std::cout << "DDS participant created\n";

        // Create subscribers
        leju::dds_common::TopicSubscriber<leju::msgs::JointState> joint_sub(
            participant, joint_topic, on_joint_state,
            [](const std::string& err) { std::cerr << "JointState error: " << err << "\n"; });

        leju::dds_common::TopicSubscriber<leju::msgs::ImuData> imu_sub(
            participant, imu_topic, on_imu_data,
            [](const std::string& err) { std::cerr << "ImuData error: " << err << "\n"; });

        if (!joint_sub.is_valid() || !imu_sub.is_valid()) {
            std::cerr << "Failed to create subscribers\n";
            return 1;
        }

        std::cout << "Subscriber ready. Waiting for messages...\n";
        std::cout << "(Press Ctrl+C to stop and show results)\n\n";

        // Wait for messages
        while (g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            // Auto-stop after receiving target count
            std::lock_guard<std::mutex> lock(g_mutex);
            if (g_target_count > 0 && g_joint_stats.count() >= g_target_count) {
                break;
            }
        }

        // Print results
        std::cout << "\n\n";
        std::cout << "========================================\n";
        std::cout << "Cross-Process Latency Test Results\n";
        std::cout << "========================================\n";
        std::cout << "Received: JointState=" << g_joint_count
                  << ", ImuData=" << g_imu_count << "\n";

        g_joint_stats.print_summary("JointState End-to-End Latency", joint_topic, 0, args.rate_hz);
        g_joint_stats.print_histogram(10);

        std::cout << "\n";
        g_imu_stats.print_summary("ImuData End-to-End Latency", imu_topic, 0, args.rate_hz);
        g_imu_stats.print_histogram(10);

    } catch (const dds::core::Exception& e) {
        std::cerr << "DDS exception: " << e.what() << "\n";
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
