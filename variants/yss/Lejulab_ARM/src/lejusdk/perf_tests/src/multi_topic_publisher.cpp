/**
 * @file multi_topic_publisher.cpp
 * @brief 多 topic 发布者，用于跨进程延迟测试
 *
 * 在同一循环迭代中发布 JointState + ImuData。
 * 需要与 multi_topic_subscriber 在独立进程中配合运行。
 */

#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>

#include <dds/dds.hpp>
#include <lejusdk-dds-idl/JointState.hpp>
#include <lejusdk-dds-idl/ImuData.hpp>
#include <lejusdk-topic-pubsub/topic_publisher.hpp>

#include "perf_utils.hpp"

static std::atomic<bool> g_running{true};
constexpr size_t kJointDim = 28;

void signal_handler(int) {
    g_running = false;
}

int main(int argc, char* argv[]) {
    auto args = leju::perf::PerfTestArgs::parse(argc, argv);
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

        // Create publishers
        leju::dds_common::TopicPublisher<leju::msgs::JointState> joint_pub(participant, joint_topic);
        leju::dds_common::TopicPublisher<leju::msgs::ImuData> imu_pub(participant, imu_topic);

        if (!joint_pub.is_valid() || !imu_pub.is_valid()) {
            std::cerr << "Failed to create publishers\n";
            return 1;
        }

        // Wait for subscriber
        std::cout << "Waiting for subscriber...\n";
        if (!joint_pub.wait_for_subscribers(1, 30000)) {
            std::cerr << "No subscriber found within timeout\n";
            return 1;
        }
        std::cout << "Subscriber connected\n";

        // Prepare JointState message
        leju::msgs::JointState joint_msg;
        joint_msg.q(std::vector<double>(kJointDim, 0.0));
        joint_msg.v(std::vector<double>(kJointDim, 0.0));
        joint_msg.vd(std::vector<double>(kJointDim, 0.0));
        joint_msg.tau(std::vector<double>(kJointDim, 0.0));

        // Prepare ImuData message
        leju::msgs::ImuData imu_msg;
        std::array<double, 3> zeros3 = {0.0, 0.0, 9.8};
        std::array<double, 4> quat = {1.0, 0.0, 0.0, 0.0};
        imu_msg.gyro(zeros3);
        imu_msg.acc(zeros3);
        imu_msg.free_acc(zeros3);
        imu_msg.quat(quat);

        // Statistics for publish execution time
        leju::perf::LatencyStats joint_pub_stats;
        leju::perf::LatencyStats imu_pub_stats;

        auto period = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / args.rate_hz));

        std::cout << "Publishing JointState + ImuData at " << args.rate_hz << " Hz...\n";

        size_t total = args.warmup + args.count;
        for (size_t i = 0; i < total && g_running; ++i) {
            auto loop_start = std::chrono::steady_clock::now();
            bool is_warmup = (i < args.warmup);

            // Publish JointState
            int64_t t1 = leju::perf::now_ns();
            int32_t sec1 = static_cast<int32_t>(t1 / 1000000000LL);
            uint32_t nsec1 = static_cast<uint32_t>(t1 % 1000000000LL);
            joint_msg.header_sec(sec1);
            joint_msg.header_nanosec(nsec1);
            joint_pub.publish(joint_msg);
            int64_t t2 = leju::perf::now_ns();

            // Publish ImuData
            int64_t t3 = leju::perf::now_ns();
            int32_t sec3 = static_cast<int32_t>(t3 / 1000000000LL);
            uint32_t nsec3 = static_cast<uint32_t>(t3 % 1000000000LL);
            imu_msg.header_sec(sec3);
            imu_msg.header_nanosec(nsec3);
            imu_pub.publish(imu_msg);
            int64_t t4 = leju::perf::now_ns();

            // Record stats (skip warmup)
            if (!is_warmup) {
                joint_pub_stats.add_sample(t2 - t1);
                imu_pub_stats.add_sample(t4 - t3);
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

        // Print publish execution time stats
        std::cout << "\n";
        joint_pub_stats.print_summary("JointState publish() Time", joint_topic);
        imu_pub_stats.print_summary("ImuData publish() Time", imu_topic);

        // Give subscriber time to process
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

    } catch (const dds::core::Exception& e) {
        std::cerr << "DDS exception: " << e.what() << "\n";
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
