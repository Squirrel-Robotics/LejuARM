/**
 * @file latency_publisher.cpp
 * @brief CycloneDDS 延迟测试发布者
 *
 * 发布带有时间戳的 Float64Array 用于延迟测量。
 */

#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>

#include <dds/dds.hpp>
#include <lejusdk-dds-idl/Float64Types.hpp>
#include <lejusdk-topic-pubsub/topic_publisher.hpp>

#include "perf_utils.hpp"

static std::atomic<bool> g_running{true};

void signal_handler(int) {
    g_running = false;
}

int main(int argc, char* argv[]) {
    // Parse arguments
    auto args = leju::perf::PerfTestArgs::parse(argc, argv);
    args.print();

    // Setup signal handler
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    try {
        // Create DDS participant
        dds::domain::DomainParticipant participant(0);
        std::cout << "DDS participant created\n";

        // Create publisher with RELIABLE QoS
        dds::pub::qos::DataWriterQos qos;
        qos << dds::core::policy::Reliability::Reliable();

        leju::dds_common::TopicPublisher<leju::msgs::Float64Array> publisher(
            participant, args.topic, qos);

        if (!publisher.is_valid()) {
            std::cerr << "Failed to create publisher\n";
            return 1;
        }

        // Wait for subscriber
        std::cout << "Waiting for subscriber...\n";
        if (!publisher.wait_for_subscribers(1, 10000)) {
            std::cerr << "No subscriber found within timeout\n";
            return 1;
        }
        std::cout << "Subscriber connected\n";

        // Prepare message with payload (each double is 8 bytes)
        leju::msgs::Float64Array msg;
        size_t num_doubles = (args.msg_size + 7) / 8;  // Round up
        std::vector<double> payload(num_doubles, 0.0);
        msg.data(payload);

        // Calculate sleep duration
        auto period = std::chrono::nanoseconds(
            static_cast<int64_t>(1e9 / args.rate_hz));

        std::cout << "Starting to publish " << args.count << " messages...\n";

        size_t total_messages = args.warmup + args.count;

        for (size_t i = 0; i < total_messages && g_running; ++i) {
            auto start = std::chrono::steady_clock::now();

            // Set timestamp using header fields
            int64_t now = leju::perf::now_ns();
            msg.header_sec(static_cast<int32_t>(now / 1000000000LL));
            msg.header_nanosec(static_cast<uint32_t>(now % 1000000000LL));

            // Publish
            publisher.publish(msg);

            // Progress report
            if ((i + 1) % 1000 == 0) {
                std::cout << "Published " << (i + 1) << "/" << total_messages << " messages\r" << std::flush;
            }

            // Rate control
            auto elapsed = std::chrono::steady_clock::now() - start;
            if (elapsed < period) {
                std::this_thread::sleep_for(period - elapsed);
            }
        }

        std::cout << "\nPublishing complete. Sent " << total_messages << " messages.\n";

        // Give subscriber time to process remaining messages
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
