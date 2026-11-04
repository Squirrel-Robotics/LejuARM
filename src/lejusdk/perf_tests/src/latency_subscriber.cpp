/**
 * @file latency_subscriber.cpp
 * @brief CycloneDDS 延迟测试订阅者
 *
 * 订阅 Float64Array 并计算延迟统计信息。
 */

#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <mutex>

#include <dds/dds.hpp>
#include <lejusdk-dds-idl/Float64Types.hpp>
#include <lejusdk-topic-pubsub/topic_subscriber.hpp>

#include "perf_utils.hpp"

static std::atomic<bool> g_running{true};
static std::mutex g_stats_mutex;
static leju::perf::LatencyStats g_stats;
static size_t g_msg_size = 0;
static size_t g_warmup_remaining = 0;

void signal_handler(int) {
    g_running = false;
}

void on_data_received(const leju::msgs::Float64Array& msg) {
    int64_t recv_time = leju::perf::now_ns();
    int64_t send_time = static_cast<int64_t>(msg.header_sec()) * 1000000000LL +
                        static_cast<int64_t>(msg.header_nanosec());
    int64_t latency = recv_time - send_time;

    std::lock_guard<std::mutex> lock(g_stats_mutex);

    // Skip warmup messages
    if (g_warmup_remaining > 0) {
        g_warmup_remaining--;
        return;
    }

    // Record message size once (data size in bytes = num_doubles * 8)
    if (g_msg_size == 0) {
        g_msg_size = msg.data().size() * sizeof(double);
    }

    // Add sample
    g_stats.add_sample(latency);

    // Progress report every 1000 messages
    if (g_stats.count() % 1000 == 0) {
        std::cout << "Received " << g_stats.count() << " messages, "
                  << "current latency: " << (latency / 1000.0) << " us\r" << std::flush;
    }
}

void on_error(const std::string& error) {
    std::cerr << "Subscriber error: " << error << "\n";
}

int main(int argc, char* argv[]) {
    // Parse arguments
    auto args = leju::perf::PerfTestArgs::parse(argc, argv);
    g_warmup_remaining = args.warmup;
    args.print();

    // Setup signal handler
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    try {
        // Create DDS participant
        dds::domain::DomainParticipant participant(0);
        std::cout << "DDS participant created\n";

        // Create subscriber with RELIABLE QoS
        dds::sub::qos::DataReaderQos qos;
        qos << dds::core::policy::Reliability::Reliable();

        leju::dds_common::TopicSubscriber<leju::msgs::Float64Array> subscriber(
            participant, args.topic, on_data_received, on_error, qos);

        if (!subscriber.is_valid()) {
            std::cerr << "Failed to create subscriber\n";
            return 1;
        }

        std::cout << "Subscriber ready. Waiting for messages on " << args.topic << "...\n";
        std::cout << "(Press Ctrl+C to stop and show results)\n\n";

        // Wait for publisher
        if (!subscriber.wait_for_publishers(1, 10000)) {
            std::cerr << "Warning: No publisher found within timeout, continuing anyway...\n";
        }

        // Wait for messages
        while (g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            // Auto-stop after receiving expected count
            if (args.count > 0 && g_stats.count() >= args.count) {
                break;
            }
        }

        // Print results
        std::cout << "\n\n";
        {
            std::lock_guard<std::mutex> lock(g_stats_mutex);

            if (g_stats.empty()) {
                std::cout << "No messages received.\n";
            } else {
                g_stats.print_summary("Latency Test Results",
                                      args.topic,
                                      g_msg_size,
                                      args.rate_hz);

                g_stats.print_histogram(10);
            }
        }

    } catch (const dds::core::Exception& e) {
        std::cerr << "DDS exception: " << e.what() << "\n";
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
