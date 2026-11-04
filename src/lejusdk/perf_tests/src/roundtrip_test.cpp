/**
 * @file roundtrip_test.cpp
 * @brief CycloneDDS 往返延迟测试 (单进程 ping-pong)
 *
 * 使用两个 topic 测量完整的往返时间 (RTT):
 * - /perf/ping: 发布者发送, Pong 订阅者接收
 * - /perf/pong: Pong 订阅者回传, 发布者计算 RTT
 */

#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <mutex>
#include <condition_variable>

#include <dds/dds.hpp>
#include <lejusdk-dds-idl/Float64Types.hpp>
#include <lejusdk-topic-pubsub/topic_publisher.hpp>
#include <lejusdk-topic-pubsub/topic_subscriber.hpp>

#include "perf_utils.hpp"

static std::atomic<bool> g_running{true};

void signal_handler(int) {
    g_running = false;
}

// Pong responder: receives ping and sends pong
class PongResponder {
public:
    PongResponder(dds::domain::DomainParticipant& participant,
                  const std::string& ping_topic,
                  const std::string& pong_topic)
        : pong_publisher_(participant, pong_topic)
        , ping_subscriber_(participant, ping_topic,
            [this](const leju::msgs::Float64Array& msg) { on_ping(msg); },
            [](const std::string& err) { std::cerr << "Pong error: " << err << "\n"; })
    {
    }

    bool is_valid() const {
        return pong_publisher_.is_valid() && ping_subscriber_.is_valid();
    }

    size_t pong_count() const { return pong_count_; }

private:
    void on_ping(const leju::msgs::Float64Array& msg) {
        // Echo the message back immediately
        pong_publisher_.publish(msg);
        pong_count_++;
    }

    leju::dds_common::TopicPublisher<leju::msgs::Float64Array> pong_publisher_;
    leju::dds_common::TopicSubscriber<leju::msgs::Float64Array> ping_subscriber_;
    std::atomic<size_t> pong_count_{0};
};

// Ping sender and RTT measurer
class PingSender {
public:
    PingSender(dds::domain::DomainParticipant& participant,
               const std::string& ping_topic,
               const std::string& pong_topic)
        : ping_publisher_(participant, ping_topic)
        , pong_subscriber_(participant, pong_topic,
            [this](const leju::msgs::Float64Array& msg) { on_pong(msg); },
            [](const std::string& err) { std::cerr << "Ping error: " << err << "\n"; })
    {
    }

    bool is_valid() const {
        return ping_publisher_.is_valid() && pong_subscriber_.is_valid();
    }

    void run_test(const leju::perf::PerfTestArgs& args) {
        // Prepare message (each double is 8 bytes)
        leju::msgs::Float64Array msg;
        size_t num_doubles = (args.msg_size + 7) / 8;
        std::vector<double> payload(num_doubles, 0.0);
        msg.data(payload);

        auto period = std::chrono::nanoseconds(
            static_cast<int64_t>(1e9 / args.rate_hz));

        std::cout << "Running round-trip test...\n";

        size_t total = args.warmup + args.count;
        for (size_t i = 0; i < total && g_running; ++i) {
            bool is_warmup = (i < args.warmup);

            // Record send time using header fields
            int64_t send_time = leju::perf::now_ns();
            msg.header_sec(static_cast<int32_t>(send_time / 1000000000LL));
            msg.header_nanosec(static_cast<uint32_t>(send_time % 1000000000LL));

            // Store for RTT calculation
            {
                std::lock_guard<std::mutex> lock(mutex_);
                pending_send_time_ = send_time;
                pending_idx_ = i;
                is_warmup_ = is_warmup;
                response_received_ = false;
            }

            // Send ping
            ping_publisher_.publish(msg);

            // Wait for pong (with timeout)
            {
                std::unique_lock<std::mutex> lock(mutex_);
                if (!cv_.wait_for(lock, std::chrono::milliseconds(100),
                    [this]{ return response_received_; })) {
                    timeout_count_++;
                }
            }

            // Rate control
            auto elapsed_since_send = leju::perf::now_ns() - send_time;
            auto remaining = period.count() - elapsed_since_send;
            if (remaining > 0) {
                std::this_thread::sleep_for(std::chrono::nanoseconds(remaining));
            }

            // Progress
            if ((i + 1) % 1000 == 0) {
                std::cout << "Completed " << (i + 1) << "/" << total << " round-trips\r" << std::flush;
            }
        }

        std::cout << "\n";
    }

    void print_results(const leju::perf::PerfTestArgs& args) {
        stats_.print_summary("Round-Trip Latency Results",
                             "/perf/ping -> /perf/pong",
                             args.msg_size,
                             args.rate_hz);

        if (timeout_count_ > 0) {
            std::cout << "Timeouts: " << timeout_count_ << "\n";
        }

        stats_.print_histogram(10);
    }

private:
    void on_pong(const leju::msgs::Float64Array& msg) {
        int64_t recv_time = leju::perf::now_ns();

        std::lock_guard<std::mutex> lock(mutex_);

        // Calculate RTT from message timestamp
        int64_t send_time = static_cast<int64_t>(msg.header_sec()) * 1000000000LL +
                            static_cast<int64_t>(msg.header_nanosec());

        if (!is_warmup_ && send_time == pending_send_time_) {
            int64_t rtt = recv_time - send_time;
            stats_.add_sample(rtt);
        }
        response_received_ = true;
        cv_.notify_one();
    }

    leju::dds_common::TopicPublisher<leju::msgs::Float64Array> ping_publisher_;
    leju::dds_common::TopicSubscriber<leju::msgs::Float64Array> pong_subscriber_;

    std::mutex mutex_;
    std::condition_variable cv_;
    int64_t pending_send_time_{0};
    size_t pending_idx_{0};
    bool response_received_{false};
    bool is_warmup_{true};

    leju::perf::LatencyStats stats_;
    size_t timeout_count_{0};
};

int main(int argc, char* argv[]) {
    auto args = leju::perf::PerfTestArgs::parse(argc, argv);
    args.print();

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    const std::string ping_topic = args.topic + "/ping";
    const std::string pong_topic = args.topic + "/pong";

    std::cout << "Ping topic: " << ping_topic << "\n";
    std::cout << "Pong topic: " << pong_topic << "\n\n";

    try {
        dds::domain::DomainParticipant participant(0);
        std::cout << "DDS participant created\n";

        // Create pong responder first (so it's ready when ping arrives)
        PongResponder pong_responder(participant, ping_topic, pong_topic);
        if (!pong_responder.is_valid()) {
            std::cerr << "Failed to create pong responder\n";
            return 1;
        }

        // Create ping sender
        PingSender ping_sender(participant, ping_topic, pong_topic);
        if (!ping_sender.is_valid()) {
            std::cerr << "Failed to create ping sender\n";
            return 1;
        }

        // Wait for discovery
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Run test
        ping_sender.run_test(args);

        // Print results
        std::cout << "\n";
        ping_sender.print_results(args);

        std::cout << "Pong responder processed " << pong_responder.pong_count() << " messages\n";

    } catch (const dds::core::Exception& e) {
        std::cerr << "DDS exception: " << e.what() << "\n";
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
