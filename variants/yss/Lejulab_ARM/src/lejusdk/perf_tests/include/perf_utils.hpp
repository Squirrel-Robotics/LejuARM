#ifndef LEJU_PERF_UTILS_HPP_
#define LEJU_PERF_UTILS_HPP_

#include <chrono>
#include <cstdint>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <string>
#include <numeric>

namespace leju {
namespace perf {

/**
 * @brief Get current time in nanoseconds using system_clock (for cross-process sync)
 */
inline int64_t now_ns() {
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()).count();
}

/**
 * @brief Latency statistics collector and calculator
 */
class LatencyStats {
public:
    LatencyStats() = default;

    void add_sample(int64_t latency_ns) {
        samples_.push_back(latency_ns);
    }

    void clear() {
        samples_.clear();
    }

    size_t count() const {
        return samples_.size();
    }

    bool empty() const {
        return samples_.empty();
    }

    // Basic statistics (in nanoseconds)
    int64_t min_ns() const {
        if (samples_.empty()) return 0;
        return *std::min_element(samples_.begin(), samples_.end());
    }

    int64_t max_ns() const {
        if (samples_.empty()) return 0;
        return *std::max_element(samples_.begin(), samples_.end());
    }

    double mean_ns() const {
        if (samples_.empty()) return 0.0;
        double sum = std::accumulate(samples_.begin(), samples_.end(), 0.0);
        return sum / samples_.size();
    }

    double median_ns() const {
        if (samples_.empty()) return 0.0;
        std::vector<int64_t> sorted = samples_;
        std::sort(sorted.begin(), sorted.end());
        size_t mid = sorted.size() / 2;
        if (sorted.size() % 2 == 0) {
            return (sorted[mid - 1] + sorted[mid]) / 2.0;
        }
        return sorted[mid];
    }

    double percentile_ns(double p) const {
        if (samples_.empty()) return 0.0;
        std::vector<int64_t> sorted = samples_;
        std::sort(sorted.begin(), sorted.end());
        size_t idx = static_cast<size_t>(p / 100.0 * (sorted.size() - 1));
        return sorted[idx];
    }

    double p95_ns() const { return percentile_ns(95.0); }
    double p99_ns() const { return percentile_ns(99.0); }

    double stddev_ns() const {
        if (samples_.size() < 2) return 0.0;
        double mean = mean_ns();
        double sum_sq = 0.0;
        for (auto s : samples_) {
            double diff = s - mean;
            sum_sq += diff * diff;
        }
        return std::sqrt(sum_sq / (samples_.size() - 1));
    }

    double jitter_ns() const {
        if (samples_.size() < 2) return 0.0;
        double sum_diff = 0.0;
        for (size_t i = 1; i < samples_.size(); ++i) {
            sum_diff += std::abs(samples_[i] - samples_[i - 1]);
        }
        return sum_diff / (samples_.size() - 1);
    }

    // Conversion helpers (nanoseconds to microseconds)
    double min_us() const { return min_ns() / 1000.0; }
    double max_us() const { return max_ns() / 1000.0; }
    double mean_us() const { return mean_ns() / 1000.0; }
    double median_us() const { return median_ns() / 1000.0; }
    double p95_us() const { return p95_ns() / 1000.0; }
    double p99_us() const { return p99_ns() / 1000.0; }
    double stddev_us() const { return stddev_ns() / 1000.0; }
    double jitter_us() const { return jitter_ns() / 1000.0; }

    void print_summary(const std::string& title,
                       const std::string& topic = "",
                       size_t msg_size = 0,
                       double rate_hz = 0.0) const {
        std::cout << "\n";
        std::cout << "============ " << title << " ============\n";
        if (!topic.empty()) {
            std::cout << "Topic: " << topic << "\n";
        }
        if (msg_size > 0) {
            std::cout << "Message Size: " << msg_size << " bytes\n";
        }
        if (rate_hz > 0) {
            std::cout << "Publish Rate: " << rate_hz << " Hz\n";
        }
        std::cout << "Messages: " << count() << "\n";
        std::cout << "----------------------------------------------\n";
        std::cout << "Latency Statistics (microseconds):\n";
        std::cout << std::fixed << std::setprecision(1);
        std::cout << "  Min:      " << std::setw(8) << min_us() << " us\n";
        std::cout << "  Max:      " << std::setw(8) << max_us() << " us\n";
        std::cout << "  Mean:     " << std::setw(8) << mean_us() << " us\n";
        std::cout << "  Median:   " << std::setw(8) << median_us() << " us\n";
        std::cout << "  P95:      " << std::setw(8) << p95_us() << " us\n";
        std::cout << "  P99:      " << std::setw(8) << p99_us() << " us\n";
        std::cout << "  Std Dev:  " << std::setw(8) << stddev_us() << " us\n";
        std::cout << "  Jitter:   " << std::setw(8) << jitter_us() << " us\n";
        std::cout << "==============================================\n";
    }

    void print_histogram(int bins = 10) const {
        if (samples_.empty()) return;

        int64_t min_val = min_ns();
        int64_t max_val = max_ns();
        int64_t range = max_val - min_val;
        if (range == 0) range = 1;

        std::vector<size_t> histogram(bins, 0);
        for (auto s : samples_) {
            int bin = static_cast<int>((s - min_val) * bins / (range + 1));
            if (bin >= bins) bin = bins - 1;
            histogram[bin]++;
        }

        size_t max_count = *std::max_element(histogram.begin(), histogram.end());
        const int bar_width = 40;

        std::cout << "\nHistogram:\n";
        for (int i = 0; i < bins; ++i) {
            double low = min_val / 1000.0 + i * range / 1000.0 / bins;
            double high = min_val / 1000.0 + (i + 1) * range / 1000.0 / bins;
            int bar_len = max_count > 0 ? static_cast<int>(histogram[i] * bar_width / max_count) : 0;

            std::cout << std::fixed << std::setprecision(0);
            std::cout << "  [" << std::setw(6) << low << "-" << std::setw(6) << high << " us]: ";
            std::cout << std::string(bar_len, '#');
            std::cout << " " << histogram[i] << " ("
                      << std::fixed << std::setprecision(1)
                      << (100.0 * histogram[i] / samples_.size()) << "%)\n";
        }
    }

private:
    std::vector<int64_t> samples_;
};

/**
 * @brief Command line argument parser for performance tests
 */
struct PerfTestArgs {
    std::string topic = "/perf/test";
    double rate_hz = 1000.0;
    size_t msg_size = 64;
    size_t count = 100000;
    size_t warmup = 100;

    static PerfTestArgs parse(int argc, char* argv[]) {
        PerfTestArgs args;
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            if ((arg == "--topic" || arg == "-t") && i + 1 < argc) {
                args.topic = argv[++i];
            } else if ((arg == "--rate" || arg == "-r") && i + 1 < argc) {
                args.rate_hz = std::stod(argv[++i]);
            } else if ((arg == "--size" || arg == "-s") && i + 1 < argc) {
                args.msg_size = std::stoul(argv[++i]);
            } else if ((arg == "--count" || arg == "-c") && i + 1 < argc) {
                args.count = std::stoul(argv[++i]);
            } else if ((arg == "--warmup" || arg == "-w") && i + 1 < argc) {
                args.warmup = std::stoul(argv[++i]);
            } else if (arg == "--help" || arg == "-h") {
                print_usage(argv[0]);
                std::exit(0);
            }
        }
        return args;
    }

    static void print_usage(const char* prog) {
        std::cout << "Usage: " << prog << " [options]\n"
                  << "Options:\n"
                  << "  -t, --topic <name>   Topic name (default: /perf/test)\n"
                  << "  -r, --rate <hz>      Publish rate in Hz (default: 1000)\n"
                  << "  -s, --size <bytes>   Message payload size (default: 64)\n"
                  << "  -c, --count <n>      Number of messages (default: 100000)\n"
                  << "  -w, --warmup <n>     Warmup messages (default: 100)\n"
                  << "  -h, --help           Show this help\n";
    }

    void print() const {
        std::cout << "Performance Test Configuration:\n"
                  << "  Topic:    " << topic << "\n"
                  << "  Rate:     " << rate_hz << " Hz\n"
                  << "  Size:     " << msg_size << " bytes\n"
                  << "  Count:    " << count << "\n"
                  << "  Warmup:   " << warmup << "\n";
    }
};

} // namespace perf
} // namespace leju

#endif // LEJU_PERF_UTILS_HPP_
