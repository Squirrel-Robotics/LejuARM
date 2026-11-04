/**
 * arm_sine_track — 多电机正弦跟踪 demo (SocketCAN 直发)
 *
 * 用途: 排查多电机同步延迟问题, 独立于闭源 hardware_node, 可直接交付给供应商运行.
 *
 * 协议: Motorevo MIT/PTM 单帧 (CAN 2.0, 8 bytes, CAN ID = 电机 ID)
 *   控制帧编码:
 *     Byte0-1:  position  (16-bit)  range [-12.5, 12.5] rad
 *     Byte2:    velocity  [11:4]
 *     Byte3:    velocity  [3:0] | kp [11:8]
 *     Byte4:    kp        [7:0]   (12-bit)  range [0, 250]
 *     Byte5:    kd        [11:4]
 *     Byte6:    kd        [3:0] | torque [11:8]
 *     Byte7:    torque    [7:0]   (12-bit)  range [-50, 50] Nm
 *
 *   反馈帧解码:
 *     Byte0:    motor_id  (8-bit)
 *     Byte1-2:  position  (16-bit)
 *     Byte3+Byte4[7:4]: velocity (12-bit)
 *     Byte4[3:0]+Byte5:  torque   (12-bit)
 *     Byte6:    errcode
 *     Byte7:    temperature
 *
 * 编译: make  (或 g++ -O2 -std=c++17 -pthread -o arm_sine_track arm_sine_track.cpp)
 * 运行: sudo ./arm_sine_track [选项]
 *
 * 选项:
 *   --can-bus NAME      CAN 接口 (默认 bcan2)
 *   --motors ID,...     电机 ID 列表, 十六进制逗号分隔 (默认 01,02,03,04,09,0A)
 *   --amplitude DEG     正弦振幅, 度 (默认 10)
 *   --frequency HZ      正弦频率 (默认 0.5)
 *   --duration SEC      运行时长 (默认 30)
 *   --rate HZ           控制频率 (默认 250)
 *   --kp VALUE          位置刚度 (默认 20)
 *   --kd VALUE          阻尼 (默认 6)
 *   --csv FILE          CSV 输出路径 (默认 sine_track_<timestamp>.csv)
 *   --dry-run           仅打印帧数据, 不实际发送
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>
#include <vector>

#include <errno.h>
#include <getopt.h>
#include <net/if.h>
#include <pthread.h>
#include <sched.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

// ── Linux SocketCAN headers ──
#include <linux/can.h>
#include <linux/can/raw.h>

// ══════════════════════════════════════════════════════════════════════
// MIT/PTM 协议常量
// ══════════════════════════════════════════════════════════════════════
static constexpr float POS_MIN = -12.5f, POS_MAX = 12.5f;       // rad
static constexpr float VEL_MIN = -10.0f, VEL_MAX = 10.0f;       // rad/s
static constexpr float KP_MIN  =   0.0f, KP_MAX  = 250.0f;
static constexpr float KD_MIN  =   0.0f, KD_MAX  =  50.0f;
static constexpr float TOR_MIN = -50.0f, TOR_MAX =  50.0f;      // Nm

static constexpr uint8_t ENABLE_CMD  = 0xFC;
static constexpr uint8_t DISABLE_CMD = 0xFD;

static std::atomic<bool> g_running{true};

static void signal_handler(int) { g_running = false; }

// ══════════════════════════════════════════════════════════════════════
// 工具函数
// ══════════════════════════════════════════════════════════════════════

/// 浮点数线性量化为 N-bit 无符号整数
template <int Bits>
static uint32_t float_to_uint(float val, float min_val, float max_val) {
    float clamped = std::clamp(val, min_val, max_val);
    uint32_t max_int = (1u << Bits) - 1;
    return static_cast<uint32_t>((clamped - min_val) / (max_val - min_val) * max_int);
}

/// N-bit 无符号整数反量化为浮点数
template <int Bits>
static float uint_to_float(uint32_t raw, float min_val, float max_val) {
    uint32_t max_int = (1u << Bits) - 1;
    return static_cast<float>(raw) / max_int * (max_val - min_val) + min_val;
}

/// 当前时间戳 (monotonic, 秒)
static double now_sec() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

/// 生成默认 CSV 文件名
static std::string default_csv_name() {
    time_t t = time(nullptr);
    struct tm tm_buf;
    localtime_r(&t, &tm_buf);
    char buf[64];
    strftime(buf, sizeof(buf), "sine_track_%Y%m%d_%H%M%S.csv", &tm_buf);
    return buf;
}

// ══════════════════════════════════════════════════════════════════════
// MIT 帧编解码
// ══════════════════════════════════════════════════════════════════════

static void encode_mit_frame(uint8_t buf[8],
                             float pos, float vel, float torque,
                             float kp, float kd) {
    uint16_t p = float_to_uint<16>(pos, POS_MIN, POS_MAX);
    uint16_t v = float_to_uint<12>(vel, VEL_MIN, VEL_MAX);
    uint16_t k = float_to_uint<12>(kp,  KP_MIN,  KP_MAX);
    uint16_t d = float_to_uint<12>(kd,  KD_MIN,  KD_MAX);
    uint16_t t = float_to_uint<12>(torque, TOR_MIN, TOR_MAX);

    buf[0] = (p >> 8) & 0xFF;
    buf[1] =  p       & 0xFF;
    buf[2] = (v >> 4) & 0xFF;
    buf[3] = ((v & 0x0F) << 4) | ((k >> 8) & 0x0F);
    buf[4] =  k       & 0xFF;
    buf[5] = (d >> 4) & 0xFF;
    buf[6] = ((d & 0x0F) << 4) | ((t >> 8) & 0x0F);
    buf[7] =  t       & 0xFF;
}

static void encode_control_frame(uint8_t buf[8], uint8_t cmd) {
    memset(buf, 0xFF, 7);
    buf[7] = cmd;
}

/// 解码反馈帧 (8 bytes) → motor_id, pos, vel, torque, errcode, temperature
struct FeedbackData {
    uint8_t motor_id;
    float   pos;          // rad
    float   vel;          // rad/s
    float   torque;       // Nm
    uint8_t errcode;
    uint8_t temperature;
};

static bool decode_feedback(const uint8_t data[8], FeedbackData& fb) {
    fb.motor_id    = data[0];
    uint32_t p_raw = (data[1] << 8) | data[2];
    uint32_t v_raw = (data[3] << 4) | (data[4] >> 4);
    uint32_t t_raw = ((data[4] & 0x0F) << 8) | data[5];
    fb.pos         = uint_to_float<16>(p_raw, POS_MIN, POS_MAX);
    fb.vel         = uint_to_float<12>(v_raw, VEL_MIN, VEL_MAX);
    fb.torque      = uint_to_float<12>(t_raw, TOR_MIN, TOR_MAX);
    fb.errcode     = data[6];
    fb.temperature = data[7];
    return true;
}

// ══════════════════════════════════════════════════════════════════════
// SocketCAN
// ══════════════════════════════════════════════════════════════════════

static int open_can_socket(const char* ifname) {
    int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
        perror("socket(PF_CAN)");
        return -1;
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl(SIOCGIFINDEX)");
        close(fd);
        return -1;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(fd);
        return -1;
    }

    // 设置非阻塞 (用于接收反馈)
    struct timeval tv = {0, 0};  // 非阻塞
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    return fd;
}

static bool send_can_frame(int fd, uint32_t can_id, const uint8_t data[8]) {
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id  = can_id;
    frame.can_dlc = 8;
    memcpy(frame.data, data, 8);
    ssize_t n = write(fd, &frame, sizeof(frame));
    return n == sizeof(frame);
}

static bool recv_can_frame(int fd, uint32_t& can_id, uint8_t data[8]) {
    struct can_frame frame;
    ssize_t n = read(fd, &frame, sizeof(frame));
    if (n == sizeof(frame)) {
        can_id = frame.can_id & CAN_SFF_MASK;
        memcpy(data, frame.data, 8);
        return true;
    }
    return false;
}

/// 读空 socket 缓冲区中的所有待接收帧
static void drain_rx(int fd) {
    uint32_t id;
    uint8_t buf[8];
    while (recv_can_frame(fd, id, buf)) {}
}

// ══════════════════════════════════════════════════════════════════════
// 电机描述
// ══════════════════════════════════════════════════════════════════════

struct MotorConfig {
    uint32_t    can_id;
    std::string name;
    float       kp;
    float       kd;

    // 运行时状态
    double      zero_pos;       // 使能后读取的零点 (raw)
    double      last_cmd_pos;   // 上次发送的 raw 目标位置 (用于回零)
    double      fb_pos;         // 最新反馈位置
    double      fb_vel;
    double      fb_torque;
    uint8_t     fb_errcode;
    uint8_t     fb_temperature;
    double      fb_timestamp;   // 收到反馈的时间戳
    bool        fb_valid;       // 本周期是否有反馈
};

// ══════════════════════════════════════════════════════════════════════
// 实时调度
// ══════════════════════════════════════════════════════════════════════

static bool set_realtime_priority(int priority) {
    struct sched_param param;
    param.sched_priority = priority;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
        fprintf(stderr, "警告: 无法设置 SCHED_FIFO (priority=%d): %s\n"
                        "  提示: 请用 sudo 运行\n", priority, strerror(errno));
        return false;
    }
    return true;
}

// ══════════════════════════════════════════════════════════════════════
// 参数解析
// ══════════════════════════════════════════════════════════════════════

struct Args {
    std::string can_bus    = "bcan2";
    std::vector<uint32_t> motor_ids = {0x01, 0x02, 0x03, 0x04, 0x09, 0x0A};
    float       amplitude  = 10.0f;   // 度
    float       frequency  = 0.5f;    // Hz
    float       duration   = 30.0f;   // 秒
    int         rate       = 250;     // Hz
    float       kp         = 20.0f;
    float       kd         = 6.0f;
    std::string csv_path;
    bool        dry_run    = false;
};

static std::vector<uint32_t> parse_motor_ids(const char* s) {
    std::vector<uint32_t> ids;
    char buf[256];
    strncpy(buf, s, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    char* tok = strtok(buf, ",");
    while (tok) {
        ids.push_back(strtoul(tok, nullptr, 16));
        tok = strtok(nullptr, ",");
    }
    return ids;
}

static Args parse_args(int argc, char* argv[]) {
    Args args;
    static struct option long_opts[] = {
        {"can-bus",    required_argument, nullptr, 'b'},
        {"motors",     required_argument, nullptr, 'm'},
        {"amplitude",  required_argument, nullptr, 'a'},
        {"frequency",  required_argument, nullptr, 'f'},
        {"duration",   required_argument, nullptr, 'd'},
        {"rate",       required_argument, nullptr, 'r'},
        {"kp",         required_argument, nullptr, 'p'},
        {"kd",         required_argument, nullptr, 'k'},
        {"csv",        required_argument, nullptr, 'c'},
        {"dry-run",    no_argument,       nullptr, 'n'},
        {"help",       no_argument,       nullptr, 'h'},
        {nullptr, 0, nullptr, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "b:m:a:f:d:r:p:k:c:nh", long_opts, nullptr)) != -1) {
        switch (opt) {
        case 'b': args.can_bus   = optarg; break;
        case 'm': args.motor_ids = parse_motor_ids(optarg); break;
        case 'a': args.amplitude = atof(optarg); break;
        case 'f': args.frequency = atof(optarg); break;
        case 'd': args.duration  = atof(optarg); break;
        case 'r': args.rate      = atoi(optarg); break;
        case 'p': args.kp        = atof(optarg); break;
        case 'k': args.kd        = atof(optarg); break;
        case 'c': args.csv_path  = optarg; break;
        case 'n': args.dry_run   = true; break;
        case 'h':
        default:
            printf("用法: sudo %s [选项]\n"
                   "  --can-bus NAME      CAN 接口 (默认 bcan2)\n"
                   "  --motors ID,...     电机 ID, 十六进制逗号分隔 (默认 01,02,03,04,09,0A)\n"
                   "  --amplitude DEG     正弦振幅, 度 (默认 10)\n"
                   "  --frequency HZ      频率 (默认 0.5)\n"
                   "  --duration SEC      时长 (默认 30)\n"
                   "  --rate HZ           控制频率 (默认 250)\n"
                   "  --kp VALUE          位置 Kp (默认 20)\n"
                   "  --kd VALUE          位置 Kd (默认 6)\n"
                   "  --csv FILE          CSV 输出路径\n"
                   "  --dry-run           仅打印不发送\n",
                   argv[0]);
            exit(opt == 'h' ? 0 : 1);
        }
    }
    if (args.csv_path.empty()) args.csv_path = default_csv_name();
    return args;
}

// ══════════════════════════════════════════════════════════════════════
// CSV 记录器
// ══════════════════════════════════════════════════════════════════════

class CsvLogger {
public:
    bool open(const std::string& path, const std::vector<MotorConfig>& motors) {
        fp_ = fopen(path.c_str(), "w");
        if (!fp_) return false;
        n_motors_ = motors.size();

        // 表头
        fprintf(fp_, "cycle,elapsed_s,send_ts_us");
        for (size_t i = 0; i < n_motors_; ++i) {
            const char* n = motors[i].name.c_str();
            fprintf(fp_, ",cmd_pos/%s,cmd_vel/%s,fb_pos/%s,fb_vel/%s,fb_torque/%s,"
                         "fb_errcode/%s,fb_temp/%s,fb_ts_us/%s,rtt_us/%s",
                    n, n, n, n, n, n, n, n, n);
        }
        fprintf(fp_, "\n");
        return true;
    }

    void log(uint64_t cycle, double elapsed, double send_ts,
             const std::vector<MotorConfig>& motors,
             const std::vector<double>& cmd_pos,
             const std::vector<double>& cmd_vel) {
        if (!fp_) return;
        fprintf(fp_, "%lu,%.6f,%.0f", (unsigned long)cycle, elapsed, send_ts * 1e6);
        for (size_t i = 0; i < n_motors_; ++i) {
            const auto& m = motors[i];
            double rtt = (m.fb_valid && m.fb_timestamp > 0) ? (m.fb_timestamp - send_ts) * 1e6 : -1.0;
            fprintf(fp_, ",%.6f,%.6f,%.6f,%.6f,%.6f,%u,%u,%.0f,%.1f",
                    cmd_pos[i], cmd_vel[i],
                    m.fb_pos, m.fb_vel, m.fb_torque,
                    m.fb_errcode, m.fb_temperature,
                    m.fb_valid ? m.fb_timestamp * 1e6 : 0.0,
                    rtt);
        }
        fprintf(fp_, "\n");
    }

    void close() {
        if (fp_) { fclose(fp_); fp_ = nullptr; }
    }

    ~CsvLogger() { close(); }

private:
    FILE* fp_ = nullptr;
    size_t n_motors_ = 0;
};

// ══════════════════════════════════════════════════════════════════════
// main
// ══════════════════════════════════════════════════════════════════════

int main(int argc, char* argv[]) {
    Args args = parse_args(argc, argv);

    const float amplitude_rad = args.amplitude * M_PI / 180.0f;
    const double interval_ns  = 1e9 / args.rate;

    // 初始化电机配置
    // bcan2 默认命名 (供应商可按需修改)
    auto name_for_id = [](uint32_t id) -> std::string {
        switch (id) {
        case 0x01: return "Larm_J1";
        case 0x02: return "Larm_J2";
        case 0x03: return "Larm_J3";
        case 0x04: return "Larm_J4";
        case 0x09: return "Head_yaw";
        case 0x0A: return "Head_pitch";
        default: {
            char buf[16];
            snprintf(buf, sizeof(buf), "Motor_0x%02X", id);
            return buf;
        }
        }
    };

    std::vector<MotorConfig> motors;
    for (uint32_t id : args.motor_ids) {
        MotorConfig m{};
        m.can_id   = id;
        m.name     = name_for_id(id);
        m.kp       = args.kp;
        m.kd       = args.kd;
        m.zero_pos = 0.0;
        m.last_cmd_pos = 0.0;
        m.fb_pos   = 0.0;
        m.fb_vel   = 0.0;
        m.fb_torque = 0.0;
        m.fb_errcode = 0;
        m.fb_temperature = 0;
        m.fb_timestamp = 0.0;
        m.fb_valid = false;
        motors.push_back(m);
    }

    const size_t n = motors.size();

    // ── 打印参数 ──
    printf("=== 多电机正弦跟踪 demo ===\n");
    printf("CAN 接口:   %s\n", args.can_bus.c_str());
    printf("电机数量:   %zu\n", n);
    printf("电机列表:  ");
    for (auto& m : motors) printf(" %s(0x%02X)", m.name.c_str(), m.can_id);
    printf("\n");
    printf("振幅:       %.1f° (%.4f rad)\n", args.amplitude, amplitude_rad);
    printf("频率:       %.2f Hz\n", args.frequency);
    printf("时长:       %.1f s\n", args.duration);
    printf("控制频率:   %d Hz\n", args.rate);
    printf("Kp=%.2f  Kd=%.2f\n", args.kp, args.kd);
    printf("CSV:        %s\n", args.csv_path.c_str());
    printf("\n");

    // ── dry-run 模式 ──
    if (args.dry_run) {
        printf("[dry-run] 模拟 3 个周期:\n");
        for (int i = 0; i < 3; ++i) {
            double t = i / (double)args.rate;
            float sine_pos = amplitude_rad * sinf(2 * M_PI * args.frequency * t);
            float sine_vel = amplitude_rad * 2 * M_PI * args.frequency * cosf(2 * M_PI * args.frequency * t);
            uint8_t buf[8];
            encode_mit_frame(buf, sine_pos, sine_vel, 0.0f, args.kp, args.kd);
            printf("  t=%.4fs pos=%+.4f rad vel=%+.4f frame=", t, sine_pos, sine_vel);
            for (int j = 0; j < 8; ++j) printf("%02X ", buf[j]);
            printf("\n");
        }
        return 0;
    }

    // ── 打开 CAN 接口 ──
    int can_fd = open_can_socket(args.can_bus.c_str());
    if (can_fd < 0) {
        fprintf(stderr, "打开 %s 失败. 请确认接口已启动:\n"
                        "  sudo ip link set %s up type can bitrate 1000000\n",
                args.can_bus.c_str(), args.can_bus.c_str());
        return 1;
    }
    printf("SocketCAN %s 已打开 (fd=%d)\n", args.can_bus.c_str(), can_fd);

    // ── 设置实时调度 ──
    set_realtime_priority(20);

    // ── 信号处理 ──
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // ── CSV 记录器 ──
    CsvLogger csv;
    if (!csv.open(args.csv_path, motors)) {
        fprintf(stderr, "无法创建 CSV: %s\n", args.csv_path.c_str());
        close(can_fd);
        return 1;
    }

    // ══════════════════════════════════════════════════════════════════
    // Step 1: 使能所有电机
    // ══════════════════════════════════════════════════════════════════
    printf("使能电机...\n");
    drain_rx(can_fd);

    uint8_t enable_buf[8];
    encode_control_frame(enable_buf, ENABLE_CMD);
    for (auto& m : motors) {
        send_can_frame(can_fd, m.can_id, enable_buf);
        usleep(2000);  // 2ms 间隔, 避免总线拥塞
    }
    usleep(50000);  // 等 50ms 让电机响应

    // 读取使能反馈
    for (int attempt = 0; attempt < 20; ++attempt) {
        uint32_t rx_id;
        uint8_t rx_data[8];
        if (recv_can_frame(can_fd, rx_id, rx_data)) {
            for (auto& m : motors) {
                if (rx_id == m.can_id) {
                    FeedbackData fb;
                    decode_feedback(rx_data, fb);
                    printf("  %s(0x%02X): pos=%.4f rad, err=0x%02X, temp=%u°C\n",
                           m.name.c_str(), m.can_id, fb.pos, fb.errcode, fb.temperature);
                }
            }
        }
        usleep(2000);
    }

    // ══════════════════════════════════════════════════════════════════
    // Step 2: 读取当前位置作为零点
    // ══════════════════════════════════════════════════════════════════
    printf("读取零点位置...\n");
    drain_rx(can_fd);

    // 发送零力矩帧触发反馈
    uint8_t zero_buf[8];
    encode_mit_frame(zero_buf, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    for (auto& m : motors) {
        send_can_frame(can_fd, m.can_id, zero_buf);
        usleep(2000);
    }
    usleep(50000);

    // 收集反馈
    for (int attempt = 0; attempt < 30; ++attempt) {
        uint32_t rx_id;
        uint8_t rx_data[8];
        if (recv_can_frame(can_fd, rx_id, rx_data)) {
            for (auto& m : motors) {
                if (rx_id == m.can_id) {
                    FeedbackData fb;
                    decode_feedback(rx_data, fb);
                    m.zero_pos = fb.pos;
                    printf("  %s 零点: %.4f rad\n", m.name.c_str(), m.zero_pos);
                }
            }
        }
        usleep(2000);
    }

    // ══════════════════════════════════════════════════════════════════
    // Step 3: 正弦跟踪控制循环
    // ══════════════════════════════════════════════════════════════════
    printf("\n开始正弦跟踪 (Ctrl+C 停止)...\n\n");

    struct timespec next_wakeup;
    clock_gettime(CLOCK_MONOTONIC, &next_wakeup);
    const double start_time = now_sec();

    uint64_t cycle = 0;
    double last_print = 0.0;

    std::vector<double> cmd_pos(n), cmd_vel(n);

    while (g_running) {
        double elapsed = now_sec() - start_time;
        if (elapsed >= args.duration) {
            printf("\n已达到 %.1fs, 停止.\n", args.duration);
            break;
        }

        // 计算正弦目标 (所有电机同相, 用于观测同步性)
        float sine_pos = amplitude_rad * sinf(2 * M_PI * args.frequency * elapsed);
        float sine_vel = amplitude_rad * 2 * M_PI * args.frequency * cosf(2 * M_PI * args.frequency * elapsed);

        // 清除本周期反馈状态
        for (auto& m : motors) m.fb_valid = false;

        // 发送控制帧给每个电机
        double send_ts = now_sec();
        for (size_t i = 0; i < n; ++i) {
            auto& m = motors[i];
            float target_pos = m.zero_pos + sine_pos;
            float target_vel = sine_vel;

            cmd_pos[i] = target_pos;
            cmd_vel[i] = target_vel;
            m.last_cmd_pos = target_pos;

            uint8_t frame[8];
            encode_mit_frame(frame, target_pos, target_vel, 0.0f, m.kp, m.kd);
            send_can_frame(can_fd, m.can_id, frame);
        }

        // 接收反馈 (非阻塞, 多次尝试)
        for (int rx_attempt = 0; rx_attempt < (int)n * 2; ++rx_attempt) {
            uint32_t rx_id;
            uint8_t rx_data[8];
            if (!recv_can_frame(can_fd, rx_id, rx_data)) break;
            double rx_ts = now_sec();
            for (auto& m : motors) {
                if (rx_id == m.can_id) {
                    FeedbackData fb;
                    decode_feedback(rx_data, fb);
                    m.fb_pos       = fb.pos;
                    m.fb_vel       = fb.vel;
                    m.fb_torque    = fb.torque;
                    m.fb_errcode   = fb.errcode;
                    m.fb_temperature = fb.temperature;
                    m.fb_timestamp = rx_ts;
                    m.fb_valid     = true;
                }
            }
        }

        // CSV 记录
        csv.log(cycle, elapsed, send_ts, motors, cmd_pos, cmd_vel);

        // 终端输出 (每 200ms)
        if (elapsed - last_print >= 0.2) {
            printf("  t=%6.2fs  sine=%+.4f rad |", elapsed, sine_pos);
            for (auto& m : motors) {
                if (m.fb_valid) {
                    double err = m.fb_pos - m.last_cmd_pos;
                    printf(" %s:%+.3f(e=%+.4f)", m.name.c_str(), m.fb_pos, err);
                } else {
                    printf(" %s:N/A", m.name.c_str());
                }
            }
            printf("\n");
            last_print = elapsed;
        }

        ++cycle;

        // clock_nanosleep 精确控制周期
        next_wakeup.tv_nsec += (long)interval_ns;
        while (next_wakeup.tv_nsec >= 1000000000L) {
            next_wakeup.tv_nsec -= 1000000000L;
            next_wakeup.tv_sec++;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wakeup, nullptr);
    }

    // ══════════════════════════════════════════════════════════════════
    // Step 4: 回零 (1 秒线性插值)
    // ══════════════════════════════════════════════════════════════════
    printf("\n回零中...\n");
    const double ramp_duration = 1.0;
    struct timespec ramp_wakeup;
    clock_gettime(CLOCK_MONOTONIC, &ramp_wakeup);
    double ramp_start = now_sec();

    std::vector<double> last_pos(n);
    for (size_t i = 0; i < n; ++i) last_pos[i] = motors[i].last_cmd_pos;

    while (now_sec() - ramp_start < ramp_duration && g_running) {
        double t = (now_sec() - ramp_start) / ramp_duration;
        t = std::clamp(t, 0.0, 1.0);

        for (size_t i = 0; i < n; ++i) {
            float pos = last_pos[i] * (1.0 - t) + motors[i].zero_pos * t;
            uint8_t frame[8];
            encode_mit_frame(frame, pos, 0.0f, 0.0f, motors[i].kp, motors[i].kd);
            send_can_frame(can_fd, motors[i].can_id, frame);
        }

        ramp_wakeup.tv_nsec += (long)interval_ns;
        while (ramp_wakeup.tv_nsec >= 1000000000L) {
            ramp_wakeup.tv_nsec -= 1000000000L;
            ramp_wakeup.tv_sec++;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ramp_wakeup, nullptr);
    }

    // ══════════════════════════════════════════════════════════════════
    // Step 5: 失能所有电机
    // ══════════════════════════════════════════════════════════════════
    printf("失能电机...\n");
    uint8_t disable_buf[8];
    encode_control_frame(disable_buf, DISABLE_CMD);
    for (auto& m : motors) {
        send_can_frame(can_fd, m.can_id, disable_buf);
        usleep(2000);
    }

    csv.close();
    close(can_fd);

    printf("\n完成. 共 %lu 个控制周期, CSV 已保存: %s\n", (unsigned long)cycle, args.csv_path.c_str());
    return 0;
}
