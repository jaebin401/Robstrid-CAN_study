/*
개발중인 코드
현재 robstride 액추에이터 can0 채널 하나에서 컨트롤 까지 구현
향후 ros2 연동, ptrhead 구현까지

[변경사항] position_control_single.cpp → CSP 모드 + 런타임 튜닝 기능
- 제어 방식: Operation mode (Type 1) → CSP mode (Type 18, runmode=5)
- 런타임 파라미터 변경: / 커맨드로 재컴파일 없이 즉시 적용
  kp <val>      : loc_kp (위치 루프 P gain)
  spd <val>     : limit_spd (내부 속도 상한, rad/s)
  cur <val>     : limit_cur (전류 상한, A)
  spd_kp <val>  : spd_kp (속도 루프 P gain, 댐핑 역할)
  spd_ki <val>  : spd_ki (속도 루프 I gain)
  <숫자>        : 위치 이동 (deg)
- enable 직전 loc_ref를 현재 위치로 초기화 (튀는 현상 방지)
- 상위 소프트 램프 유지 (limit_spd와 이중 안전)
 */

#include <iostream>
#include <string>
#include <cstring>
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>
#include <csignal>
#include <cmath>
#include <sstream>
#include <iterator>
#include <algorithm>
#include <iomanip>

// Linux Headers
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>

// --- Configuration ---
const char* CAN_INTERFACE = "can0";
const int HOST_ID = 0xFD;
const int MOTOR_ID = 127;

const double MAX_SPEED_DEG_PER_SEC = 500.0;
const double STEP_DEG = 2.0;

// --- CSP 파라미터 초기값 ---
// 진동이 심하면: loc_kp를 낮추거나, spd_kp를 올리거나, limit_spd를 낮춰
// 런타임에 / 커맨드로 재컴파일 없이 변경 가능
const float INIT_LOC_KP    = 15.0f;   // 위치 루프 P gain (기본값 40, 낮게 시작)
const float INIT_LIMIT_SPD =  5.0f;   // 내부 속도 상한 rad/s (낮을수록 부드럽고 느림)
const float INIT_LIMIT_CUR = 10.0f;   // 전류 상한 A (토크 간접 제한)
const float INIT_SPD_KP    = 10.0f;   // 속도 루프 P gain (기본값 6, 댐핑 역할)
const float INIT_SPD_KI    =  0.01f;  // 속도 루프 I gain (기본값 0.02, 진동 시 낮춤)

// --- CAN Protocol Constants ---
const uint32_t COMM_ENABLE          = 3;
const uint32_t COMM_STOP            = 4;
const uint32_t COMM_WRITE_PARAMETER = 18;
const uint32_t COMM_READ_PARAM      = 0x11;

// 파라미터 인덱스 (매뉴얼 4.1.14)
const uint16_t IDX_RUN_MODE  = 0x7005;
const uint16_t IDX_LOC_REF   = 0x7016;  // CSP 위치 지령 (rad, float)
const uint16_t IDX_LIMIT_SPD = 0x7017;  // CSP 속도 상한 (rad/s, float)
const uint16_t IDX_LIMIT_CUR = 0x7018;  // 전류 상한 (A, float)
const uint16_t IDX_MECH_POS  = 0x7019;  // 실제 기계각 (float, read-only)
const uint16_t IDX_LOC_KP    = 0x701E;  // 위치 루프 Kp (float)
const uint16_t IDX_SPD_KP    = 0x701F;  // 속도 루프 Kp (float)
const uint16_t IDX_SPD_KI    = 0x7020;  // 속도 루프 Ki (float)

const uint8_t RUN_MODE_CSP = 5;

// --- Motor State ---
struct MotorState {
    int id;
    std::atomic<double> final_target_pos;
    double current_setpoint;
    double real_pos;
    bool is_enabled;

    // 현재 적용 중인 파라미터 (모니터 출력용)
    std::atomic<float> cur_loc_kp;
    std::atomic<float> cur_limit_spd;
    std::atomic<float> cur_limit_cur;
    std::atomic<float> cur_spd_kp;
    std::atomic<float> cur_spd_ki;

    MotorState(int motor_id)
        : id(motor_id), final_target_pos(0.0), current_setpoint(0.0),
          real_pos(0.0), is_enabled(false),
          cur_loc_kp(INIT_LOC_KP), cur_limit_spd(INIT_LIMIT_SPD),
          cur_limit_cur(INIT_LIMIT_CUR), cur_spd_kp(INIT_SPD_KP),
          cur_spd_ki(INIT_SPD_KI) {}
};

MotorState* motor;
std::atomic<bool> running(true);
std::atomic<bool> monitor_active(true);

// --- Helper Functions ---
void pack_float_le(uint8_t* buf, float val)  { memcpy(buf, &val, sizeof(float)); }
void pack_u16_le(uint8_t* buf, uint16_t val) { memcpy(buf, &val, sizeof(uint16_t)); }

float unpack_float_le(const uint8_t* buf) {
    float val;
    memcpy(&val, buf, sizeof(float));
    return val;
}

struct termios orig_termios;

void reset_terminal_mode() {
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
}

void set_conio_terminal_mode() {
    struct termios new_termios;
    tcgetattr(STDIN_FILENO, &orig_termios);
    memcpy(&new_termios, &orig_termios, sizeof(new_termios));
    atexit(reset_terminal_mode);
    cfmakeraw(&new_termios);
    new_termios.c_lflag |= ISIG;
    new_termios.c_oflag |= OPOST;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
}

int kbhit() {
    struct timeval tv = { 0L, 0L };
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
}

int getch() {
    int r;
    unsigned char c;
    if ((r = read(STDIN_FILENO, &c, sizeof(c))) < 0) return r;
    return c;
}

bool send_frame(int s, uint32_t can_id, const uint8_t* data, uint8_t dlc) {
    struct can_frame frame;
    frame.can_id  = can_id | CAN_EFF_FLAG;
    frame.can_dlc = dlc;
    if (data) memcpy(frame.data, data, dlc);
    else      memset(frame.data, 0, 8);
    return (write(s, &frame, sizeof(struct can_frame)) == sizeof(struct can_frame));
}

bool write_param_float(int s, int motor_id, uint16_t index, float value) {
    uint32_t id = (COMM_WRITE_PARAMETER << 24) | (HOST_ID << 8) | motor_id;
    uint8_t data[8] = {0};
    pack_u16_le(&data[0], index);
    pack_float_le(&data[4], value);
    return send_frame(s, id, data, 8);
}

bool write_param_uint8(int s, int motor_id, uint16_t index, uint8_t value) {
    uint32_t id = (COMM_WRITE_PARAMETER << 24) | (HOST_ID << 8) | motor_id;
    uint8_t data[8] = {0};
    pack_u16_le(&data[0], index);
    data[4] = value;
    return send_frame(s, id, data, 8);
}

void send_read_param(int s, int motor_id, uint16_t index) {
    uint32_t id = (COMM_READ_PARAM << 24) | (HOST_ID << 8) | motor_id;
    uint8_t data[8] = {0};
    pack_u16_le(&data[0], index);
    send_frame(s, id, data, 8);
}

bool enable_motor(int s, int motor_id) {
    uint32_t ext_id = (COMM_ENABLE << 24) | (HOST_ID << 8) | motor_id;
    return send_frame(s, ext_id, nullptr, 0);
}

bool stop_motor(int s, int motor_id) {
    uint32_t ext_id = (COMM_STOP << 24) | (HOST_ID << 8) | motor_id;
    return send_frame(s, ext_id, nullptr, 0);
}

bool write_csp_loc_ref(int s, int motor_id, double pos_rad) {
    return write_param_float(s, motor_id, IDX_LOC_REF, (float)pos_rad);
}

// --- 파라미터 일괄 적용 ---
void apply_csp_params(int s) {
    write_param_float(s, motor->id, IDX_LOC_KP,    motor->cur_loc_kp.load());
    write_param_float(s, motor->id, IDX_LIMIT_SPD,  motor->cur_limit_spd.load());
    write_param_float(s, motor->id, IDX_LIMIT_CUR,  motor->cur_limit_cur.load());
    write_param_float(s, motor->id, IDX_SPD_KP,     motor->cur_spd_kp.load());
    write_param_float(s, motor->id, IDX_SPD_KI,     motor->cur_spd_ki.load());
}

// --- Control Loop ---
void control_loop(int s) {
    const double dt = 0.02;
    const double max_step_rad = (MAX_SPEED_DEG_PER_SEC * M_PI / 180.0) * dt;
    int loop_count = 0;

    while (running) {
        auto start = std::chrono::steady_clock::now();
        loop_count++;

        // 1. Send CSP loc_ref
        MotorState* m = motor;

        if (m->is_enabled) {
            double target  = m->final_target_pos.load();
            double current = m->current_setpoint;
            double diff    = target - current;

            // 상위 소프트 램프: 한 주기에 max_step_rad 이상 이동 금지
            if (std::abs(diff) > max_step_rad) {
                m->current_setpoint += (diff > 0) ? max_step_rad : -max_step_rad;
            } else {
                m->current_setpoint = target;
            }
            write_csp_loc_ref(s, m->id, m->current_setpoint);
        }

        // 5주기마다 실제 위치 read
        if (loop_count % 5 == 0) {
            send_read_param(s, m->id, IDX_MECH_POS);
        }
        std::this_thread::sleep_for(std::chrono::microseconds(50));

        // 2. Read Incoming Frames
        struct can_frame frame;
        struct timeval tv = {0, 0};
        fd_set rdfs;
        FD_ZERO(&rdfs);
        FD_SET(s, &rdfs);

        while (select(s + 1, &rdfs, NULL, NULL, &tv) > 0) {
            if (read(s, &frame, sizeof(struct can_frame)) > 0) {
                uint32_t type   = (frame.can_id >> 24) & 0x1F;
                uint32_t src_id = (frame.can_id >> 8) & 0xFF;
                if (type == COMM_READ_PARAM && (uint32_t)motor->id == src_id) {
                    motor->real_pos = (double)unpack_float_le(&frame.data[4]);
                }
            }
            FD_ZERO(&rdfs);
            FD_SET(s, &rdfs);
            tv.tv_sec = 0; tv.tv_usec = 0;
        }

        // 3. Print Status
        if (monitor_active) {
            double tgt_deg = motor->final_target_pos.load() * 180.0 / M_PI;
            double act_deg = motor->real_pos * 180.0 / M_PI;
            std::string en = motor->is_enabled
                ? "\033[32mON \033[0m" : "\033[31mOFF\033[0m";

            std::cout << "\r\033[K[CSP] "
                      << "ID:" << motor->id << " " << en
                      << " T:" << std::fixed << std::setprecision(1) << std::setw(6) << tgt_deg
                      << " A:" << std::fixed << std::setprecision(1) << std::setw(6) << act_deg
                      << "  kp:" << std::setprecision(1) << motor->cur_loc_kp.load()
                      << " spd:" << motor->cur_limit_spd.load()
                      << " skp:" << motor->cur_spd_kp.load()
                      << " | " << std::flush;
        }

        auto end     = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        if (elapsed.count() < 20000) {
            std::this_thread::sleep_for(std::chrono::microseconds(20000) - elapsed);
        }
    }
}

int init_can(const char* ifname) {
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) return -1;
    strcpy(ifr.ifr_name, ifname);
    ioctl(s, SIOCGIFINDEX, &ifr);
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) return -1;
    return s;
}

void signal_handler(int signum) {
    running = false;
    reset_terminal_mode();
}

int main() {
    motor = new MotorState(MOTOR_ID);

    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    int s = init_can(CAN_INTERFACE);
    if (s < 0) {
        std::cerr << "[ERROR] CAN interface open failed: " << CAN_INTERFACE << std::endl;
        return 1;
    }

    // --- Startup ---
    stop_motor(s, motor->id);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // runmode = 5 (CSP) — enable 전에 반드시 설정
    write_param_uint8(s, motor->id, IDX_RUN_MODE, RUN_MODE_CSP);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    apply_csp_params(s);
    std::cout << "[INIT] CSP mode | "
              << "loc_kp="    << INIT_LOC_KP
              << "  limit_spd=" << INIT_LIMIT_SPD
              << "  limit_cur=" << INIT_LIMIT_CUR
              << "  spd_kp="  << INIT_SPD_KP
              << "  spd_ki="  << INIT_SPD_KI << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    send_read_param(s, motor->id, IDX_MECH_POS);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::thread t(control_loop, s);

    std::cout << "========================================\n"
              << "Space: enable/disable  W/S: +/-2deg  Q: quit\n"
              << "/: 커맨드 모드\n"
              << "  <deg>         위치 이동\n"
              << "  kp <val>      loc_kp 변경        (현재 " << INIT_LOC_KP << ")\n"
              << "  spd <val>     limit_spd 변경     (현재 " << INIT_LIMIT_SPD << " rad/s)\n"
              << "  cur <val>     limit_cur 변경     (현재 " << INIT_LIMIT_CUR << " A)\n"
              << "  spd_kp <val>  spd_kp 변경 댐핑  (현재 " << INIT_SPD_KP << ")\n"
              << "  spd_ki <val>  spd_ki 변경        (현재 " << INIT_SPD_KI << ")\n"
              << "========================================" << std::endl;

    set_conio_terminal_mode();

    double step_rad = STEP_DEG * M_PI / 180.0;

    while (running) {
        if (kbhit()) {
            int key = getch();

            // Space: enable/disable
            if (key == ' ') {
                bool to_enable = !motor->is_enabled;
                if (to_enable) {
                    // enable 직전 loc_ref를 현재 위치로 초기화 → 튀는 현상 방지
                    motor->current_setpoint = motor->real_pos;
                    motor->final_target_pos = motor->real_pos;
                    write_csp_loc_ref(s, motor->id, motor->real_pos);
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    enable_motor(s, motor->id);
                } else {
                    stop_motor(s, motor->id);
                }
                motor->is_enabled = to_enable;
            }
            // Q: quit
            else if (key == 'q' || key == 'Q') {
                running = false;
            }
            // /: command mode
            else if (key == '/') {
                monitor_active = false;
                reset_terminal_mode();

                std::cout << "\nCMD > ";
                std::string line;
                std::getline(std::cin, line);

                if (!line.empty()) {
                    std::stringstream ss(line);
                    std::string cmd;
                    ss >> cmd;

                    // 파라미터 변경 커맨드 처리
                    auto handle_param = [&](const std::string& name,
                                            uint16_t idx,
                                            std::atomic<float>& field,
                                            float lo, float hi) {
                        float val;
                        if (ss >> val) {
                            if (val < lo || val > hi) {
                                std::cout << " -> 범위 초과 (" << lo << " ~ " << hi << ")" << std::endl;
                            } else {
                                field = val;
                                write_param_float(s, motor->id, idx, val);
                                std::cout << " -> " << name << " = " << val << " (즉시 적용)" << std::endl;
                            }
                        } else {
                            std::cout << " -> 값을 입력해주세요." << std::endl;
                        }
                    };

                    if      (cmd == "kp")     handle_param("loc_kp",    IDX_LOC_KP,    motor->cur_loc_kp,    0.0f, 200.0f);
                    else if (cmd == "spd")    handle_param("limit_spd", IDX_LIMIT_SPD, motor->cur_limit_spd, 0.1f,  33.0f);
                    else if (cmd == "cur")    handle_param("limit_cur", IDX_LIMIT_CUR, motor->cur_limit_cur, 0.5f,  16.0f);
                    else if (cmd == "spd_kp") handle_param("spd_kp",    IDX_SPD_KP,    motor->cur_spd_kp,    0.0f,  50.0f);
                    else if (cmd == "spd_ki") handle_param("spd_ki",    IDX_SPD_KI,    motor->cur_spd_ki,    0.0f,   5.0f);
                    else {
                        // 숫자면 위치 이동
                        try {
                            double deg = std::stod(cmd);
                            if (motor->is_enabled) {
                                motor->final_target_pos = deg * M_PI / 180.0;
                                std::cout << " -> " << deg << " deg 이동" << std::endl;
                            } else {
                                std::cout << " -> 모터가 비활성 상태 (Space로 enable)" << std::endl;
                            }
                        } catch (...) {
                            std::cout << " -> 알 수 없는 커맨드 (kp/spd/cur/spd_kp/spd_ki/<deg>)" << std::endl;
                        }
                    }
                } else {
                    std::cout << " -> Cancelled." << std::endl;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                set_conio_terminal_mode();
                monitor_active = true;
            }
            // W/S: direct control
            else if (motor->is_enabled) {
                if (key == 'w' || key == 'W')
                    motor->final_target_pos = motor->final_target_pos.load() + step_rad;
                if (key == 's' || key == 'S')
                    motor->final_target_pos = motor->final_target_pos.load() - step_rad;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // --- Shutdown ---
    reset_terminal_mode();
    if (t.joinable()) t.join();
    std::cout << "\n";
    stop_motor(s, motor->id);
    close(s);
    return 0;
}
