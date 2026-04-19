/*
개발중인 코드
현재 robstride 액추에이터 can0 채널 하나에서 컨트롤 까지 구현
향후 ros2 연동, ptrhead 구현까지

[변경사항] position_control_single.cpp 에서 복제 → CSP 모드로 전환
- 제어 방식: Operation mode (Type 1) → CSP mode (Type 18, runmode=5)
- write_operation_frame() 제거
- write_param_float(), write_param_uint8() 추가 (Type 18 파라미터 write)
- startup 시퀀스에서 runmode=5, limit_spd, limit_cur, loc_kp 설정
- control_loop에서 매 주기 loc_ref만 write (궤적 생성은 상위에서 담당)
- MotorState에서 kp, kd 제거 → loc_kp, limit_spd, limit_cur 추가
- 기존 소프트 램프(current_setpoint 슬라이딩) 유지
  → 모터 내부 위치 루프가 있어도 급격한 목표 변화는 limit_spd로 제한되므로
     상위에서 부드럽게 넘겨주는 것이 더 안전함
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

// --- CSP 파라미터 기본값 (매뉴얼 4.3.4 참고) ---
// loc_kp: 위치 루프 P gain. 기본값 40. 클수록 빠르게 추종하지만 진동 위험
// limit_spd: CSP 내부 속도 상한 (rad/s). 0~33. 안전하게 낮게 시작
// limit_cur: 속도·위치 모드 전류 상한 (A). 0~16. 토크를 간접 제한
const float CSP_LOC_KP    = 40.0f;
const float CSP_LIMIT_SPD = 10.0f;
const float CSP_LIMIT_CUR = 10.0f;

// --- CAN Protocol Constants ---
const uint32_t COMM_ENABLE          = 3;
const uint32_t COMM_STOP            = 4;
const uint32_t COMM_WRITE_PARAMETER = 18;
const uint32_t COMM_READ_PARAM      = 0x11;

// 파라미터 인덱스 (매뉴얼 4.1.14)
const uint16_t IDX_RUN_MODE  = 0x7005;  // 0=operation, 1=PP, 2=velocity, 5=CSP
const uint16_t IDX_LOC_REF   = 0x7016;  // CSP 위치 지령 (rad, float)
const uint16_t IDX_LIMIT_SPD = 0x7017;  // CSP 속도 상한 (rad/s, float)
const uint16_t IDX_LIMIT_CUR = 0x7018;  // 전류 상한 (A, float)
const uint16_t IDX_LOC_KP    = 0x701E;  // 위치 루프 Kp (float)
const uint16_t IDX_MECH_POS  = 0x7019;  // 실제 기계각 읽기 (float, read-only)

const uint8_t RUN_MODE_CSP = 5;

// --- Motor State ---
struct MotorState {
    int id;
    std::atomic<double> final_target_pos;
    double current_setpoint;
    double real_pos;
    bool is_enabled;

    MotorState(int motor_id)
        : id(motor_id), final_target_pos(0.0), current_setpoint(0.0),
          real_pos(0.0), is_enabled(false) {}
};

MotorState* motor;
std::atomic<bool> running(true);
std::atomic<bool> monitor_active(true);

// --- Helper Functions ---
void pack_float_le(uint8_t* buf, float val)   { memcpy(buf, &val, sizeof(float)); }
void pack_u16_le(uint8_t* buf, uint16_t val)  { memcpy(buf, &val, sizeof(uint16_t)); }

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
    frame.can_id = can_id | CAN_EFF_FLAG;
    frame.can_dlc = dlc;
    if (data) memcpy(frame.data, data, dlc);
    else      memset(frame.data, 0, 8);
    return (write(s, &frame, sizeof(struct can_frame)) == sizeof(struct can_frame));
}

// --- Type 18: 단일 파라미터 write (float) ---
// data[0~1]: index LE, data[4~7]: float LE
bool write_param_float(int s, int motor_id, uint16_t index, float value) {
    uint32_t id = (COMM_WRITE_PARAMETER << 24) | (HOST_ID << 8) | motor_id;
    uint8_t data[8] = {0};
    pack_u16_le(&data[0], index);
    pack_float_le(&data[4], value);
    return send_frame(s, id, data, 8);
}

// --- Type 18: 단일 파라미터 write (uint8) ---
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

// --- Motor Enable / Stop ---
bool enable_motor(int s, int motor_id) {
    uint32_t ext_id = (COMM_ENABLE << 24) | (HOST_ID << 8) | motor_id;
    return send_frame(s, ext_id, nullptr, 0);
}

bool stop_motor(int s, int motor_id) {
    uint32_t ext_id = (COMM_STOP << 24) | (HOST_ID << 8) | motor_id;
    return send_frame(s, ext_id, nullptr, 0);
}

// --- CSP 위치 지령 write ---
// 매 제어 주기마다 loc_ref(rad)만 Type 18로 전송
// 모터 내부 위치→속도→전류 3단 루프가 나머지를 처리
bool write_csp_loc_ref(int s, int motor_id, double pos_rad) {
    return write_param_float(s, motor_id, IDX_LOC_REF, (float)pos_rad);
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

            // 상위 소프트 램프: 한 주기에 max_step_rad 이상 이동하지 않도록 제한
            // 모터 내부 limit_spd도 있지만, 상위에서 한 번 더 걸어두면 더 안전
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
                    float val = unpack_float_le(&frame.data[4]);
                    motor->real_pos = (double)val;
                }
            }
            FD_ZERO(&rdfs);
            FD_SET(s, &rdfs);
            tv.tv_sec = 0; tv.tv_usec = 0;
        }

        // 3. Print Status
        if (monitor_active) {
            std::cout << "\r\033[K[CSP] ";

            double tgt_deg = motor->final_target_pos.load() * 180.0 / M_PI;
            double act_deg = motor->real_pos * 180.0 / M_PI;
            std::string status_color = motor->is_enabled
                ? "\033[32mON \033[0m" : "\033[31mOFF\033[0m";

            std::cout << "ID:" << motor->id << " " << status_color
                      << " T:" << std::fixed << std::setprecision(1) << std::setw(6) << tgt_deg
                      << " A:" << std::fixed << std::setprecision(1) << std::setw(6) << act_deg
                      << " | ";
            std::cout << std::flush;
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

    // --- Startup Sequence ---
    stop_motor(s, motor->id);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // 1) runmode = 5 (CSP)
    //    enable 전에 반드시 설정해야 함. enable 후 변경 불가
    write_param_uint8(s, motor->id, IDX_RUN_MODE, RUN_MODE_CSP);
    std::cout << "[INIT] runmode = 5 (CSP)" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // 2) CSP 파라미터 설정
    write_param_float(s, motor->id, IDX_LOC_KP,    CSP_LOC_KP);
    write_param_float(s, motor->id, IDX_LIMIT_SPD,  CSP_LIMIT_SPD);
    write_param_float(s, motor->id, IDX_LIMIT_CUR,  CSP_LIMIT_CUR);
    std::cout << "[INIT] loc_kp=" << CSP_LOC_KP
              << "  limit_spd=" << CSP_LIMIT_SPD << " rad/s"
              << "  limit_cur=" << CSP_LIMIT_CUR << " A" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // 3) 현재 위치 읽기 (enable 전 setpoint 초기화용)
    send_read_param(s, motor->id, IDX_MECH_POS);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::thread t(control_loop, s);

    std::cout << "========================================" << std::endl;
    std::cout << "[MODE] CSP — Space: enable/disable  W/S: +/-  /: deg 입력  Q: 종료" << std::endl;

    set_conio_terminal_mode();

    double step_rad = STEP_DEG * M_PI / 180.0;

    while (running) {
        if (kbhit()) {
            int key = getch();

            // 1. Toggle Enable (Space)
            if (key == ' ') {
                bool to_enable = !motor->is_enabled;

                if (to_enable) {
                    // enable 직전: setpoint을 현재 실제 위치로 초기화
                    // 갑자기 튀는 것 방지
                    motor->current_setpoint  = motor->real_pos;
                    motor->final_target_pos  = motor->real_pos;

                    // CSP에서는 enable 전에 loc_ref를 현재 위치로 먼저 write
                    write_csp_loc_ref(s, motor->id, motor->real_pos);
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));

                    enable_motor(s, motor->id);
                } else {
                    stop_motor(s, motor->id);
                }
                motor->is_enabled = to_enable;
            }
            // 2. Quit (Q)
            else if (key == 'q' || key == 'Q') {
                running = false;
            }
            // 3. Command Mode (/)
            else if (key == '/') {
                monitor_active = false;
                reset_terminal_mode();

                std::cout << "\nCMD (deg) > ";
                std::string line;
                std::getline(std::cin, line);

                if (!line.empty()) {
                    std::stringstream ss(line);
                    std::vector<double> inputs;
                    double temp;
                    while (ss >> temp) inputs.push_back(temp);

                    if (inputs.size() == 1) {
                        if (motor->is_enabled)
                            motor->final_target_pos = inputs[0] * M_PI / 180.0;
                        std::cout << " -> Moved to " << inputs[0] << " deg." << std::endl;
                    } else {
                        std::cout << " -> Invalid input." << std::endl;
                    }
                } else {
                    std::cout << " -> Cancelled." << std::endl;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                set_conio_terminal_mode();
                monitor_active = true;
            }
            // 4. Direct Control (W/S)
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
