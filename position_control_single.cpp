/*
개발중인 코드
현재 robstride 액추에이터 can0 채널 하나에서 컨트롤 까지 구현
향후 ros2 연동, ptrhead 구현까지
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

const int NUM_MOTORS = 1;
const int MOTOR_IDS[NUM_MOTORS] = {127}; 
// const int MOTOR_ID = 127;

const double MAX_SPEED_DEG_PER_SEC = 500.0; 
const double STEP_DEG = 2.0; 

const double DEFAULT_MIN_LIMIT = -900.0;
const double DEFAULT_MAX_LIMIT = 900.0;

const double P_MIN = -12.57;
const double P_MAX = 12.57;

// --- Motor State ---
struct MotorState {
    int id;
    std::atomic<double> final_target_pos; 
    double current_setpoint;              
    double real_pos; 
    bool is_enabled; 
    
    std::atomic<double> kp;
    std::atomic<double> kd;
    
    MotorState(int motor_id) 
        : id(motor_id), final_target_pos(0.0), current_setpoint(0.0), 
          real_pos(0.0), is_enabled(false),
          kp(40.0), kd(5) {}
};

MotorState* motors[NUM_MOTORS]; 
std::atomic<bool> running(true);
// 화면 출력 제어용 (명령어 입력 중에는 모니터링 출력 일시 정지)
std::atomic<bool> monitor_active(true); 

const uint32_t COMM_OPERATION_CONTROL = 1;
const uint32_t COMM_ENABLE = 3;
const uint32_t COMM_STOP = 4;
const uint32_t COMM_WRITE_PARAMETER = 18;
const uint32_t COMM_READ_PARAM = 0x11; 
const uint16_t PARAM_MODE = 0x7005;
const uint16_t IDX_MECH_POS = 0x7019; 

// --- Helper Functions ---
void pack_float_le(uint8_t* buf, float val) { memcpy(buf, &val, sizeof(float)); }
void pack_u16_le(uint8_t* buf, uint16_t val) { memcpy(buf, &val, sizeof(uint16_t)); }
void pack_u16_be(uint8_t* buf, uint16_t val) {
    buf[0] = (val >> 8) & 0xFF;
    buf[1] = val & 0xFF;
}

float unpack_float_le(const uint8_t* buf) {
    float val;
    memcpy(&val, buf, sizeof(float));
    return val;
}

// 터미널 설정 원복용
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
    // Ctrl+C (SIGINT) 처리를 위해 ISIG 플래그는 살려둠
    new_termios.c_lflag |= ISIG; 
    new_termios.c_oflag |= OPOST; // 줄바꿈 처리를 위해 출력 후처리 켜기
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
    else memset(frame.data, 0, 8);
    int ret = write(s, &frame, sizeof(struct can_frame));
    return (ret == sizeof(struct can_frame));
}

bool read_frame(int s, struct can_frame* frame) {
    struct timeval tv = {0, 5000}; 
    fd_set rdfs;
    FD_ZERO(&rdfs);
    FD_SET(s, &rdfs);
    int ret = select(s + 1, &rdfs, NULL, NULL, &tv);
    if (ret <= 0) return false;
    if (read(s, frame, sizeof(struct can_frame)) < 0) return false;
    return true;
}

void send_read_param(int s, int motor_id, uint16_t index) {
    uint32_t id = (COMM_READ_PARAM << 24) | (HOST_ID << 8) | motor_id;
    uint8_t data[8] = {0};
    pack_u16_le(&data[0], index);
    send_frame(s, id, data, 8);
}

// --- Control Logic ---
bool enable_motor(int s, int motor_id) {
    uint32_t ext_id = (COMM_ENABLE << 24) | (HOST_ID << 8) | motor_id;
    return send_frame(s, ext_id, nullptr, 0);
}

bool stop_motor(int s, int motor_id) {
    uint32_t ext_id = (COMM_STOP << 24) | (HOST_ID << 8) | motor_id;
    return send_frame(s, ext_id, nullptr, 0);
}

bool write_operation_frame(int s, int motor_id, double pos, double kp_val, double kd_val) {
    const double POS_SCALE = 4 * M_PI;
    const double KP_SCALE = 5000.0;
    const double KD_SCALE = 100.0;

    double pos_clamped = std::max(-POS_SCALE, std::min(POS_SCALE, pos));
    double kp_clamped = std::max(0.0, std::min(KP_SCALE, kp_val));
    double kd_clamped = std::max(0.0, std::min(KD_SCALE, kd_val));
    
    uint16_t pos_u16 = (uint16_t)(((pos_clamped / POS_SCALE) + 1.0) * 0x7FFF);
    uint16_t vel_u16 = 0x7FFF;
    uint16_t kp_u16 = (uint16_t)((kp_clamped / KP_SCALE) * 0xFFFF);
    uint16_t kd_u16 = (uint16_t)((kd_clamped / KD_SCALE) * 0xFFFF);
    uint16_t torque_u16 = 0x7FFF;

    uint8_t data[8];
    pack_u16_be(&data[0], pos_u16);
    pack_u16_be(&data[2], vel_u16);
    pack_u16_be(&data[4], kp_u16);
    pack_u16_be(&data[6], kd_u16);
    
    uint32_t ext_id = (COMM_OPERATION_CONTROL << 24) | (torque_u16 << 8) | motor_id;
    return send_frame(s, ext_id, data, 8);
}

void control_loop(int s) {
    const double dt = 0.02; 
    const double max_step_rad = (MAX_SPEED_DEG_PER_SEC * M_PI / 180.0) * dt;
    int loop_count = 0;

    while (running) {
        auto start = std::chrono::steady_clock::now();
        loop_count++;
        
        // 1. Send Commands
        for (int i = 0; i < NUM_MOTORS; i++) {
            MotorState* m = motors[i];
            
            if (m->is_enabled) {
                double target = m->final_target_pos.load();
                double current = m->current_setpoint;
                double diff = target - current;

                if (std::abs(diff) > max_step_rad) {
                    if (diff > 0) m->current_setpoint += max_step_rad;
                    else          m->current_setpoint -= max_step_rad;
                } else {
                    m->current_setpoint = target;
                }
                write_operation_frame(s, m->id, m->current_setpoint, m->kp.load(), m->kd.load());
            } 
            
            if (loop_count % 5 == 0) {
                send_read_param(s, m->id, IDX_MECH_POS);
            }
            std::this_thread::sleep_for(std::chrono::microseconds(50)); 
        }
        
        // 2. Read Incoming Frames
        struct can_frame frame;
        struct timeval tv = {0, 0}; 
        fd_set rdfs;
        FD_ZERO(&rdfs);
        FD_SET(s, &rdfs);
        
        while(select(s + 1, &rdfs, NULL, NULL, &tv) > 0) {
            if (read(s, &frame, sizeof(struct can_frame)) > 0) {
                uint32_t type = (frame.can_id >> 24) & 0x1F;
                
                if (type == COMM_READ_PARAM) {
                    uint32_t src_id = (frame.can_id >> 8) & 0xFF; 
                    for(int i=0; i<NUM_MOTORS; i++) {
                        if ((uint32_t)motors[i]->id == src_id) {
                            float val = unpack_float_le(&frame.data[4]);
                            motors[i]->real_pos = (double)val;
                        }
                    }
                }
            }
            FD_ZERO(&rdfs);
            FD_SET(s, &rdfs);
            tv.tv_sec = 0; tv.tv_usec = 0;
        }

        // 3. Print Status (Only when monitor is active)
        if (monitor_active) {
            std::cout << "\r\033[K[KEY] "; // \033[K : Clear line from cursor
            for(int i=0; i<NUM_MOTORS; i++) {
                double tgt_deg = motors[i]->final_target_pos.load() * 180.0 / M_PI;
                double act_deg = motors[i]->real_pos * 180.0 / M_PI;
                std::string status_color = motors[i]->is_enabled ? "\033[32mON \033[0m" : "\033[31mOFF\033[0m"; 
                std::cout << "ID:" << motors[i]->id << " " << status_color
                          << " T:" << std::fixed << std::setprecision(1) << std::setw(6) << tgt_deg
                          << " A:" << std::fixed << std::setprecision(1) << std::setw(6) << act_deg << " | ";
            }
            std::cout << std::flush;
        }
        
        auto end = std::chrono::steady_clock::now();
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
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) return -1;
    return s;
}

void signal_handler(int signum) { 
    running = false; 
    reset_terminal_mode(); // 종료 시 터미널 복구
}

int main() {
    for(int i=0; i<NUM_MOTORS; i++) motors[i] = new MotorState(MOTOR_IDS[i]);
    //motor = new MotorState(MOTOR_ID);

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    int s = init_can(CAN_INTERFACE);
    if (s < 0) return 1;

    // Startup
    for (int i = 0; i < NUM_MOTORS; i++) {
        stop_motor(s, motors[i]->id);
        send_read_param(s, motors[i]->id, IDX_MECH_POS);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    /*
    
    stop_motor(s, motor->id);
    send_read_param(s, motor->id, IDX_MECH_POS);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    */
    std::thread t(control_loop, s);

    std::cout << "========================================" << std::endl;

    set_conio_terminal_mode(); // Raw Mode Enable

    double step_rad = STEP_DEG * M_PI / 180.0;

    while (running) {
        if (kbhit()) {
            int key = getch();
            
            // 1. Toggle Enable (Space)
            if (key == ' ') { 
                // Toggle Logic based on first motor
                bool to_enable = !motors[0]->is_enabled;
                /*
                if(to_enable) {
                    motor->current_setpoint = motor->real_pos;
                    motor->final_target_pos = motor->real_pos;
                    enable_motor(s, motor->id);
                } 
                else {
                    stop_motor(s, motor->id);
                }
                motor->is_enabled = to_enable;
            */
                for(int i=0; i<NUM_MOTORS; i++) {
                    if(to_enable) {
                        motors[i]->current_setpoint = motors[i]->real_pos;
                        motors[i]->final_target_pos = motors[i]->real_pos;
                        enable_motor(s, motors[i]->id);
                    } else {
                        stop_motor(s, motors[i]->id);
                    }
                    motors[i]->is_enabled = to_enable;
                }
            }
            // 2. Quit (Q)
            else if (key == 'q' || key == 'Q') {
                running = false;
            }
            // 3. Command Mode (/)
            else if (key == '/') {
                monitor_active = false; // Stop updating monitor line
                reset_terminal_mode();  // Restore canonical mode for cin
                
                std::cout << "\nCMD > ";
                std::string line;
                std::getline(std::cin, line); // Wait for Enter
                
                // Parse Command
                if (!line.empty()) {
                    std::stringstream ss(line);
                    std::vector<double> inputs;
                    double temp;
                    while (ss >> temp) inputs.push_back(temp);
                    
                    if (inputs.size() == NUM_MOTORS) {
                        for(int i=0; i<NUM_MOTORS; i++) {
                            if(motors[i]->is_enabled) motors[i]->final_target_pos = inputs[i] * M_PI / 180.0;
                        }
                        std::cout << " -> Moved." << std::endl;
                    } else {
                        std::cout << " -> Invalid Input." << std::endl;
                    }
                } else {
                    std::cout << " -> Cancelled." << std::endl;
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                set_conio_terminal_mode(); // Restore Raw Mode
                monitor_active = true;
            }
            // 4. Direct Control
            else if (motors[0]->is_enabled || motors[1]->is_enabled) {
                if (key == 'w' || key == 'W') motors[0]->final_target_pos = motors[0]->final_target_pos.load() + step_rad;
                if (key == 's' || key == 'S') motors[0]->final_target_pos = motors[0]->final_target_pos.load() - step_rad;
                
                if (key == 'e' || key == 'E') motors[1]->final_target_pos = motors[1]->final_target_pos.load() + step_rad;
                if (key == 'd' || key == 'D') motors[1]->final_target_pos = motors[1]->final_target_pos.load() - step_rad;

                if (key == 'r' || key == 'R') motors[2]->final_target_pos = motors[2]->final_target_pos.load() + step_rad;
                if (key == 'f' || key == 'F') motors[2]->final_target_pos = motors[2]->final_target_pos.load() - step_rad;
                
                if (key == 't' || key == 'T') motors[3]->final_target_pos = motors[3]->final_target_pos.load() + step_rad;
                if (key == 'g' || key == 'G') motors[3]->final_target_pos = motors[3]->final_target_pos.load() - step_rad;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    reset_terminal_mode();
    if(t.joinable()) t.join();
    std::cout << "\n";
    for (int i = 0; i < NUM_MOTORS; i++) stop_motor(s, motors[i]->id);
    close(s);
    return 0;
}