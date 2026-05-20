# Robstride CAN Study

> RobStride QDD 액추에이터를 SocketCAN 기반 C++ 환경에서 제어하기 위한 스터디 레포지토리.  
> [Seeed Studio RobStride_Control](https://github.com/Seeed-Projects/RobStride_Control) 오픈소스를 기반으로, QUB 휴머노이드 로봇 프로젝트에 맞게 수정 및 재구성하였습니다.

---

## 📚 참고 자료 (References)

- **베이스 오픈소스**: [Seeed Studio — RobStride_Control](https://github.com/Seeed-Projects/RobStride_Control)
- **Seeed Studio 공식 가이드**: [robstride_control wiki](https://wiki.seeedstudio.com/robstride_control/)
- **RobStride 공식 문서 (커맨드 프로토콜)**: [Product Information](https://github.com/RobStride/Product_Information/tree/main/Product%20Literature)

헤더 파일 및 핵심 cpp 원본은 위 오픈소스에서 기반하였으며, QUB 프로젝트 환경에 맞게 수정하였습니다.

---

## 🛠 환경 (Environment)

| 항목 | 내용 |
|---|---|
| **OS** | Ubuntu 22.04 LTS |
| **Language** | C++ |
| **Build System** | CMake |
| **Target Actuator** | RobStride QDD RS02 / RS03 / RS04 |


---

## 🚀 빌드 및 실행 (Build & Run)

### 1. 필수 패키지 설치

```bash
sudo apt-get update
sudo apt-get install cmake g++ build-essential can-utils
```

### 2. 레포 클론

```bash
git clone https://github.com/jaebin401/Robstrid-CAN_study.git
cd Robstrid-CAN_study
```

### 3. CAN 인터페이스 설정

Peak CAN M.2 사용 시:

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

정상 동작 확인:

```bash
candump can0
```

### 4. 컴파일 및 실행

```bash
cd src
g++ position_control.cpp -o run
sudo ./run
```

---

## ⚙️ 코드 설정

`src/position_control.cpp` 상단의 Configuration 섹션에서 환경에 맞게 수정합니다.

```cpp
// --- Configuration ---
const char* CAN_INTERFACE = "can0";         // CAN 채널 지정
const int HOST_ID = 0xFD;

const int NUM_MOTORS = 2;                   // 제어할 모터 수
const int MOTOR_IDS[NUM_MOTORS] = {1, 2};  // 각 모터의 CAN ID
```

---

## 🎮 실행 방법

프로그램 실행 시 터미널 GUI가 표시됩니다.

- 시작 시 모터는 **비활성화(off)** 상태이며 현재 각도가 실시간으로 표시됩니다.
- off 상태에서 모터를 손으로 돌리면 각도값이 변하는 것으로 CAN 통신 정상 동작을 확인할 수 있습니다.
- **Space** 키로 모터 활성화(on) 전환

활성화 후 키보드 제어:

| 키 | 동작 |
|---|---|
| `W` / `S` | 1번 모터 + / − |
| `E` / `D` | 2번 모터 + / − |
| `R` / `F` | 3번 모터 + / − |
| `T` / `G` | 4번 모터 + / − |

---

## 🔗 관련 레포지토리

본 레포지토리는 [QUB 휴머노이드 로봇 프로젝트](https://github.com/jaebin401/QUB_humanoid)의 일부입니다.

| 레포 | 설명 |
|---|---|
| [QUB_humanoid]| 프로젝트 전체 개요 (umbrella repo), 작성중|
| [QUB_URDF](https://github.com/jaebin401/QUB_URDF) | 로봇 형상 정보 (URDF + mesh) |
| [QUB_RL](https://github.com/jaebin401/QUB_RL) | Humanoid-gym 오픈소스 기반 isaac gym 강화학습 |
| [QUB_RL_v2](https://github.com/jaebin401/QUB_RL_v2) | tron1 오픈소스 기반 isaac gym 강화학습 |

---

## 👤 작성자 (Author)

**Jaebin Ahn (@jaebin401)**  
학부 기계공학 전공 / 소프트웨어 부전공  
KUDOS 로봇 동아리 · Apple Developer Academy  

- GitHub: [@jaebin401](https://github.com/jaebin401)
- Instagram: [통학하는 공대생](https://www.instagram.com/study_4_machine/)

---

## Acknowledgments

- [RobStride_Control](https://github.com/Seeed-Projects/RobStride_Control) — Seeed Studio
- [legged_gym](https://github.com/leggedrobotics/legged_gym) — ETH Zurich Robotic Systems Lab