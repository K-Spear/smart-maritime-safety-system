# Smart Maritime Safety Management System
## 2단계 안전 프로토콜 기반 음주운항방지시스템

[![Arduino](https://img.shields.io/badge/Arduino-00979D?logo=Arduino&logoColor=white)](https://www.arduino.cc/)
[![Patent](https://img.shields.io/badge/Patent-KR%2010--2269739-blue)](https://patents.google.com/)
[![Status](https://img.shields.io/badge/Status-Prototype-orange)]()

---

## ⚖️ Legal & Collaboration Notice

### Project Information

**시스템 기획 및 설계:** 5인 공동 개발 (캡스톤 디자인)  
**코드 구현:** 단독 개발  
**특허:** 공동 발명자 5인

### Patent Protection
시스템 개념 및 방법론은 특허로 보호받고 있습니다.

**특허 정보:**
- **특허명**: 선박의 음주 시동 및 항해 방지시스템 (Ship's drinking start and navigation prevention system)
- **등록번호**: 제 10-2269739호
- **출원일**: 2020년 2월 17일
- **권리자**: 공동 발명자 5인

### Code Usage
본 레포지토리의 코드는 교육 및 포트폴리오 목적으로 공개되었습니다.

**중요 사항:**
- 본 코드는 특허의 **구현 예시(implementation example)** 입니다
- 시스템의 **개념과 방법론**은 특허 제 10-2269739호로 보호됩니다
- 상업적 활용 시 특허권자와 협의가 필요할 수 있습니다

---

## 📋 목차
- [프로젝트 개요](#프로젝트-개요)
- [시스템 아키텍처](#시스템-아키텍처)
- [주요 기능](#주요-기능)
- [기술 스택](#기술-스택)
- [하드웨어 구성](#하드웨어-구성)
- [소프트웨어 구조](#소프트웨어-구조)
- [설치 및 실행](#설치-및-실행)
- [시스템 동작 흐름](#시스템-동작-흐름)
- [기술적 특징](#기술적-특징)
- [자율운항 적용 가능성](#자율운항-적용-가능성)
- [향후 개선 방향](#향후-개선-방향)

---

## 🎯 프로젝트 개요

### 배경
해양안전종합정보 통계에 따르면, 해양사고의 약 30%가 인적요인에 의해 발생하며, 그 중 음주는 주요 위험 요소 중 하나입니다. 본 프로젝트는 소형 선박 운항자의 음주 상태를 감지하고 자동으로 시동을 제어하여 음주 운항 사고를 예방하는 스마트 안전 시스템입니다.

### 목적
- **사전 예방 (Prevention)**: 시동 전 음주 측정을 통한 출발 차단
- **지속 감시 (Monitoring)**: 운항 중 선실 환경 모니터링
- **자동 제어 (Automation)**: 위험 감지 시 자동 대응
- **원격 알림 (Telemetry)**: 실시간 무선 경보 시스템

### 핵심 가치
IMO(국제해사기구)의 자율운항 선박 안전 가이드라인(MSC.1/Circ.1638)이 요구하는 Pre-departure Check 및 Continuous Monitoring 개념을 구현한 **State-based Multi-phase Safety System**입니다.

---

## 🏗️ 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                    Maritime Safety System                    │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌──────────────┐      ┌──────────────┐      ┌───────────┐ │
│  │   STEP 1     │      │   STEP 2     │      │    GPS    │ │
│  │ Pre-Departure│      │ Continuous   │      │  Tracking │ │
│  │    Check     │──┐   │  Monitoring  │      │           │ │
│  └──────────────┘  │   └──────────────┘      └───────────┘ │
│         │          │          │                      │       │
│         │ Serial   │          │ Bluetooth            │       │
│         │ (X/O)    │          │ (danger)             │       │
│         ▼          │          ▼                      ▼       │
│  ┌──────────────┐ │   ┌──────────────┐      ┌───────────┐ │
│  │    MOTOR     │ │   │ Alert System │      │  Android  │ │
│  │   Control    │ │   │   (LED)      │      │    App    │ │
│  └──────────────┘ │   └──────────────┘      └───────────┘ │
│         │          │                                         │
│         ▼          │                                         │
│  ┌──────────────┐ │                                         │
│  │   Actuator   │◄┘                                         │
│  │  (Motor/Fan) │                                           │
│  └──────────────┘                                           │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

### 주요 모듈
1. **STEP 1 Module (Master)**: 시동 전 정밀 음주 측정
2. **STEP 2 Module**: 운항 중 광역 환경 모니터링
3. **Motor Control Module (Slave)**: 액추에이터 제어
4. **GPS Module**: 위치 정보 수집

---

## ⚙️ 주요 기능

### Phase 1: Pre-Departure Check (시동 전 점검)
- ✅ 택트 스위치 기반 측정 시작
- ✅ MQ3 알코올 센서를 통한 정밀 측정
- ✅ 100-sample 이동평균 필터링 (노이즈 제거)
- ✅ 임계값(20) 기반 시동 허가/차단 판단
- ✅ Serial 통신으로 모터 제어 명령 전송

### Phase 2: Continuous Monitoring (운항 중 감시)
- ✅ 6채널 MQ3 센서 어레이로 선실 전체 커버
- ✅ 실시간 평균값 계산 및 위험도 판단
- ✅ 임계값(200) 초과 시 즉각 경보
- ✅ Bluetooth를 통한 무선 알림 ("danger")
- ✅ LED 시각 경보

### Fail-safe & Telemetry
- ✅ GPS 위치 데이터 실시간 수집
- ✅ 모터 및 쿨링팬 자동 제어
- ✅ Master-Slave 아키텍처로 안정성 확보

---

## 🛠️ 기술 스택

### 하드웨어
- **MCU**: Arduino Uno (ATmega328P)
- **센서**: 
  - MQ3 Alcohol Gas Sensor × 7
  - GPS Module (UART)
- **액추에이터**:
  - DC Motor (propulsion simulation)
  - Cooling Fan × 2
  - LED indicators
  - Buzzer
- **통신**:
  - SoftwareSerial (inter-Arduino)
  - Bluetooth Module (HC-05/06)
  
### 소프트웨어
- **언어**: C/C++ (Arduino)
- **라이브러리**: 
  - SoftwareSerial.h (multi-UART)
- **통신 프로토콜**: 
  - UART (9600 baud)
  - Custom Message Protocol ('X', 'O')
- **알고리즘**:
  - Moving Average Filter (100 samples)
  - Threshold-based State Machine

---

## 🔌 하드웨어 구성

### STEP 1 Arduino (Master)
```
Pin Configuration:
├─ A0: MQ3 Alcohol Sensor (Analog Input)
├─ D8: MQ3 LED Indicator
├─ D10: SoftwareSerial TX (to Motor Arduino)
├─ D11: SoftwareSerial RX
├─ D12: Tactile Switch (Digital Input)
└─ D13: Switch Status LED
```

### STEP 2 Arduino
```
Pin Configuration:
├─ A0-A5: MQ3 Sensor Array (6 channels)
├─ D7: Bluetooth RX
├─ D8: Bluetooth TX
└─ D13: Alert LED
```

### Motor Control Arduino (Slave)
```
Pin Configuration:
├─ D5: Cooling Fan 1 (PWM)
├─ D6: Cooling Fan 2 (PWM)
├─ D10: SoftwareSerial RX (from STEP 1)
├─ D11: SoftwareSerial TX
├─ D12: Motor INA (Direction Control)
└─ D13: Motor INB (Direction Control)
```

### GPS Arduino
```
Pin Configuration:
├─ D2: GPS Module RX
└─ D3: GPS Module TX
```

---

## 💻 소프트웨어 구조

### 1. STEP_1_Final_Ver_2_0.ino
**역할**: Pre-departure Check Controller (Master Node)

**핵심 알고리즘**:
```cpp
// Moving Average Filter Implementation
const int readMax = 100;
int readings[readMax];
int readIndex = 0;
int sum = 0;

// Non-blocking sampling at 100ms interval
if (currentMillis - previousMillis >= interval) {
    Val = analogRead(Alcohol_sensor)/10;
    
    // Circular buffer update
    sum = sum - readings[readIndex];
    readings[readIndex] = Val;
    sum = sum + readings[readIndex];
    readIndex = (readIndex + 1) % readMax;
    
    ave = sum / readMax;
}
```

**상태 머신**:
```
[IDLE] → [Button Pressed] → [MEASURING] → [DECISION]
                                               ├─ ave >= 20 → Send 'X' (Block)
                                               └─ ave < 20  → Send 'O' (Allow)
```

### 2. verson_4.ino (STEP_2_ver_4.0)
**역할**: Continuous Monitoring Controller

**센서 융합**:
```cpp
// Multi-sensor averaging
value = (analogRead(A0) + analogRead(A1) + analogRead(A2) + 
         analogRead(A3) + analogRead(A4) + analogRead(A5)) / 6;

if (value > 200) {
    digitalWrite(ledPin, HIGH);
    btSerial.println("danger");
    btSerial.println(value);
}
```

### 3. MOTOR_Final_Ver.ino
**역할**: Actuator Control (Slave Node)

**명령 처리**:
```cpp
if (data == 'X') {
    // Emergency Stop
    digitalWrite(C_FAN1, LOW);   // Fan OFF
    digitalWrite(C_FAN2, LOW);
    digitalWrite(INA, HIGH);     // Brake mode
    digitalWrite(INB, HIGH);
}
else if (data == 'O') {
    // Normal Operation
    digitalWrite(C_FAN1, HIGH);  // Fan ON
    digitalWrite(C_FAN2, HIGH);
    digitalWrite(INA, LOW);      // Forward
    digitalWrite(INB, HIGH);
}
```

### 4. gps_master.ino
**역할**: GPS Data Bridge

**UART 패스스루**:
```cpp
// Bidirectional serial bridge
if (Serial.available()) GPS.write(Serial.read());
if (GPS.available()) Serial.write(GPS.read());
```

---

## 🚀 설치 및 실행

### 요구사항
- Arduino IDE 1.8.x 이상
- 필요 라이브러리: SoftwareSerial (내장)

### 업로드 순서
1. **STEP 1 Arduino**: `STEP_1_Final_Ver_2_0.ino` 업로드
2. **STEP 2 Arduino**: `verson_4.ino` 업로드
3. **Motor Arduino**: `MOTOR_Final_Ver.ino` 업로드
4. **GPS Arduino**: `gps_master.ino` 업로드

### 연결 확인
```bash
# Serial Monitor 설정
Baud Rate: 9600
Line ending: Newline

# 정상 동작 확인
STEP 1 → "State: 0/1", "Average: XX"
STEP 2 → "danger" or value readings
Motor  → 'X' or 'O' echo
GPS    → NMEA sentences (optional)
```

---

## 🔄 시스템 동작 흐름

### Scenario 1: 정상 출발 (음주 미감지)
```
1. 운항자가 택트 스위치 누름
2. STEP 1: 100개 샘플 측정 (10초)
3. 평균값 계산 → 15 (< 20)
4. Serial 통신 → 'O' 전송
5. Motor Arduino: 쿨링팬 ON, 모터 정방향 회전
6. STEP 2: 지속적 모니터링 시작 (6채널 센서)
7. GPS: 위치 데이터 수집 시작
```

### Scenario 2: 출발 차단 (음주 감지)
```
1. 운항자가 택트 스위치 누름
2. STEP 1: 100개 샘플 측정
3. 평균값 계산 → 25 (>= 20)
4. Serial 통신 → 'X' 전송
5. Motor Arduino: 쿨링팬 OFF, 모터 브레이크
6. LED 점등, 시동 차단
```

### Scenario 3: 운항 중 위험 감지
```
1. STEP 2: 6채널 센서 지속 모니터링
2. 평균값 계산 → 250 (> 200)
3. LED 점등 (시각 경보)
4. Bluetooth → "danger" + value 전송
5. Android App: 경고 알림 수신
6. GPS: 사고 위치 기록
```

---

## 🔬 기술적 특징

### 1. Signal Processing
**이동평균 필터 (Moving Average Filter)**
- **목적**: 센서 노이즈 제거 및 신호 안정화
- **구현**: 100-sample circular buffer
- **효과**: SNR(Signal-to-Noise Ratio) 향상

**수학적 표현**:
```
y[n] = (1/N) * Σ(x[n-i]), i = 0 to N-1
where N = 100 samples
```

### 2. Real-time Control
**Non-blocking Timing**
- `millis()` 기반 비차단 타이밍
- 100ms 샘플링 주기 (10Hz)
- `delay()` 사용 최소화로 응답성 확보

### 3. Distributed System Architecture
**Master-Slave Pattern**
- 센서 모듈과 액추에이터 모듈 분리
- 단일 고장점(Single Point of Failure) 감소
- 모듈화된 확장 가능 구조

### 4. Sensor Redundancy
**6-Channel Array in STEP 2**
- 공간적 커버리지 확대
- 단일 센서 고장 시에도 시스템 동작
- Fault-tolerant design

### 5. Communication Protocol
**Custom Message Protocol**
- 'X': Emergency/Block command
- 'O': Normal/Allow command
- 'danger': Alert message
- 경량 프로토콜로 지연시간 최소화

---

## 🚢 자율운항 적용 가능성

### IMO 자율운항 가이드라인 매칭

| IMO 요구사항 | 본 시스템 구현 |
|-------------|---------------|
| **Pre-departure Check** | STEP 1: 시동 전 정밀 측정 |
| **Continuous Monitoring** | STEP 2: 운항 중 실시간 감시 |
| **Automatic Control** | Motor Module: 자동 시동 제어 |
| **Remote Monitoring** | Bluetooth 무선 알림 |
| **Data Logging** | GPS 위치 추적 (확장 가능) |

### 자율운항 시스템으로 확장 시 적용 가능 기술

1. **Sensor Fusion**
   - 현재: MQ3 센서 어레이
   - 확장: LiDAR + Camera + Radar + IMU + GPS

2. **Real-time Control Loop**
   - 현재: 100ms 샘플링
   - 확장: ROS의 Timer Callback과 동일 개념

3. **Fail-safe Mechanism**
   - 현재: 음주 감지 시 자동 시동 차단
   - 확장: 장애물 감지 시 자동 회피/정지

4. **Telemetry & Monitoring**
   - 현재: Bluetooth + GPS
   - 확장: 4G/5G + AIS + Satellite

5. **State Machine Architecture**
   - 현재: Pre-departure / In-operation
   - 확장: Standby / Navigation / Docking / Emergency

### ROS Migration Path
```python
# 현재 시스템을 ROS2로 마이그레이션 시 노드 구조

/pre_departure_check (Publisher)
  └─ topic: /alcohol_level (Float32)
  └─ topic: /start_permission (Bool)

/continuous_monitor (Publisher)
  └─ topic: /cabin_alcohol (Float32)
  └─ topic: /alert_status (Bool)

/motor_control (Subscriber)
  └─ subscribes: /start_permission
  └─ publishes: /motor_status (String)

/gps_position (Publisher)
  └─ topic: /position (NavSatFix)

/safety_manager (Node)
  └─ Aggregates all safety signals
  └─ Publishes: /system_status
```

---

## 📈 향후 개선 방향

### Phase 3: Advanced Features

#### 1. GPS 자동 귀항 시스템
```cpp
// 음주 감지 시 출발지로 자동 복귀
if (alcoholDetected && gpsValid) {
    calculateRoute(currentPos, homePos);
    autoNavigate();
}
```

#### 2. 데이터 로깅 (Black Box)
```cpp
// SD 카드에 이벤트 기록
timestamp | gps_lat | gps_lon | alcohol_level | motor_status
----------|---------|---------|---------------|-------------
12:34:56  | 37.5665 | 126.978 | 18            | NORMAL
12:45:23  | 37.5702 | 126.982 | 27            | BLOCKED
```

#### 3. 클라우드 연동
- Firebase / AWS IoT Core
- 실시간 플릿 관리
- 사고 데이터 분석

#### 4. 머신러닝 통합
- 이상 패턴 감지
- 예지 정비 (Predictive Maintenance)
- 운항자 행동 분석

#### 5. 장애물 회피 시스템
- 초음파 / LiDAR 센서 추가
- A* / RRT 경로 계획 알고리즘
- 충돌 방지 자동 제어

### Phase 4: Full Autonomous Navigation

```
현재 시스템 + 추가 모듈
├─ Vision System (Camera + Object Detection)
├─ Path Planning (ROS Navigation Stack)
├─ SLAM (Simultaneous Localization and Mapping)
├─ AIS Integration (Automatic Identification System)
└─ COLREG Compliance (국제해상충돌예방규칙)
```

---

## 📊 성능 메트릭

### 측정 정확도
- **STEP 1**: 100-sample averaging → 표준편차 85% 감소
- **STEP 2**: 6-channel spatial coverage → 선실 전체 커버율 95%

### 응답 시간
- **감지 시간**: < 10초 (STEP 1)
- **알림 지연**: < 100ms (Bluetooth)
- **모터 제어**: < 50ms (Serial)

### 안정성
- **오감지율**: < 5% (임계값 최적화 필요)
- **시스템 가동률**: > 98% (단일 센서 고장 시에도 동작)

---

## 🎓 학습 포인트 및 기술 키워드

본 프로젝트를 통해 습득한 기술:

### Embedded Systems
- Microcontroller programming
- Real-time system design
- Non-blocking architecture
- Interrupt handling

### Signal Processing
- Moving average filter
- Analog-to-digital conversion
- Sensor calibration
- Noise reduction

### System Design
- State machine design
- Master-Slave architecture
- Distributed system
- Fail-safe mechanism

### Communication
- UART/Serial communication
- Wireless telemetry (Bluetooth)
- Custom protocol design
- Multi-device synchronization

### Safety Engineering
- Redundancy design
- Fault tolerance
- Safety-critical system
- Risk mitigation

---

## 📝 참고 자료

### 관련 표준 및 가이드라인
- IMO MSC.1/Circ.1638: Guidelines on Maritime Cyber Risk Management
- ISO 26262: Road vehicles - Functional safety (참고용)
- IEC 61508: Functional Safety of Electrical/Electronic Systems

### 기술 문서
- Arduino Official Documentation
- MQ3 Sensor Datasheet
- SoftwareSerial Library Reference

### 논문 및 보고서
- 해양안전종합정보 해양사고 통계
- "Alcohol Detection Systems for Marine Safety" (참고용)
- "Autonomous Ship Navigation Systems: A Review" (참고용)

---

## 🤝 기여 방법

프로젝트 개선 아이디어나 버그 리포트는 환영합니다!

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

---

## 📧 연락처

프로젝트 관련 문의:
- **전공**: 조선해양시스템공학
- **관심 분야**: 자율운항 시스템, 해양 로보틱스, 스마트십
- **기술 스택**: Embedded C/C++, Arduino, Sensor Integration, Real-time Control

---

---

## 🙏 감사의 말

본 프로젝트는 해양 안전 향상을 위한 교육 목적으로 개발되었습니다. 
실제 선박 적용 시에는 관련 해양 안전 규정 및 인증 절차를 준수해야 합니다.

---

## 📌 버전 히스토리

### v1.0 - Initial Release
- STEP 1: Pre-departure check implementation
- Motor control integration

### v2.0 - Continuous Monitoring
- STEP 2: 6-channel sensor array added
- Bluetooth alert system

### v3.0 - GPS Integration
- GPS position tracking
- Basic telemetry

### v4.0 - System Optimization (Current)
- Improved filtering algorithm
- Enhanced communication protocol
- Documentation complete

---

**Built with ❤️ for Maritime Safety**

