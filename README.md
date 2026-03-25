# ieve_project2

## 🚀 2. 시스템 노드 실행 (Jetson Zsh 환경)

### 💻 Terminal 1: Camera Perception (Launch)
```zsh
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch perception perception.launch.py
```

### 💻 Terminal 2: LiDAR Perception Node
```zsh
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 run perception_lidar perception_node8
```

### 💻 Terminal 3: Path Planning
```zsh
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 run planning planning_node
```

### 💻 Terminal 4: Control
```zsh
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 run control control_node2
```

### 💻 Terminal 5: Drive Mux & Joy
```zsh
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 run joy joy_node & ros2 run control joy_drive_node & ros2 run control drive_mux_node
```

### 💻 Terminal 6: Serial Bridge (ESP32)
최종 제어 명령을 시리얼 통신을 통해 ESP32 하드웨어로 전송합니다.
```zsh
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 run serial_bridge serial_node2
```
### 💻 Terminal 7: Lidar Streaming
```zsh
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 run urg_node urg_node_driver --ros-args -p ip_address:=192.168.0.10
```
---

## 📌 Pin Mapping

하드웨어 연결 시 아래 핀 맵을 반드시 준수해야 합니다.

| Device | Function | ESP32 Pin | Color | Remarks |
| --- | --- | --- | --- | --- |
| **Steering Servo** | PWM Signal | **D18** | Orange | Steering Control |
| **MD20A Driver** | PWM (Speed) | **D27** | - | **L298N에서 MD20A 드라이버로 변경** |
|  | DIR (Direction) | **D26** | - | Forward / Backward |
| **Encoder** | Phase A | **D32** | Yellow |  |
|  | Phase B | **D33** | Green |  |
|  | VCC / GND | **VIN (5V)** / GND | Blue / Black | **신호 안정화를 위해 3.3V ➡️ 5V로 변경** |

---
## 💻 Source Code (ESP32 PID Control)

실제로 들어갈 아두이노 코드입니다. 차가 직진으로 가도록 trim파라미터를 조정하세요.
```cpp
#include <ESP32Servo.h>

const int SERVO_PIN = 18, ENCODER_A = 32, ENCODER_B = 33, PWM_PIN = 27, DIR_PIN = 26;
const float WHEEL_DIAMETER_MM = 65.0, PI_VAL = 3.141592;
const int PPR = 660;

const float MAX_SPEED = 0.4;
const int MAX_PWM = 100;
const int MIN_PWM = 25;

// ⭐ [수정 1] 직진 영점 조절을 위한 트림(Trim) 파라미터 추가 (단위: Degree)
// 차가 오른쪽으로 쏠리면 이 값을 +2.0, +4.0 등으로 올려보고, 
// 오히려 더 쏠리면 -2.0, -4.0 등 음수로 바꿔가며 완벽한 직진을 찾습니다.
const float STEERING_TRIM_DEG = 3.0; 

double targetV = 0.0, currentV = 0, lastCurrentV = 0;
double Kp = 50.0, Ki = 20.0, Kd = 0.0, Kv = 100.0;
float filterAlpha = 0.3;

double error, cumError, dError;
volatile long encoderTicks = 0;
unsigned long lastTime = 0, lastPrintTime = 0;
Servo steeringServo;

void IRAM_ATTR readEncoder() {
  if (digitalRead(ENCODER_A) == digitalRead(ENCODER_B)) encoderTicks--;
  else encoderTicks++;
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(2);
  
  pinMode(ENCODER_A, INPUT_PULLUP); pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(PWM_PIN, OUTPUT); pinMode(DIR_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), readEncoder, CHANGE);
  
  ESP32PWM::allocateTimer(0);
  steeringServo.setPeriodHertz(50);
  steeringServo.attach(SERVO_PIN, 500, 2400);
  steeringServo.writeMicroseconds(1450);
  lastTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  double dt = (currentTime - lastTime) / 1000.0;

  if (dt >= 0.05) {
    long ticks = encoderTicks; encoderTicks = 0;
    
    float rawV = (((float)ticks / PPR) * (WHEEL_DIAMETER_MM / 1000.0) * PI_VAL) / dt;
    currentV = (filterAlpha * rawV) + ((1.0 - filterAlpha) * currentV);
    error = targetV - currentV;
    
    if (abs(targetV) < 0.01 || abs(error) > 0.2) {
      cumError = 0;
    } else {
      cumError += error * dt;
      cumError = constrain(cumError, -0.5, 0.5);
    }

    double output = (Kv * targetV) + (Kp * error) + (Ki * cumError);

    if (abs(targetV) < 0.01 && abs(currentV) < 0.03) {
      analogWrite(PWM_PIN, 0);
      cumError = 0;
    } else {
      int pwmOut = (int)abs(output);
      if (pwmOut <= 3) {
        analogWrite(PWM_PIN, 0);
      } else {
        pwmOut = constrain(pwmOut + MIN_PWM, MIN_PWM, MAX_PWM);
        digitalWrite(DIR_PIN, (output >= 0) ? LOW : HIGH);
        analogWrite(PWM_PIN, pwmOut);
      }
    }
    lastCurrentV = currentV;
    lastTime = currentTime;
  }

  if (currentTime - lastPrintTime >= 50) {
    Serial.println(currentV);
    lastPrintTime = currentTime;
  }

  // ⭐ 수신함(Buffer) 청소기 로직
  if (Serial.available() > 0) {
    String latestCmd = "";
    
    while (Serial.available() > 0) {
      String temp = Serial.readStringUntil('\n');
      temp.trim();
      if (temp.length() > 0) {
        latestCmd = temp;
      }
    }
    
    if (latestCmd.length() > 0 && !isalpha(latestCmd[0])) {
      int commaIndex = latestCmd.indexOf(',');
      if (commaIndex > 0) {
        targetV = constrain(latestCmd.substring(0, commaIndex).toFloat(), -MAX_SPEED, MAX_SPEED);
        
        float steeringRad = latestCmd.substring(commaIndex + 1).toFloat();
        
        // ⭐ [수정 2] 기본 90도에 STEERING_TRIM_DEG(영점 오프셋)를 더해줍니다.
        float servoAngleDeg = 90.0 - (steeringRad * 180.0 / PI_VAL) + STEERING_TRIM_DEG; 
        
        servoAngleDeg = constrain(servoAngleDeg, 65.0, 115.0);
        
        int pulseWidth = 500 + (int)((2400 - 500) * (servoAngleDeg / 180.0));
        steeringServo.writeMicroseconds(pulseWidth);
      }
    }
  }
}
```


하드웨어 연결 확인용 (아두이노) 코드입니다. 
시리얼창에 다음과 같이 입력해보세요. s45, s90, s135, 1.0 ,0.0, -1.0

```cpp
#include <ESP32Servo.h>

// 1. 핀 설정 (현재 하드웨어 배선 완벽 반영)
const int SERVO_PIN = 18;
const int ENCODER_A = 32; // 노란선
const int ENCODER_B = 33; // 초록선
const int PWM_PIN = 27;   // MD20A PWM
const int DIR_PIN = 26;   // MD20A DIR

// 2. 물리적 사양 설정
const float WHEEL_DIAMETER_MM = 65.0; 
const float PI_VAL = 3.141592;
const int PPR = 150; 

// 3. PID 제어 변수
double setpoint = 0.0;
double Kp = 35.0, Ki = 8.0, Kd = 0.5; 
double error, lastError, cumError, rateError;

// 4. 상태 및 계산 변수
volatile long encoderTicks = 0;
unsigned long lastTime = 0;
unsigned long lastPrintTime = 0; // 출력 주기 조절용 변수
float currentVelocity = 0;
float currentRPM = 0.0;          // 내부 계산용으로 유지
Servo steeringServo;

void IRAM_ATTR readEncoder() {
  if (digitalRead(ENCODER_B) == HIGH) encoderTicks++;
  else encoderTicks--;
}

void setup() {
  Serial.begin(115200);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), readEncoder, RISING);

  ESP32PWM::allocateTimer(0);
  steeringServo.setPeriodHertz(50);
  steeringServo.attach(SERVO_PIN, 500, 2400); 
  steeringServo.write(90); // 초기 조향각 중앙 정렬

  lastTime = millis();
  Serial.println("✅ 시스템 준비 완료!");
}

void loop() {
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastTime;

  // 50ms(0.05초) 마다 모터 속도 제어
  if (deltaTime >= 50) { 
    long ticks = encoderTicks;
    encoderTicks = 0;

    // 1. 회전수 및 RPM 계산
    float rotations = (float)ticks / PPR;
    currentRPM = (rotations / (deltaTime / 1000.0)) * 60.0; // 1분(60초) 기준 회전수

    // 2. 현재 m/s 속도 계산
    float distance = rotations * (WHEEL_DIAMETER_MM / 1000.0) * PI_VAL;
    currentVelocity = abs(distance / (deltaTime / 1000.0));

    // 3. PID 오차 계산
    error = abs(setpoint) - currentVelocity;
    
    if (abs(setpoint) > 0.01) {
      cumError += error * (deltaTime / 1000.0);
    } else {
      cumError = 0;
    }
    
    cumError = constrain(cumError, -5.0, 5.0); 
    rateError = (error - lastError) / (deltaTime / 1000.0);
    double output = (Kp * error) + (Ki * cumError) + (Kd * rateError);
    
    int pwmOut = (int)output;
    if (abs(setpoint) > 0.01) {
      if (pwmOut < 45) pwmOut = 45; // 모터 최소 구동 PWM 보정
    } else {
      pwmOut = 0;
    }
    
    pwmOut = constrain(pwmOut, 0, 255);

    // 4. 모터 구동
    if (setpoint > 0) {
      digitalWrite(DIR_PIN, LOW); 
      analogWrite(PWM_PIN, pwmOut);
    } else if (setpoint < 0) {
      digitalWrite(DIR_PIN, HIGH);  
      analogWrite(PWM_PIN, pwmOut);
    } else {
      digitalWrite(DIR_PIN, LOW);  
      analogWrite(PWM_PIN, 0);     
    }

    lastError = error;
    lastTime = currentTime;
  }

  // 🌟 0.1초(100ms)마다 시리얼 모니터에 목표 속도와 현재 속도(m/s) 출력
  if (currentTime - lastPrintTime >= 100) {
    // 플로터에서 같은 스케일로 보기 위해 RPM 대신 currentVelocity를 출력합니다.
    Serial.print("TargetSpeed:");
    Serial.print(setpoint);
    Serial.print(", CurrentSpeed:");
    Serial.println(currentVelocity);
    
    lastPrintTime = currentTime;
  }

  // 시리얼 통신 파싱 로직 (컴퓨터 -> ESP32 명령 수신)
  if (Serial.available() > 0) {
    String inputStr = Serial.readStringUntil('\n');
    inputStr.trim();
    if (inputStr.length() > 0) {
      if (inputStr.charAt(0) == 's' || inputStr.charAt(0) == 'S') {
        int angle = inputStr.substring(1).toInt();
        steeringServo.write(constrain(angle, 0, 180));
      } else {
        setpoint = inputStr.toFloat();
        cumError = 0; 
      }
    }
  }
}
```
PID 제어 최종 연결용 코드입니다.(아두이노)

```cpp
#include <ESP32Servo.h>

// 1. 핀 설정 
const int SERVO_PIN = 18;
const int ENCODER_A = 32; 
const int ENCODER_B = 33; 
const int PWM_PIN = 27;   
const int DIR_PIN = 26;   

// 2. 물리적 사양 설정
const float WHEEL_DIAMETER_MM = 65.0; 
const float PI_VAL = 3.141592;
const int PPR = 150; 

// 3. PID 제어 변수 (부드러운 주행을 위해 원래 값으로 복구 ⭐)
double setpoint = 0.0;
double Kp = 35.0, Ki = 8.0, Kd = 0.5; 
double error, lastError, cumError, rateError;

// 4. 상태 및 계산 변수
volatile long encoderTicks = 0;
unsigned long lastTime = 0;
unsigned long lastPrintTime = 0; 
float currentVelocity = 0;
Servo steeringServo;

void IRAM_ATTR readEncoder() {
  if (digitalRead(ENCODER_B) == HIGH) encoderTicks++;
  else encoderTicks--;
}

void setup() {
  Serial.begin(115200);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), readEncoder, RISING);

  ESP32PWM::allocateTimer(0);
  steeringServo.setPeriodHertz(50);
  steeringServo.attach(SERVO_PIN, 500, 2400); 
  steeringServo.write(90); // 조향 정중앙

  lastTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastTime;

  // [모터 속도 제어 루프: 50ms 주기]
  if (deltaTime >= 50) { 
    long ticks = encoderTicks;
    encoderTicks = 0;

    // 현재 속도(m/s) 계산
    float rotations = (float)ticks / PPR;
    float distance = rotations * (WHEEL_DIAMETER_MM / 1000.0) * PI_VAL;
    currentVelocity = abs(distance / (deltaTime / 1000.0));

    // PID 연산
    error = abs(setpoint) - currentVelocity;
    
    if (abs(setpoint) > 0.01) {
      cumError += error * (deltaTime / 1000.0);
    } else {
      cumError = 0;
    }
    cumError = constrain(cumError, -5.0, 5.0); 
    
    rateError = (error - lastError) / (deltaTime / 1000.0);
    double output = (Kp * error) + (Ki * cumError) + (Kd * rateError);
    
    int pwmOut = (int)output;
    if (abs(setpoint) > 0.01) {
      if (pwmOut < 45) pwmOut = 45; // 🌟 최소 파워 원래대로 복구
    } else {
      pwmOut = 0;
    }
    pwmOut = constrain(pwmOut, 0, 255);

    // 방향 제어 및 모터 구동
    if (setpoint > 0) {
      digitalWrite(DIR_PIN, LOW);  // 전진
      analogWrite(PWM_PIN, pwmOut);
    } else if (setpoint < 0) {
      digitalWrite(DIR_PIN, HIGH); // 후진
      analogWrite(PWM_PIN, pwmOut);
    } else {
      digitalWrite(DIR_PIN, LOW);  // 정지
      analogWrite(PWM_PIN, 0);     
    }

    lastError = error;
    lastTime = currentTime;
  }

  // [데이터 송신: 100ms 주기]
  // 파이썬 노드가 에러 없이 읽을 수 있도록 순수한 숫자만 전송
  if (currentTime - lastPrintTime >= 100) {
    Serial.println(currentVelocity);
    lastPrintTime = currentTime;
  }

  // [명령 수신: ROS 2 통신 최적화 파싱]
  if (Serial.available() > 0) {
    String inputStr = Serial.readStringUntil('\n');
    inputStr.trim();
    
    if (inputStr.length() > 0) {
      int commaIndex = inputStr.indexOf(',');
      
      if (commaIndex > 0) {
        // "속도,조향각(라디안)" 형태 완벽 파싱
        String speedStr = inputStr.substring(0, commaIndex);
        String steerStr = inputStr.substring(commaIndex + 1);

        setpoint = speedStr.toFloat();
        cumError = 0; 

        // 조향 방향 반전 유지 (+ 기호를 - 로 적용됨)
        float steeringRad = steerStr.toFloat();
        int servoAngle = 90 - (int)(steeringRad * 180.0 / PI_VAL); 
        steeringServo.write(constrain(servoAngle, 0, 180));
      }
    }
  }
}
```
PID 개발을 위한 그래프 확인용 코드 입니다.

```cpp
#include <ESP32Servo.h>

// 1. 핀 설정 
const int SERVO_PIN = 18;
const int ENCODER_A = 32; 
const int ENCODER_B = 33; 
const int PWM_PIN = 27;   
const int DIR_PIN = 26;   

// 2. 물리적 사양 설정
const float WHEEL_DIAMETER_MM = 65.0; 
const float PI_VAL = 3.141592;
const int PPR = 150; 

// 3. PID 제어 변수 (부드러운 주행 세팅)
double setpoint = 0.0;
double Kp = 35.0, Ki = 8.0, Kd = 0.5; 
double error, lastError, cumError, rateError;

// 4. 상태 및 계산 변수
volatile long encoderTicks = 0;
unsigned long lastTime = 0;
unsigned long lastPrintTime = 0; 
float currentVelocity = 0;
Servo steeringServo;

void IRAM_ATTR readEncoder() {
  if (digitalRead(ENCODER_B) == HIGH) encoderTicks++;
  else encoderTicks--;
}

void setup() {
  Serial.begin(115200);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), readEncoder, RISING);

  ESP32PWM::allocateTimer(0);
  steeringServo.setPeriodHertz(50);
  steeringServo.attach(SERVO_PIN, 500, 2400); 
  steeringServo.write(90); // 조향 정중앙

  lastTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastTime;

  // [모터 속도 제어 루프: 50ms 주기]
  if (deltaTime >= 50) { 
    long ticks = encoderTicks;
    encoderTicks = 0;

    // 현재 속도(m/s) 계산
    float rotations = (float)ticks / PPR;
    float distance = rotations * (WHEEL_DIAMETER_MM / 1000.0) * PI_VAL;
    currentVelocity = abs(distance / (deltaTime / 1000.0));

    // PID 연산
    error = abs(setpoint) - currentVelocity;
    
    if (abs(setpoint) > 0.01) {
      cumError += error * (deltaTime / 1000.0);
    } else {
      cumError = 0;
    }
    cumError = constrain(cumError, -5.0, 5.0); 
    
    rateError = (error - lastError) / (deltaTime / 1000.0);
    double output = (Kp * error) + (Ki * cumError) + (Kd * rateError);
    
    int pwmOut = (int)output;
    if (abs(setpoint) > 0.01) {
      if (pwmOut < 45) pwmOut = 45; 
    } else {
      pwmOut = 0;
    }
    pwmOut = constrain(pwmOut, 0, 255);

    // 방향 제어 및 모터 구동
    if (setpoint > 0) {
      digitalWrite(DIR_PIN, LOW);  // 전진
      analogWrite(PWM_PIN, pwmOut);
    } else if (setpoint < 0) {
      digitalWrite(DIR_PIN, HIGH); // 후진
      analogWrite(PWM_PIN, pwmOut);
    } else {
      digitalWrite(DIR_PIN, LOW);  // 정지
      analogWrite(PWM_PIN, 0);     
    }

    lastError = error;
    lastTime = currentTime;
  }

  // ⭐ [데이터 송신: 100ms 주기] 
  // 시리얼 플로터에서 두 그래프를 겹쳐 보기 위한 포맷으로 수정!
  if (currentTime - lastPrintTime >= 100) {
    Serial.print("TargetSpeed:");
    Serial.print(setpoint);
    Serial.print(", CurrentSpeed:");
    Serial.println(currentVelocity);
    lastPrintTime = currentTime;
  }

  // [명령 수신: 수동 + ROS 2 통신 파싱]
  if (Serial.available() > 0) {
    String inputStr = Serial.readStringUntil('\n');
    inputStr.trim();
    
    if (inputStr.length() > 0) {
      // 케이스 1: 수동 조향 테스트 (예: s45)
      if (inputStr.charAt(0) == 's' || inputStr.charAt(0) == 'S') {
        int angle = inputStr.substring(1).toInt();
        steeringServo.write(constrain(angle, 0, 180));
      } 
      // 케이스 2: 쉼표가 있는 ROS 2 방식 (예: 1.0,0.5)
      else if (inputStr.indexOf(',') > 0) {
        int commaIndex = inputStr.indexOf(',');
        String speedStr = inputStr.substring(0, commaIndex);
        String steerStr = inputStr.substring(commaIndex + 1);

        setpoint = speedStr.toFloat();
        cumError = 0; 

        float steeringRad = steerStr.toFloat();
        int servoAngle = 90 - (int)(steeringRad * 180.0 / PI_VAL); 
        steeringServo.write(constrain(servoAngle, 0, 180));
      }
      // 케이스 3: 쉼표 없는 단순 속도 (예: 1.0)
      else {
        setpoint = inputStr.toFloat();
        cumError = 0;
      }
    }
  }
}
```
>>>>>>> a0e9abe (Initial commit: iexe_project2 시뮬레이터 및 플래닝 통합 완료)
