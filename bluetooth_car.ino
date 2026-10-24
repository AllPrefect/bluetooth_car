/* ============================================================
   蓝牙遥控 + 自动避障小车 (Arduino Uno)
   ------------------------------------------------------------
   ┌── 完整接线表 ──────────────────────────────────────────────┐
   │                                                             │
   │  L298N 电机驱动（D2~D7 连续排针）：                          │
   │    IN1  → D2            左电机方向 A                      │
   │    IN2  → D3            左电机方向 B                      │
   │    IN3  → D4            右电机方向 A                      │
   │    ENA  → D5   (PWM)    左电机调速                        │
   │    ENB  → D6   (PWM)    右电机调速                        │
   │    IN4  → D7            右电机方向 B                      │
   │    +12V → 外部 7~12V 电池                                 │
   │    GND  → Arduino GND（必须共地！）                        │
   │    ※ 电机方向已软件补偿（接线勿再改动）：                  │
   │      左电机 IN1/IN2 引脚定义对调(3/2)，                    │
   │      F/B 与左/右转函数内电平对调                          │
   │                                                             │
   │  HC-SR04 超声波：                                          │
   │    VCC  → 5V                                               │
   │    GND  → GND                                              │
   │    Trig → D12                                              │
   │    Echo → D11                                              │
   │                                                             │
   │  舵机 (SG90 等)：                                          │
   │    红   → 5V / 独立 6V（大舵机建议独立供电）                │
   │    棕黑 → GND                                              │
   │    橙黄 → D10 (PWM)                                        │
   │                                                             │
   │  OLED 0.96" SSD1306 (I²C)：                                │
   │    VCC → 3.3V                                              │
   │    GND → GND                                               │
   │    SDA → A4                                                │
   │    SCL → A5                                                │
   │                                                             │
   │  蓝牙 (HC-05/HC-06)：                                      │
   │    VCC → 5V                                                │
   │    GND → GND                                               │
   │    TX  → D8   (SoftwareSerial RX，交叉接)                  │
   │    RX  → D9   (SoftwareSerial TX，交叉接)                  │
   │                                                             │
   │  状态 LED：                                                 │
   │  正极──220Ω──A0   负极──GND                                │
   │                                                             │
   └─────────────────────────────────────────────────────────────┘

   ┌── 蓝牙命令 ─────────────────────────────────────────────────┐
   │  F/B/L/R/S   手动：点动前进/后退/左转/右转/停止 → 切 MANUAL │
   │  A / a       切回 AUTO 自动避障模式                         │
   │  + / U       加速 (PWM +20)                                │
   │  - / D       减速 (PWM -20)                                │
   │  1~9         速度档位 (80~255，1档=最低可运转)              │
   │  T / t       舵机自检：左-中-右-中 各停600ms                │
   └─────────────────────────────────────────────────────────────┘

   码盘测速模块 ×2（左右轮各一）：
     VCC → 5V   GND → GND   左轮 OUT → A1   右轮 OUT → A2

   依赖库（Arduino IDE → 工具 → 管理库 → 搜索安装）：
     • SoftwareSerial          (Arduino 内置，IDE 自带)
     • SSD1306Ascii           (OLED 文本驱动，几乎不占 RAM)
     • Servo                  (Arduino 内置，IDE 自带)
     （超声波不用库：NewPing 强占 Timer2 中断，与码盘计数冲突）
   ============================================================ */

#include <Wire.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include <SoftwareSerial.h>
#include <Servo.h>

/* ==================== 引脚定义 ==================== */

// 左电机 IN1/IN2 软件对调：实测左电机转向接反（按 L 变前进、按 R 变后退）
#define LEFT_IN1 3
#define LEFT_IN2 2
#define LEFT_ENA 5   // PWM（Timer0，不受舵机影响）

#define RIGHT_IN3 4
#define RIGHT_IN4 7
#define RIGHT_ENB 6  // PWM

#define SONAR_TRIG 12
#define SONAR_ECHO 11
#define SONAR_MAX_CM 200

#define SERVO_PIN 10
#define SERVO_CENTER 90                   // 中位校准：探头偏左就调小，偏右就调大
#define SERVO_LEFT  (SERVO_CENTER + 60)   // 实测：角度大的一侧指向车身左侧
#define SERVO_RIGHT (SERVO_CENTER - 60)

#define STATUS_LED A0

/* 码盘测速（左右轮各一，Timer2 每 1ms 轮询计数；
   SoftwareSerial 占用全部 PCINT 中断向量，码盘不能用 PCINT1） */
#define ENC_LEFT_PIN   A1
#define ENC_RIGHT_PIN  A2
#define ENC_PPR        20      // 码盘每圈孔数，套件常见 20，按实物改
#define WHEEL_CIRC_CM  20.4f   // 轮周长 = π×直径，65mm 轮约 20.4cm，按实测改

/* ==================== 全局对象 ==================== */

SoftwareSerial *bluetoothSerial;
Servo scanServo;
SSD1306AsciiWire display;  // 文本模式驱动，无 1KB 显存缓冲，Uno RAM 够用
bool oledOk = false;       // OLED 在线标志：不在线时跳过全部显示操作，不影响车辆控制

/* ==================== 模式 & 状态 ==================== */

enum CarMode { MODE_AUTO,
               MODE_MANUAL };
enum AutoState {
  AUTO_FORWARD,
  AUTO_OBSTACLE_STOP,
  AUTO_SCAN_LEFT,
  AUTO_SCAN_RIGHT,
  AUTO_TURN_LEFT,
  AUTO_TURN_RIGHT,
  AUTO_BACK_UP   // 死胡同：两侧都堵，倒车脱困后重新扫描
};

CarMode currentMode = MODE_MANUAL;  // 默认手动待命，电机不启动；发 A 进自动，发 F/B/L/R 手动行驶
AutoState autoState = AUTO_FORWARD;

/* ==================== 避障参数 ==================== */

const int OBSTACLE_THRESHOLD = 25;   // 正前方刹停距离 cm
const int APPROACH_DIST = 50;        // 接近减速区：进入该距离提前降速，避免全速怼到跟前急刹
const int SIDE_CLEAR_MIN = 30;       // 左右至少这么空才算可通行，两侧都不够 = 死胡同
const int TURN_TIME_MS = 800;        // 基准转向时长（对应基准速度 180，实际按速度缩放）
const int BACKUP_TIME_MS = 600;      // 死胡同倒车时长（同样按速度缩放）
const int SCAN_TIME_MS = 300;
const int STALL_TIME_MS = 1200;      // 给油但车轮不转的判定时长（卡住逃逸，需码盘在线）

int frontDistance = 0;
int leftDistance = 0;
int rightDistance = 0;

int currentSpeed = 180;
const int SPEED_STEP = 20;
const int MIN_RUN_PWM = 80;  // 电机死区：低于该值只哼不转，按电池电量可在 60~100 微调

char bluetoothCmd = 0;
unsigned int cmdSeq = 0;  // 每收到一条有效蓝牙命令 +1，OLED 据此刷新
char lastManualCmd = 'S';  // 手动模式最近一次运动命令
const unsigned long MANUAL_STEP_MS = 500;  // 点动时长：每条 F/B/L/R 执行这么久后自动停车
unsigned long manualStepEndMs = 0;         // 当前点动步的截止时刻
unsigned long lastCmdMs = 0;  // 最近一次收到蓝牙命令的时间（LED 闪烁反馈用）
bool manualBlocked = false;   // 手动前进被前方障碍自动拦停中

/* 码盘计数（中断内累加） */
volatile unsigned int encLeftTicks = 0;
volatile unsigned int encRightTicks = 0;
volatile uint8_t encLastState = 0;
float leftSpeedCms = 0;   // 实测车速 cm/s
float rightSpeedCms = 0;
bool encSeen = false;     // 码盘至少出过一次脉冲；未接线时保持 false，自动模式停用卡住检测

/* ==================== 非阻塞调度时间戳 ==================== */

unsigned long lastSonarMs = 0;
unsigned long lastOledMs = 0;
unsigned long lastEncCalcMs = 0;
unsigned long stateStartMs = 0;

const unsigned long SONAR_INTERVAL = 100;
const unsigned long OLED_INTERVAL = 200;
const unsigned long ENC_CALC_INTERVAL = 500;

/* ==================== 电机控制 ==================== */

void carStop() {
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN3, LOW);
  digitalWrite(RIGHT_IN4, LOW);
  analogWrite(LEFT_ENA, 0);
  analogWrite(RIGHT_ENB, 0);
}

// 实测前进/后退与指令相反，F/B 实现已对调（自动避障前进也走这里）
void carForward(int speed) {
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, HIGH);
  digitalWrite(RIGHT_IN3, LOW);
  digitalWrite(RIGHT_IN4, HIGH);
  analogWrite(LEFT_ENA, speed);
  analogWrite(RIGHT_ENB, speed);
}

void carBackward(int speed) {
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN3, HIGH);
  digitalWrite(RIGHT_IN4, LOW);
  analogWrite(LEFT_ENA, speed);
  analogWrite(RIGHT_ENB, speed);
}

// 实测整车转向与指令相反，左右转实现已对调（自动避障转向也走这里）
void carTurnLeft(int speed) {
  digitalWrite(LEFT_IN1, HIGH);
  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN3, LOW);
  digitalWrite(RIGHT_IN4, HIGH);
  analogWrite(LEFT_ENA, speed);
  analogWrite(RIGHT_ENB, speed);
}

void carTurnRight(int speed) {
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, HIGH);
  digitalWrite(RIGHT_IN3, HIGH);
  digitalWrite(RIGHT_IN4, LOW);
  analogWrite(LEFT_ENA, speed);
  analogWrite(RIGHT_ENB, speed);
}

/* ==================== 状态 LED ==================== */

void updateLed() {
  static unsigned long lastBlinkMs = 0;
  static bool ledOn = false;

  if (currentMode == MODE_MANUAL) {
    // 手动模式：LED 常灭，收到蓝牙命令时短闪一下作为反馈
    digitalWrite(STATUS_LED, (millis() - lastCmdMs < 100) ? HIGH : LOW);
    return;
  }

  if (autoState == AUTO_OBSTACLE_STOP || autoState == AUTO_SCAN_LEFT ||
      autoState == AUTO_SCAN_RIGHT || autoState == AUTO_BACK_UP) {
    if (millis() - lastBlinkMs > 100) {
      ledOn = !ledOn;
      digitalWrite(STATUS_LED, ledOn);
      lastBlinkMs = millis();
    }
  } else {
    digitalWrite(STATUS_LED, HIGH);
  }
}

/* ==================== 舵机控制 ==================== */

void servoCenter() {
  scanServo.write(SERVO_CENTER);
}
void servoLeft() {
  scanServo.write(SERVO_LEFT);
}
void servoRight() {
  scanServo.write(SERVO_RIGHT);
}

/* ==================== 超声波测一次 ==================== */

int readSonarCm() {
  // 手动触发一次测距：NewPing 强占 Timer2 比较中断（__vector_7），与码盘冲突，故不用库
  digitalWrite(SONAR_TRIG, LOW);
  delayMicroseconds(4);
  digitalWrite(SONAR_TRIG, HIGH);
  delayMicroseconds(10);  // ≥10µs 触发脉冲
  digitalWrite(SONAR_TRIG, LOW);
  // 回波高电平时长 µs；按 SONAR_MAX_CM 推导超时(58µs/cm+余量)无回波返回 0 → 映射 999（极远/无障碍），
  // 避障用 < 阈值比较，999 永远不会误触发
  unsigned long us = pulseIn(SONAR_ECHO, HIGH, SONAR_MAX_CM * 58UL + 2000UL);
  return (us == 0) ? 999 : (int)(us / 58);
}

/* ==================== 码盘计数 & 测速 ==================== */

// Timer2 每 1ms 轮询一次 A1/A2 电平，只数上升沿，即使信号抖动也不会重复计数
// 最高可测 500 脉冲/秒（1ms 采样需脉冲宽度>2ms），远超电机实际转速
ISR(TIMER2_COMPA_vect) {
  uint8_t now = PINC & (_BV(PORTC1) | _BV(PORTC2));  // A1=PC1, A2=PC2
  uint8_t rising = now & (uint8_t)~encLastState;
  if (rising & _BV(PORTC1)) encLeftTicks++;
  if (rising & _BV(PORTC2)) encRightTicks++;
  encLastState = now;
}

void updateEncSpeed(unsigned long now) {
  if (now - lastEncCalcMs < ENC_CALC_INTERVAL) return;
  noInterrupts();  // 16 位计数器在 AVR 上读写非原子，关中断取走并清零
  unsigned int lt = encLeftTicks;  encLeftTicks = 0;
  unsigned int rt = encRightTicks; encRightTicks = 0;
  interrupts();
  float dt = (now - lastEncCalcMs) / 1000.0f;
  // 脉冲数 / 时间窗 / 每圈脉冲数 × 轮周长 = cm/s
  leftSpeedCms  = lt / dt / ENC_PPR * WHEEL_CIRC_CM;
  rightSpeedCms = rt / dt / ENC_PPR * WHEEL_CIRC_CM;
  if (lt > 3 || rt > 3) encSeen = true;  // 半秒内 >3 脉冲说明码盘确实在线
  lastEncCalcMs = now;
}

/* ==================== OLED 显示 ==================== */

void updateOLED() {
  if (!oledOk) return;  // OLED 不在线：直接跳过，主循环其余功能照常
  // 只重写有变化的行：不清整屏、无闪烁，I2C 流量也小
  static bool lastIsAuto = false;
  static int lastFront = -1;  // 各 static 初值保证首帧全画
  static char lastDir = 0;
  static int lastDirVal = -1;
  static char lastMotion = 0;
  static int lastSpeed = -1;
  static int lastLSpd = -1;
  static int lastRSpd = -1;
  static unsigned int lastCmdSeq = 0xFFFF;  // 初值取反保证首帧行 0 必画（cmdSeq 真实初值是 0）

  bool isAuto = (currentMode == MODE_AUTO);

  // 行 0：模式 + 最近蓝牙指令（无指令显示 -）
  if (isAuto != lastIsAuto || cmdSeq != lastCmdSeq) {
    display.setCursor(0, 0);
    display.print(isAuto ? "AUTO  " : "MANUAL");
    display.print(" CMD:");
    display.print(bluetoothCmd ? bluetoothCmd : '-');
    display.print("  ");
    lastCmdSeq = cmdSeq;
  }

  // 行 2：正前距离 + 舵机指向（括号内为该方向距离）
  char dir = 'C';
  int dirVal = frontDistance;
  if (isAuto && autoState == AUTO_SCAN_LEFT)  { dir = 'L'; dirVal = leftDistance; }
  if (isAuto && autoState == AUTO_SCAN_RIGHT) { dir = 'R'; dirVal = rightDistance; }
  if (frontDistance != lastFront || dir != lastDir || dirVal != lastDirVal || isAuto != lastIsAuto) {
    char buf[22];
    snprintf(buf, sizeof(buf), "D:%dcm Servo:%c(%d)", frontDistance, dir, dirVal);  // 最长 20 字符
    display.setCursor(0, 2);
    display.print(buf);
    for (int i = strlen(buf); i < 20; i++) display.print(" ");  // 补空格擦除旧行残留
    lastFront = frontDistance;
    lastDir = dir;
    lastDirVal = dirVal;
  }

  // 行 4：设定速度 PWM + 码盘实测车速（左/右轮 cm/s）
  int lSpd = (int)(leftSpeedCms + 0.5f);
  int rSpd = (int)(rightSpeedCms + 0.5f);
  if (currentSpeed != lastSpeed || lSpd != lastLSpd || rSpd != lastRSpd) {
    char buf[22];
    snprintf(buf, sizeof(buf), "SPD:%d L:%d R:%d", currentSpeed, lSpd, rSpd);
    display.setCursor(0, 4);
    display.print(buf);
    for (int i = strlen(buf); i < 20; i++) display.print(" ");
    lastSpeed = currentSpeed;
    lastLSpd = lSpd;
    lastRSpd = rSpd;
  }

  // 行 6：运动状态（定宽 7 字符，直接覆盖不残留）
  char motion;
  if (isAuto) {
    if (autoState == AUTO_FORWARD && frontDistance > OBSTACLE_THRESHOLD) motion = 'F';
    else if (autoState == AUTO_TURN_LEFT)  motion = 'L';
    else if (autoState == AUTO_TURN_RIGHT) motion = 'R';
    else if (autoState == AUTO_BACK_UP)    motion = 'B';
    else motion = 'S';
  } else {
    motion = manualBlocked ? '!' : lastManualCmd;  // '!'=前进被障碍拦停
  }
  if (motion != lastMotion || isAuto != lastIsAuto) {
    display.setCursor(0, 6);
    switch (motion) {
      case 'F': display.print(" FWD >>"); break;
      case 'B': display.print("<< BWD "); break;
      case 'L': display.print("<< LFT "); break;
      case 'R': display.print("RGT >> "); break;
      case '!': display.print("BLOCK! "); break;
      default:  display.print(" STOP  "); break;
    }
    lastMotion = motion;
  }

  lastIsAuto = isAuto;
}

/* ==================== 自动避障状态机 ==================== */
/* 流程：FORWARD(前行，50cm 内提前减速) --前方<25cm--> OBSTACLE_STOP(停车0.3s)
        ├─ 障碍已消失 --> 直接回 FORWARD（省一轮扫描）
        └─ 仍在 --> SCAN_LEFT --> SCAN_RIGHT（各0.3s，左右各测一次）
              ├─ 两侧都<30cm = 死胡同 --> BACK_UP(倒车) --> 重新扫描
              └─ 转向空旷侧 TURN_LEFT / TURN_RIGHT --> FORWARD
   转向/倒车时长按当前速度缩放，保持转角大致恒定；
   卡住兜底：码盘在线且持续给油车轮不转（低矮障碍超声波看不到）→ 视同遇障重新决策 */

// 自动模式统一运行速度：速度被调到 0/死区时用最低可运转速度兜底
int runSpeed() {
  return (currentSpeed < MIN_RUN_PWM) ? MIN_RUN_PWM : currentSpeed;
}

// 转向/倒车时长按当前速度缩放：低速转得慢就多转一会儿，高速少转，转角恒定
unsigned long scaledMs(int baseMs) {
  unsigned long ms = (unsigned long)baseMs * 180UL / (unsigned long)runSpeed();
  return constrain(ms, 400UL, 2200UL);
}

void runAutoStateMachine() {
  unsigned long now = millis();

  // 卡住检测计时：仅 FORWARD 状态累计，车轮恢复转动立刻清零
  static unsigned long stallStartMs = 0;
  bool wheelsTurning = (leftSpeedCms > 2.0f || rightSpeedCms > 2.0f);
  if (autoState != AUTO_FORWARD || wheelsTurning || !encSeen) stallStartMs = now;

  switch (autoState) {

    case AUTO_FORWARD: {
      int spd = runSpeed();
      if (frontDistance < APPROACH_DIST) {           // 接近减速：降 1/3，留足刹停余量
        int slow = spd * 2 / 3;
        if (slow < MIN_RUN_PWM + 10) slow = MIN_RUN_PWM + 10;
        spd = slow;
      }
      carForward(spd);
      if (encSeen && now - stallStartMs > STALL_TIME_MS) {
        // 电机在转但车不动：顶到低矮障碍，走一遍避障决策脱困
        autoState = AUTO_OBSTACLE_STOP;
        stateStartMs = now;
        Serial.println("[AUTO] Wheels stalled, re-decide");
      } else if (frontDistance < OBSTACLE_THRESHOLD) {  // readSonarCm 失败时返回 999，不会误触发
        autoState = AUTO_OBSTACLE_STOP;
        stateStartMs = now;
        Serial.print("[AUTO] Obstacle! d=");
        Serial.println(frontDistance);
      }
      break;
    }

    case AUTO_OBSTACLE_STOP:
      carStop();
      servoCenter();
      if (now - stateStartMs > 300) {
        if (frontDistance >= OBSTACLE_THRESHOLD + 5) {
          autoState = AUTO_FORWARD;  // 障碍已消失（行人走过等），直接续行
          lastSonarMs = now - SONAR_INTERVAL;
          Serial.println("[AUTO] Clear again, resume FWD");
        } else {
          autoState = AUTO_SCAN_LEFT;
          stateStartMs = now;
          servoLeft();
          Serial.println("[AUTO] Scanning LEFT...");
        }
      }
      break;

    case AUTO_SCAN_LEFT:
      if (now - stateStartMs > SCAN_TIME_MS) {
        leftDistance = readSonarCm();
        if (leftDistance == 999) leftDistance = readSonarCm();  // 近距无回波偶发假999，补测一次
        Serial.print("[AUTO] Left: ");
        Serial.println(leftDistance);
        autoState = AUTO_SCAN_RIGHT;
        stateStartMs = now;
        servoRight();
      }
      break;

    case AUTO_SCAN_RIGHT:
      if (now - stateStartMs > SCAN_TIME_MS) {
        rightDistance = readSonarCm();
        if (rightDistance == 999) rightDistance = readSonarCm();  // 近距无回波偶发假999，补测一次
        Serial.print("[AUTO] Right: ");
        Serial.println(rightDistance);
        servoCenter();

        // 无回波方向距离=999（视为最空旷），>= 比较自然优先选择空侧
        if (leftDistance < SIDE_CLEAR_MIN && rightDistance < SIDE_CLEAR_MIN) {
          autoState = AUTO_BACK_UP;  // 两侧都堵死：倒车拉开距离再重新扫
          stateStartMs = now;
          Serial.println("[AUTO] Dead end, backing up");
        } else if (leftDistance >= rightDistance) {
          autoState = AUTO_TURN_LEFT;
          stateStartMs = now;
          Serial.println("[AUTO] Choose LEFT");
        } else {
          autoState = AUTO_TURN_RIGHT;
          stateStartMs = now;
          Serial.println("[AUTO] Choose RIGHT");
        }
      }
      break;

    case AUTO_BACK_UP:
      carBackward(runSpeed());
      if (now - stateStartMs > scaledMs(BACKUP_TIME_MS)) {
        autoState = AUTO_SCAN_LEFT;  // 倒完直接重新扫，不回 FORWARD（正前大概率仍是墙）
        stateStartMs = now;
        servoLeft();
        Serial.println("[AUTO] Scanning LEFT...");
      }
      break;

    case AUTO_TURN_LEFT:
      carTurnLeft(runSpeed());
      if (now - stateStartMs > scaledMs(TURN_TIME_MS)) {
        autoState = AUTO_FORWARD;
        stateStartMs = now;
        lastSonarMs = now - SONAR_INTERVAL;  // 立即重测正前方
      }
      break;

    case AUTO_TURN_RIGHT:
      carTurnRight(runSpeed());
      if (now - stateStartMs > scaledMs(TURN_TIME_MS)) {
        autoState = AUTO_FORWARD;
        stateStartMs = now;
        lastSonarMs = now - SONAR_INTERVAL;  // 立即重测正前方
      }
      break;
  }
}

/* ==================== 模式切换 ==================== */

void switchToManual() {
  if (currentMode == MODE_MANUAL) return;
  currentMode = MODE_MANUAL;
  carStop();
  servoCenter();
  Serial.println("[MODE] -> MANUAL");
}

void switchToAuto() {
  if (currentMode == MODE_AUTO) return;
  currentMode = MODE_AUTO;
  autoState = AUTO_FORWARD;
  stateStartMs = millis();
  manualBlocked = false;  // 离开手动模式，清除拦停标志
  servoCenter();
  Serial.println("[MODE] -> AUTO");
}

/* ==================== 蓝牙命令解析 ==================== */
/* 单字符协议：每次循环最多读 1 字节，\n \r 空格直接丢弃。
   9600bps 下字符间隔约 1ms，SoftwareSerial 64 字节缓冲可吸收突发 */

void applyManualMotion() {
  switch (lastManualCmd) {
    case 'F': carForward(currentSpeed);  break;
    case 'B': carBackward(currentSpeed); break;
    case 'L': carTurnLeft(currentSpeed); break;
    case 'R': carTurnRight(currentSpeed); break;
    default:  carStop();                 break;
  }
}

void handleBluetooth() {
  if (!bluetoothSerial->available()) return;

  bluetoothCmd = bluetoothSerial->read();

  if (bluetoothCmd == '\n' || bluetoothCmd == '\r' || bluetoothCmd == ' ') return;

  lastCmdMs = millis();  // 每收到一个有效字符，LED 短闪反馈
  cmdSeq++;

  Serial.print("[BLE] ");
  Serial.print(bluetoothCmd);
  Serial.print(" -> ");

  switch (bluetoothCmd) {
    case 'F':
    case 'f':
      switchToManual();
      lastManualCmd = 'F';
      manualStepEndMs = millis() + MANUAL_STEP_MS;  // 点动：到时自停，重复收到则续时
      applyManualMotion();
      Serial.println("FWD");
      break;
    case 'B':
    case 'b':
      switchToManual();
      lastManualCmd = 'B';
      manualStepEndMs = millis() + MANUAL_STEP_MS;
      applyManualMotion();
      Serial.println("BWD");
      break;
    case 'L':
    case 'l':
      switchToManual();
      lastManualCmd = 'L';
      manualStepEndMs = millis() + MANUAL_STEP_MS;
      applyManualMotion();
      Serial.println("LFT");
      break;
    case 'R':
    case 'r':
      switchToManual();
      lastManualCmd = 'R';
      manualStepEndMs = millis() + MANUAL_STEP_MS;
      applyManualMotion();
      Serial.println("RGT");
      break;
    case 'S':
    case 's':
      switchToManual();
      lastManualCmd = 'S';
      applyManualMotion();
      Serial.println("STP");
      break;
    case 'A':
    case 'a':
      switchToAuto();
      break;
    case 'T':
    case 't':
      // 舵机自检：强制手动停车后左右扫一遍，判断舵机/接线是否正常
      switchToManual();
      Serial.println("SERVO TEST: L-C-R-C");
      scanServo.write(SERVO_LEFT);   delay(600);
      scanServo.write(SERVO_CENTER); delay(600);
      scanServo.write(SERVO_RIGHT);  delay(600);
      servoCenter();
      Serial.println("SERVO TEST done");
      break;
    case '+':
    case 'U':
    case 'u':
      currentSpeed += SPEED_STEP;
      if (currentSpeed > 255) currentSpeed = 255;
      if (currentSpeed < MIN_RUN_PWM) currentSpeed = MIN_RUN_PWM;  // 从停转加速直接进入可运转区间
      if (currentMode == MODE_MANUAL) applyManualMotion();  // 立即生效
      Serial.print("SPD+ ");
      Serial.println(currentSpeed);
      break;
    case '-':
    case 'D':
    case 'd':
      currentSpeed -= SPEED_STEP;
      if (currentSpeed < MIN_RUN_PWM) currentSpeed = 0;  // 减进死区直接停，避免只哼不转
      if (currentMode == MODE_MANUAL) applyManualMotion();  // 立即生效
      Serial.print("SPD- ");
      Serial.println(currentSpeed);
      break;
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
      {
        int idx = bluetoothCmd - '0';
        currentSpeed = map(idx, 1, 9, MIN_RUN_PWM, 255);  // 1 档=最低可运转速度，避开低速死区
        if (currentSpeed > 255) currentSpeed = 255;
        if (currentMode == MODE_MANUAL) applyManualMotion();  // 立即生效
        Serial.print("SPD ");
        Serial.println(currentSpeed);
        break;
      }
    default:
      Serial.println("(ignored)");
  }
}

/* ==================== 手动点动定时 ==================== */
/* 每条 F/B/L/R 只执行固定时长后自动停车：
   - APP 单击发一次 = 走一小段（寸动）
   - APP 按住重复发 = 定时不断续期，相当于按住走
   - 兼带断连失控保护：蓝牙断链最多失控 MANUAL_STEP_MS */

void updateManualStep() {
  if (currentMode != MODE_MANUAL || lastManualCmd == 'S') return;
  if ((long)(millis() - manualStepEndMs) < 0) return;  // 未到点动截止时刻
  lastManualCmd = 'S';
  carStop();
  Serial.println("[MAN] Step done, stop");
}

/* ==================== 手动前向防碰撞 ==================== */
/* 手动模式前进时前方有障碍自动停车；障碍清除后自动恢复前进。
   只拦前进，不拦后退/转向，否则被挡住时无法脱困。 */

void updateManualGuard() {
  if (currentMode != MODE_MANUAL || lastManualCmd != 'F') {
    manualBlocked = false;
    return;
  }
  if (frontDistance < OBSTACLE_THRESHOLD) {
    if (!manualBlocked) {
      manualBlocked = true;
      Serial.println("[GUARD] Front obstacle, auto stop");
    }
    carStop();  // 持续保持停车，防止被重复下发的 F 覆盖
  } else if (manualBlocked) {
    manualBlocked = false;
    applyManualMotion();  // 障碍清除，恢复前进
    Serial.println("[GUARD] Clear, resume FWD");
  }
}

/* ==================== Arduino 主函数 ==================== */

int freeRam() {  // 诊断用：查看剩余 RAM（Uno 共 2048 字节）
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval ? (int)__brkval : (int)&__heap_start);
}

void setup() {
  Serial.begin(9600);

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);
  Serial.print("[1] LED on, free RAM: ");
  Serial.println(freeRam());

  // 先探测 0x3C 是否应答：OLED 未接/故障时跳过显示初始化（降级运行），
  // 避免对不在线设备反复读写导致启动卡死，车辆仍可正常遥控
  Wire.begin();
  Wire.beginTransmission(0x3C);
  oledOk = (Wire.endTransmission() == 0);
  if (oledOk) {
    display.begin(&Adafruit128x64, 0x3C);
    display.setFont(System5x7);
    Wire.setClock(400000L);  // I2C 提速到 400kHz，刷屏更快
    Serial.println("[2] OLED begin done");
    display.clear();
    display.setCursor(0, 0);
    display.println("Hello World!");
    Serial.println("[3] OLED show done");
    delay(2000);  // 开机画面停留
  } else {
    Serial.println("[3] OLED offline, run without display");
  }
  Serial.println("[4] OLED stage done");

  bluetoothSerial = new SoftwareSerial(8, 9);
  bluetoothSerial->begin(9600);
  Serial.println("[5] BLE init done");

  // 超声波引脚：Trig 推挽输出，Echo 输入（替代 NewPing 的内部配置）
  pinMode(SONAR_TRIG, OUTPUT);
  digitalWrite(SONAR_TRIG, LOW);
  pinMode(SONAR_ECHO, INPUT);

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN3, OUTPUT);
  pinMode(RIGHT_IN4, OUTPUT);

  // 码盘模块：用内部上拉，未接模块时引脚不浮空、读数稳定为 0
  pinMode(ENC_LEFT_PIN, INPUT_PULLUP);
  pinMode(ENC_RIGHT_PIN, INPUT_PULLUP);
  encLastState = PINC & (_BV(PORTC1) | _BV(PORTC2));
  // Timer2 CTC 模式 1kHz：定时轮询码盘电平（Timer0=millis，Timer1=舵机，Timer2 空闲）
  TCCR2A = _BV(WGM21);
  TCCR2B = _BV(CS22);   // 16MHz / 64 分频
  OCR2A  = 249;         // 250 计数 → 1kHz 中断
  TIMSK2 = _BV(OCIE2A);
  lastEncCalcMs = millis();  // 测速窗口从现在起算，避免开机时长计入首帧

  carStop();

  scanServo.attach(SERVO_PIN);
  servoCenter();

  frontDistance = readSonarCm();
  lastSonarMs = millis();

  Serial.println("=== Bluetooth Car Ready ===");
  Serial.println("F/B/L/R/S=Manual  A=Auto  +/-=Speed  1-9=Gear");
}

void loop() {
  unsigned long now = millis();

  // 舵机居中时测距才是"正前方"；扫描/转向期间跳过，
  // 避免侧面读数污染 frontDistance，也避免与扫描测距背靠背 ping 互相干扰
  bool servoCentered = (currentMode == MODE_MANUAL) ||
                       (autoState == AUTO_FORWARD) ||
                       (autoState == AUTO_OBSTACLE_STOP);
  if (servoCentered && now - lastSonarMs >= SONAR_INTERVAL) {
    frontDistance = readSonarCm();
    lastSonarMs = now;
  }

  updateEncSpeed(now);

  handleBluetooth();

  if (currentMode == MODE_AUTO) {
    runAutoStateMachine();
  } else {
    updateManualStep();   // 点动计时到点自动停车
    updateManualGuard();  // 手动模式：前方障碍自动停车
  }

  updateLed();

  if (now - lastOledMs >= OLED_INTERVAL) {
    updateOLED();
    lastOledMs = now;
  }
}
