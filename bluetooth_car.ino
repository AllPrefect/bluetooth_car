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
  AUTO_VERIFY,     // 停车全幅扫描：确认方向无障碍后才允许前进
  AUTO_FORWARD,
  AUTO_TURN_LEFT,
  AUTO_TURN_RIGHT,
  AUTO_BACK_UP,    // 死胡同：两侧都堵，倒车脱困后重新扫描
  AUTO_TURN_AROUND, // 连续倒车仍出不去：原地掉头换方向
  AUTO_FAULT       // 超声波疑似失效（连续多轮全无回波 + 轮子卡住）：停车报警，等按 A 重试
};

CarMode currentMode = MODE_MANUAL;  // 默认手动待命，电机不启动；发 A 进自动，发 F/B/L/R 手动行驶
AutoState autoState = AUTO_FORWARD;

/* ==================== 避障参数 ==================== */

const int OBSTACLE_THRESHOLD = 25;   // 正前方刹停距离 cm
const int APPROACH_DIST = 50;        // 接近减速区：进入该距离提前降速，避免全速怼到跟前急刹
const int BRAKE_ZONE_DIST = 35;      // 贴身减速区：再降一半，减小撞墙动能
const int SIDE_CLEAR_MIN = 30;       // 左右至少这么空才算可通行
const int DEAD_END_SIDE_CM = 50;     // 死胡同判定：正前堵住且两侧都不足 50cm，转向也进不去 → 倒车
const int TURN_TIME_MS = 800;        // 基准转向时长（对应基准速度 180，实际按速度缩放）
const int BACKUP_TIME_MS = 900;      // 死胡同倒车时长（同样按速度缩放，给足脱困距离）
const int MAX_BACKUP_TIMES = 3;      // 连续倒车 3 次仍出不去就原地掉头
int backUpCount = 0;                 // 连续倒车次数，前进成功即清零
const int STALL_TIME_MS = 1200;      // 给油但车轮不转的判定时长（卡住逃逸，需码盘在线）

int frontDistance = 0;
int leftDistance = 0;
int rightDistance = 0;

/* 起步前确认扫描（AUTO_VERIFY）：
   车停稳，舵机匀速连续摆过 -75°~+75°，扫到格点就测一次，全程不停顿；
   扫描用短超时测距(量程 120cm，阻塞约 7.6ms)，舵机转速不被打断，观感是顺滑的匀速摆动。
   扫完整幅再决策：正前 ±30° 最近距离 >= 30cm 才允许前进，否则按左右结果转向/倒车。 */
const int8_t SWEEP_MAX_DEG = 75;             // 扫描半幅（留余量不顶 0/180 机械限位）
const int8_t SWEEP_STEP_DEG = 15;            // 采样格点
const int SWEEP_POINTS = 2 * SWEEP_MAX_DEG / SWEEP_STEP_DEG + 1;
const int SWEEP_CENTER_IDX = SWEEP_MAX_DEG / SWEEP_STEP_DEG;
const unsigned long MICRO_STEP_MS = 5;         // 匀速扫描：每 5ms 走 1°（≈200°/s）
const unsigned long FORWARD_SONAR_MS = 60;     // 行驶中正前测距间隔
const unsigned long FORWARD_MIN_MS = 600;      // 每次确认后最短前进时长
const int CONFIRM_CLEAR_CM = 30;               // 正前确认阈值：该距离以上才允许前进
int8_t scanOffset = 0;        // 当前波束角度（相对中位，正=左），供显示
int8_t scanPos = 0;           // 摆动位置（同时就是写入舵机的角度）
int8_t scanDir = -1;          // 摆动方向 -1=向右 +1=向左
bool scanReturning = false;   // 扫到左端后匀速回到中位再交棒，避免最后一步甩 75°
int8_t scanLastIdx = -99;     // 上次采样的格点，避免同一格重复 ping
int8_t lastServoOffset = -128;// 上次写入的舵机角度，相同值不再重复写
unsigned long scanTickMs = 0;
int scanFrontMin = 0;         // 本次扫描 正前±30° 内最近距离（车头通路，用于放行判定）
int scanCenterMin = 0;        // 本次扫描 正中读数（用于决定本段能走多远）
int scanLeftMin = 0;          // 本次扫描 左半区最近距离
int scanRightMin = 0;         // 本次扫描 右半区最近距离
int sweepMap[SWEEP_POINTS];   // 本次扫描角度图：idx 0=最右 … 正中 … 最大=最左
uint8_t noEchoScans = 0;      // 连续多少轮扫描整幅都无回波（全 999）：多轮全空 + 卡住 = 传感器失效
unsigned long forwardDeadlineMs = 0;  // 本段前进的截止时刻（到点停车重扫）



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
unsigned long stallStartMs = 0;   // 卡住检测起点（进 FORWARD 时重置，避免沿用扫描期间的时间戳）

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

// 前进中差速绕行：veerRight=true 右轮减速向右绕；慢侧不低于死区，避免变原地转
void carForwardVeer(int spd, bool veerRight) {
  int slow = spd / 2;
  if (slow < MIN_RUN_PWM) slow = MIN_RUN_PWM;
  digitalWrite(LEFT_IN1, LOW);
  digitalWrite(LEFT_IN2, HIGH);
  digitalWrite(RIGHT_IN3, LOW);
  digitalWrite(RIGHT_IN4, HIGH);
  analogWrite(LEFT_ENA, veerRight ? spd : slow);
  analogWrite(RIGHT_ENB, veerRight ? slow : spd);
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

  if (autoState == AUTO_FAULT) {  // 传感器失效报警：快闪
    if (millis() - lastBlinkMs > 60) {
      ledOn = !ledOn;
      digitalWrite(STATUS_LED, ledOn);
      lastBlinkMs = millis();
    }
  } else if (autoState == AUTO_VERIFY || autoState == AUTO_BACK_UP || autoState == AUTO_TURN_AROUND) {
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
  lastServoOffset = 0;
}

// 只在角度变化时写舵机：Servo.write 内部有 map/除法，每拍重复写没有意义
void writeScanServo(int8_t offset) {
  if (offset == lastServoOffset) return;
  lastServoOffset = offset;
  scanServo.write(constrain(SERVO_CENTER + offset, 5, 175));  // 钳制在物理行程内
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

// 扫描专用快速测距：量程 120cm，pulseIn 超时约 7.6ms，阻塞远小于一步 5ms 的推进节拍，
// 匀速摆动不被打断；120cm 外一律 999（等同"空旷"）。
// 量程不能太小：60cm 时把 60~100cm 外的侧墙误判成空旷，会转过去再撞墙
int readSonarCmFast() {
  digitalWrite(SONAR_TRIG, LOW);
  delayMicroseconds(4);
  digitalWrite(SONAR_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRIG, LOW);
  unsigned long us = pulseIn(SONAR_ECHO, HIGH, 120 * 58UL + 600UL);
  return (us == 0) ? 999 : (int)(us / 58);
}

// 测 3 次取中值：斜面/软障碍回波弱时单次 ping 易返回 999（被当成空旷），
// 单样本会直接打穿 25cm 刹停线；中值滤波后需 2/3 次都无回波才误判，概率大幅下降
int readSonarFiltered() {
  int a = readSonarCmFast();
  delay(5);
  int b = readSonarCmFast();
  delay(5);
  int c = readSonarCmFast();
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  if (a > b) { int t = a; a = b; b = t; }
  return b;
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
  if (isAuto && autoState == AUTO_VERIFY) {  // 扫描中：显示当前扫描方向与读数
    dir = (scanOffset > 0) ? 'L' : ((scanOffset < 0) ? 'R' : 'C');
    dirVal = sweepMap[(scanOffset + SWEEP_MAX_DEG + SWEEP_STEP_DEG / 2) / SWEEP_STEP_DEG];
  }
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
    if (autoState == AUTO_VERIFY) motion = 'V';
    else if (autoState == AUTO_FORWARD && frontDistance > OBSTACLE_THRESHOLD) motion = 'F';
    else if (autoState == AUTO_TURN_LEFT)  motion = 'L';
    else if (autoState == AUTO_TURN_RIGHT) motion = 'R';
    else if (autoState == AUTO_BACK_UP)    motion = 'B';
    else if (autoState == AUTO_TURN_AROUND) motion = 'U';
    else if (autoState == AUTO_FAULT) motion = 'X';
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
      case 'V': display.print(" SCAN  "); break;
      case 'U': display.print(" U-TURN"); break;
      case 'X': display.print(" FAULT "); break;
      default:  display.print(" STOP  "); break;
    }
    lastMotion = motion;
  }

  lastIsAuto = isAuto;
}

/* ==================== 自动避障状态机 ==================== */
/* 扫描-确认-前进 循环（先确认方向通畅再走，不再边走边扫）：
   VERIFY  ：停车，舵机从 -75° 到 +75° 匀速连续扫一轮（每 15° 采样一次，不停顿），
             扫完才决策 —— 正前 ±30°（车头通路）最近距离 ≥30cm 才允许前进
    ├─ 前方通畅 --> FORWARD
    ├─ 两侧都不足 50cm（转过去也是墙）= 死胡同 --> BACK_UP（倒完重新 VERIFY；
    │     连续倒 3 次仍出不去则 U-TURN 掉头）
    └─ 单侧通畅 --> TURN_LEFT / TURN_RIGHT（转完重新 VERIFY）
   FORWARD ：只盯正前（60ms 单发+无回波补测），按确认到的距离走一段：
             前方<25cm 或走完确认距离（单段上限 50cm/2s）→ 停车重新 VERIFY
             50cm 降 1/3、35cm 半速；侧翼按上一轮扫描结果差速绕行
   转向/倒车时长按当前速度缩放，保持转角大致恒定；
   卡住兜底：码盘在线且持续给油车轮不转 → 重新 VERIFY（贴身障碍超声波常无回波） */

// 自动模式统一运行速度：速度被调到 0/死区时用最低可运转速度兜底
int runSpeed() {
  return (currentSpeed < MIN_RUN_PWM) ? MIN_RUN_PWM : currentSpeed;
}

// 转向/倒车时长按当前速度缩放：低速转得慢就多转一会儿，高速少转，转角恒定
unsigned long scaledMs(int baseMs) {
  unsigned long ms = (unsigned long)baseMs * 180UL / (unsigned long)runSpeed();
  return constrain(ms, 400UL, 2200UL);
}

/* ==================== 确认扫描 & 行驶测距 ==================== */

// 进入确认扫描：立即停车，舵机摆到最右端，开始逐格扫描
void startVerifySweep() {
  carStop();
  autoState = AUTO_VERIFY;
  stateStartMs = millis();
  scanPos = 0;                     // 从中位起摆，先扫右侧再一路扫到左侧
  scanDir = -1;
  scanReturning = false;
  scanLastIdx = -99;
  scanOffset = scanPos;
  scanTickMs = millis();
  scanFrontMin = 999;
  scanCenterMin = 999;
  scanLeftMin = 999;
  scanRightMin = 999;
  for (int i = 0; i < SWEEP_POINTS; i++) sweepMap[i] = 999;
  writeScanServo(scanOffset);
  Serial.println(F("[AUTO] Scan full range..."));
}

// 扫描完成：按扫描结果决定下一步（前进 / 转向 / 倒车）
void finishVerifySweep() {
  frontDistance = scanCenterMin;   // 前进段测的是正前方，这里保持同一口径
  leftDistance = scanLeftMin;
  rightDistance = scanRightMin;
  servoCenter();
  scanOffset = 0;
  Serial.print(F("[AUTO] Scan done front="));
  Serial.print(scanFrontMin);
  Serial.print(F(" L="));
  Serial.print(scanLeftMin);
  Serial.print(F(" R="));
  Serial.println(scanRightMin);

  // 整幅全部 999：可能真空旷，也可能探头失效。单轮无法区分，只累计连续轮数，
  // 等"全空 + 轮子转不动"同时出现时才判定失效（见 FORWARD 卡住分支）
  bool allNoEcho = true;
  for (int i = 0; i < SWEEP_POINTS; i++)
    if (sweepMap[i] != 999) { allNoEcho = false; break; }
  if (allNoEcho) {
    if (noEchoScans < 250) noEchoScans++;
  } else {
    noEchoScans = 0;
  }

  if (scanFrontMin >= CONFIRM_CLEAR_CM) {  // 前方确认畅通，才允许前进
    autoState = AUTO_FORWARD;
    stateStartMs = millis();
    stallStartMs = millis();  // 卡住检测从这里重新计时，不沿用扫描期间的时间戳
    backUpCount = 0;  // 脱困成功，倒车计数清零
    // 按正中方向确认到的距离定本段前进时长：走到还剩约 35cm 就停车重扫
    int travelCm = scanCenterMin - 35;
    if (travelCm < 10) travelCm = 10;
    if (travelCm > 50) travelCm = 50;   // 单段最长 50cm：更频繁停车重扫，减少侧前方盲区
    unsigned long ms = (unsigned long)travelCm * 1000UL / (unsigned long)(runSpeed() / 6);
    forwardDeadlineMs = millis() + constrain(ms, FORWARD_MIN_MS, 2000UL);
  } else if (scanLeftMin < DEAD_END_SIDE_CM && scanRightMin < DEAD_END_SIDE_CM) {
    // 正前堵死，两侧也不足 50cm（转过去也是墙）= 死胡同
    if (backUpCount >= MAX_BACKUP_TIMES) {  // 倒了几次还是出不去，不要无限倒
      autoState = AUTO_TURN_AROUND;
      stateStartMs = millis();
      backUpCount = 0;
      Serial.println(F("[AUTO] Dead end persists, U-turn"));
    } else {
      autoState = AUTO_BACK_UP;  // 先倒车拉开距离
      stateStartMs = millis();
      backUpCount++;
      Serial.print(F("[AUTO] Dead end, backing up #"));
      Serial.println(backUpCount);
    }
  } else if (scanLeftMin >= scanRightMin) {
    autoState = AUTO_TURN_LEFT;
    stateStartMs = millis();
    Serial.println(F("[AUTO] Choose LEFT"));
  } else {
    autoState = AUTO_TURN_RIGHT;
    stateStartMs = millis();
    Serial.println(F("[AUTO] Choose RIGHT"));
  }
}

// 匀速连续扫描：位置按经过时间推进，全程不停顿；测距在摆动途中按格点取样。
// 扫描测距用 120cm 量程版本(约7.6ms)，阻塞小于一步 5ms 的推进节拍，舵机转速不被打断。
// 扫到左端后不停下甩回中位，而是继续匀速走回 0°，再交给状态机（末段不采样）
void updateVerifySweep(unsigned long now) {
  if (now - scanTickMs >= MICRO_STEP_MS) {
    unsigned long steps = (now - scanTickMs) / MICRO_STEP_MS;
    if (steps > 10) steps = 10;   // 极端阻塞后不追补，避免瞬间甩动
    scanTickMs += steps * MICRO_STEP_MS;
    while (steps-- > 0) {
      scanPos += scanDir;
      if (scanReturning) {
        if (scanPos <= 0) {         // 回到中位，扫描结束
          scanPos = 0;
          scanOffset = 0;
          writeScanServo(0);
          finishVerifySweep();
          return;
        }
      } else if (scanPos <= -SWEEP_MAX_DEG) {
        scanPos = -SWEEP_MAX_DEG;
        scanDir = 1;                // 到右端折返
      } else if (scanPos >= SWEEP_MAX_DEG) {
        scanPos = SWEEP_MAX_DEG;
        scanDir = -1;
        scanReturning = true;       // 扫完左端，开始匀速回中
      }
    }
  }

  scanOffset = scanPos;
  writeScanServo(scanPos);

  if (scanReturning) return;       // 回中段不再采样，避免重复 ping 拖慢回流

  int idx = (scanPos + SWEEP_MAX_DEG + SWEEP_STEP_DEG / 2) / SWEEP_STEP_DEG;
  if (idx < 0) idx = 0;
  if (idx > SWEEP_POINTS - 1) idx = SWEEP_POINTS - 1;
  if (idx != scanLastIdx) {                       // 每跨过一个格点测一次
    scanLastIdx = idx;
    int d = readSonarCmFast();                    // 连续摆动中没有稳定窗口，单发取样
    sweepMap[idx] = d;
    if (idx == SWEEP_CENTER_IDX) scanCenterMin = d;
    if (idx >= SWEEP_CENTER_IDX - 2 && idx <= SWEEP_CENTER_IDX + 2) {
      // 正前 ±30° 都算"车头通路"：HC-SR04 有效锥角约 ±15°，左前/右前 20~30° 的障碍
      // 一旦车开动就出了声波锥，必须在扫描阶段拦住
      if (d < scanFrontMin) scanFrontMin = d;
      frontDistance = scanFrontMin;  // 扫描中实时更新显示
    } else if (idx < SWEEP_CENTER_IDX) {
      if (d < scanRightMin) scanRightMin = d;
    } else {
      if (d < scanLeftMin) scanLeftMin = d;
    }
  }
}

// 行驶中只盯正前：车在动，用 120cm 量程快测(~7.6ms)，避免长时间阻塞主循环；
// 单发 999 再补测一次，斜面偶发打空不会直接写进 frontDistance
void updateForwardSonar(unsigned long now) {
  if (now - lastSonarMs < FORWARD_SONAR_MS) return;
  int d = readSonarCmFast();
  if (d == 999) d = readSonarCmFast();
  frontDistance = d;
  lastSonarMs = now;
}

void runAutoStateMachine() {
  unsigned long now = millis();

  // 卡住检测计时：仅 FORWARD 状态累计，车轮恢复转动立刻清零
  bool wheelsTurning = (leftSpeedCms > 2.0f || rightSpeedCms > 2.0f);
  if (autoState != AUTO_FORWARD || wheelsTurning || !encSeen) stallStartMs = now;

  switch (autoState) {

    case AUTO_VERIFY:
      // 停车扫描由 updateVerifySweep() 推进，扫完自行切到 前进/转向/倒车
      break;

    case AUTO_FORWARD: {
      int spd = runSpeed();
      if (frontDistance < BRAKE_ZONE_DIST) spd = spd / 2;         // 贴身：半速，减小撞击动能
      else if (frontDistance < APPROACH_DIST) spd = spd * 2 / 3;  // 接近：降 1/3
      if (frontDistance < APPROACH_DIST && spd < MIN_RUN_PWM + 10) spd = MIN_RUN_PWM + 10;
      // 侧翼用上一轮扫描结果修正：单侧近就向另一侧差速绕；两侧都近=窄通道，照直走
      if (frontDistance >= APPROACH_DIST && leftDistance < SIDE_CLEAR_MIN && rightDistance >= SIDE_CLEAR_MIN) {
        carForwardVeer(spd, true);   // 左侧近 → 向右绕
      } else if (frontDistance >= APPROACH_DIST && rightDistance < SIDE_CLEAR_MIN && leftDistance >= SIDE_CLEAR_MIN) {
        carForwardVeer(spd, false);  // 右侧近 → 向左绕
      } else {
        carForward(spd);
      }
      if (encSeen && now - stallStartMs > STALL_TIME_MS) {
        if (noEchoScans >= 3) {
          // 连续三轮扫描整幅无回波，却又顶住了走不动 → 超声波失效/被遮挡，停车报警
          autoState = AUTO_FAULT;
          stateStartMs = now;
          carStop();
          Serial.println(F("[FAULT] Sonar blind + wheels stalled, stopped. Press A to retry"));
        } else {
          Serial.println(F("[AUTO] Wheels stalled, re-scan"));  // 顶住障碍但超声波测不到：重扫
          startVerifySweep();
        }
      } else if (frontDistance < OBSTACLE_THRESHOLD) {
        Serial.print(F("[AUTO] Obstacle! d="));
        Serial.println(frontDistance);
        startVerifySweep();  // 停车重新扫描确认
      } else if (now >= forwardDeadlineMs) {
        Serial.println(F("[AUTO] Segment done, re-scan"));  // 走完确认过的一段，重新扫
        startVerifySweep();
      }
      break;
    }

    case AUTO_BACK_UP:
      carBackward(runSpeed());
      if (now - stateStartMs > scaledMs(BACKUP_TIME_MS)) {
        startVerifySweep();  // 倒完重新扫，确认无障碍再走
      }
      break;

    case AUTO_TURN_LEFT:
      carTurnLeft(runSpeed());
      if (now - stateStartMs > scaledMs(TURN_TIME_MS)) {
        startVerifySweep();  // 转完先扫描确认，再决定是否前进
      }
      break;

    case AUTO_TURN_RIGHT:
      carTurnRight(runSpeed());
      if (now - stateStartMs > scaledMs(TURN_TIME_MS)) {
        startVerifySweep();  // 转完先扫描确认，再决定是否前进
      }
      break;

    case AUTO_TURN_AROUND:  // 死胡同反复脱不了困：原地掉头 180°（时长翻倍）
      carTurnLeft(runSpeed());
      if (now - stateStartMs > scaledMs(TURN_TIME_MS) * 2) {
        startVerifySweep();
      }
      break;

    case AUTO_FAULT:  // 传感器失效：保持停车，闪灯报警；按 A 重新扫描即解除
      carStop();
      break;
  }
}

/* ==================== 模式切换 ==================== */

void switchToManual() {
  if (currentMode == MODE_MANUAL) return;
  currentMode = MODE_MANUAL;
  carStop();
  servoCenter();
  Serial.println(F("[MODE] -> MANUAL"));
}

void switchToAuto() {
  if (currentMode == MODE_AUTO) {  // 已在自动模式再按 A：重新扫描一次（相当于"重试"）
    startVerifySweep();
    return;
  }
  currentMode = MODE_AUTO;
  manualBlocked = false;   // 离开手动模式，清除拦停标志
  backUpCount = 0;         // 新一次自动行驶，倒车计数从零开始
  startVerifySweep();      // 先停车扫描，确认方向通畅后才前进
  Serial.println(F("[MODE] -> AUTO"));
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

  Serial.print(F("[BLE] "));
  Serial.print(bluetoothCmd);
  Serial.print(F(" -> "));

  switch (bluetoothCmd) {
    case 'F':
    case 'f':
      switchToManual();
      lastManualCmd = 'F';
      manualStepEndMs = millis() + MANUAL_STEP_MS;  // 点动：到时自停，重复收到则续时
      applyManualMotion();
      Serial.println(F("FWD"));
      break;
    case 'B':
    case 'b':
      switchToManual();
      lastManualCmd = 'B';
      manualStepEndMs = millis() + MANUAL_STEP_MS;
      applyManualMotion();
      Serial.println(F("BWD"));
      break;
    case 'L':
    case 'l':
      switchToManual();
      lastManualCmd = 'L';
      manualStepEndMs = millis() + MANUAL_STEP_MS;
      applyManualMotion();
      Serial.println(F("LFT"));
      break;
    case 'R':
    case 'r':
      switchToManual();
      lastManualCmd = 'R';
      manualStepEndMs = millis() + MANUAL_STEP_MS;
      applyManualMotion();
      Serial.println(F("RGT"));
      break;
    case 'S':
    case 's':
      switchToManual();
      lastManualCmd = 'S';
      applyManualMotion();
      Serial.println(F("STP"));
      break;
    case 'A':
    case 'a':
      switchToAuto();
      break;
    case 'T':
    case 't':
      // 舵机自检：强制手动停车后左右扫一遍，判断舵机/接线是否正常
      switchToManual();
      Serial.println(F("SERVO TEST: L-C-R-C"));
      scanServo.write(SERVO_LEFT);   delay(600);
      scanServo.write(SERVO_CENTER); delay(600);
      scanServo.write(SERVO_RIGHT);  delay(600);
      servoCenter();
      Serial.println(F("SERVO TEST done"));
      break;
    case '+':
    case 'U':
    case 'u':
      currentSpeed += SPEED_STEP;
      if (currentSpeed > 255) currentSpeed = 255;
      if (currentSpeed < MIN_RUN_PWM) currentSpeed = MIN_RUN_PWM;  // 从停转加速直接进入可运转区间
      if (currentMode == MODE_MANUAL) applyManualMotion();  // 立即生效
      Serial.print(F("SPD+ "));
      Serial.println(currentSpeed);
      break;
    case '-':
    case 'D':
    case 'd':
      currentSpeed -= SPEED_STEP;
      if (currentSpeed < MIN_RUN_PWM) currentSpeed = 0;  // 减进死区直接停，避免只哼不转
      if (currentMode == MODE_MANUAL) applyManualMotion();  // 立即生效
      Serial.print(F("SPD- "));
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
        Serial.print(F("SPD "));
        Serial.println(currentSpeed);
        break;
      }
    default:
      Serial.println(F("(ignored)"));
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
  Serial.println(F("[MAN] Step done, stop"));
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
      Serial.println(F("[GUARD] Front obstacle, auto stop"));
    }
    carStop();  // 持续保持停车，防止被重复下发的 F 覆盖
  } else if (manualBlocked) {
    manualBlocked = false;
    applyManualMotion();  // 障碍清除，恢复前进
    Serial.println(F("[GUARD] Clear, resume FWD"));
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
  Serial.print(F("[1] LED on, free RAM: "));
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
    Serial.println(F("[2] OLED begin done"));
    display.clear();
    display.setCursor(0, 0);
    display.println("Hello World!");
    Serial.println(F("[3] OLED show done"));
    delay(2000);  // 开机画面停留
  } else {
    Serial.println(F("[3] OLED offline, run without display"));
  }
  Serial.println(F("[4] OLED stage done"));

  bluetoothSerial = new SoftwareSerial(8, 9);
  bluetoothSerial->begin(9600);
  Serial.println(F("[5] BLE init done"));

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

  Serial.println(F("=== Bluetooth Car Ready ==="));
  Serial.println(F("F/B/L/R/S=Manual  A=Auto  +/-=Speed  1-9=Gear"));
}

void loop() {
  unsigned long now = millis();

  // 确认扫描阶段由 updateVerifySweep 推进；前进阶段只盯正前；手动模式低频测距
  if (currentMode == MODE_AUTO && autoState == AUTO_VERIFY) {
    updateVerifySweep(now);
  } else if (currentMode == MODE_AUTO && autoState == AUTO_FORWARD) {
    updateForwardSonar(now);
  } else if (currentMode == MODE_MANUAL && now - lastSonarMs >= SONAR_INTERVAL) {
    // 手动模式优先保证蓝牙响应：先单发快测(约8ms)，只有近到需要拦停时才做中值确认
    int d = readSonarCmFast();
    if (d < APPROACH_DIST) d = readSonarFiltered();
    frontDistance = d;
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
