/*
 * BLDC4260 火車式 5 檔控制 (接腳修正 + 轉向/回饋修正)
 * 硬體接線 (對應結構體)：
 *   PWM  (藍) -> GPIO21  (25 kHz, 8-bit: 0~255)
 *   方向 (白) -> GPIO19  (HIGH=正轉, LOW=反轉)
 *   剎車 (綠) -> GPIO18  (HIGH=運行, LOW=剎車)
 *   FG   (黃) -> GPIO17  (外部 4.7k 上拉至 3.3V, 12脈衝/圈)
 *   VCC  (紅) -> 24V 電源正極
 *   GND  (黑) -> 24V 電源負極 + ESP32 GND (共地)
 */

// ---------- 電機資料結構 ----------
struct MainPowerMotor {
  int id;
  int Dir;
  int FG;
  int PWMpin;
  int Brake;
};

MainPowerMotor motordata[] = {
  {0, 0, 0, 0, 0},          // 占位
  {1, 19, 17, 21, 18},      // 修正後的實際接腳
  {2, 0, 0, 0, 0}
};

MainPowerMotor motor = motordata[1];

// ---------- 引腳巨集 (方便閱讀) ----------
#define DIR_PIN    motor.Dir    // GPIO19
#define FG_PIN     motor.FG     // GPIO17
#define PWM_PIN    motor.PWMpin // GPIO21
#define BRAKE_PIN  motor.Brake  // GPIO18

// PWM 參數
const int PWM_FREQ = 25000;
const int PWM_RES  = 8;         // 0~255
const int PWM_CH   = 0;         // LEDC 通道 0

// ---------- 檔位定義 ----------
const int MIN_GEAR = -2;
const int MAX_GEAR = 2;
int currentGear = 0;   // 預設 0 檔 (剎車)

// 方向定義
#define DIR_FORWARD  LOW   // 正轉 (前進)
#define DIR_BACKWARD HIGH    // 反轉 (後退)

// 剎車定義 (HIGH 運行, LOW 剎車)
#define BRAKE_RELEASE HIGH
#define BRAKE_ACTIVE  LOW

// ---------- PWM 對應轉速 (反比關係) ----------
// PWM = 0   → 最快 (gear ±2)
// PWM = 255 → 馬達不轉 (gear 0)
const int PWM_FAST = 0;      // 最快
const int PWM_SLOW = 100;    // 慢速 (可調整, 越接近255越慢)

// ---------- 轉速回饋變數 ----------
volatile unsigned long pulseCount = 0;   // FG 中斷累積脈衝
unsigned long lastCalcTime = 0;
float currentRPM = 0.0;
const int PULSE_PER_REV = 12;

// ---------- 中斷服務程式 (需 IRAM_ATTR) ----------
void IRAM_ATTR fgISR() {
  pulseCount++;
}

// ---------- 將檔位轉換為 PWM 值 ----------
int gearToPWM(int gear) {
  if (gear == 0) return 255;           // 剎車：馬達不轉
  int absGear = abs(gear);
  if (absGear == 2) return PWM_FAST;   // 最快 (PWM=0)
  if (absGear == 1) return PWM_SLOW;   // 慢速
  return 255; // 保險
}

// ---------- 根據檔位控制馬達 (解決轉向問題) ----------
void applyMotorState() {
  if (currentGear == 0) {
    // 0 檔：剎車 + 馬達停止
    digitalWrite(BRAKE_PIN, BRAKE_ACTIVE);   // LOW 剎車
    digitalWrite(DIR_PIN, DIR_FORWARD);      // 方向任意，預設正轉
    ledcWrite(PWM_CH, 255);                  // PWM=255 停止
  } else {
    // 非 0 檔：釋放剎車，設定方向，輸出 PWM
    digitalWrite(BRAKE_PIN, BRAKE_RELEASE);  // HIGH 運行

    // ★ 轉向修正：正檔位前進，負檔位後退
    if (currentGear > 0) {
      digitalWrite(DIR_PIN, DIR_FORWARD);    // HIGH = 正轉
    } else {
      digitalWrite(DIR_PIN, DIR_BACKWARD);   // LOW  = 反轉
    }

    // 輸出對應 PWM
    ledcWrite(PWM_CH, gearToPWM(currentGear));
  }
}

// ---------- 序列埠指令處理 ----------
void handleSerial() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == '\n' || cmd == '\r') return;

    switch (cmd) {
      case 'w':
      case 'W':
        if (currentGear < MAX_GEAR) currentGear++;
        applyMotorState();
        break;
      case 's':
      case 'S':
        if (currentGear > MIN_GEAR) currentGear--;
        applyMotorState();
        break;
      case 'r':
      case 'R':
        currentGear = 0;  // 緊急剎車強制歸零
        applyMotorState();
        Serial.println("!!! 緊急剎車 (檔位歸零) !!!");
        break;
      default:
        Serial.println("未知指令, 請使用 W / S / R");
        break;
    }
  }
}

// ---------- 轉速計算 (每秒更新) 解決回饋問題 ----------
void updateRPM() {
  unsigned long now = millis();
  if (now - lastCalcTime >= 1000) {
    // 安全讀取並清零脈衝計數
    unsigned long pulses;
    noInterrupts();
    pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    // 每分鐘轉速 = (脈衝數 / 12) * 60
    currentRPM = (float)pulses / PULSE_PER_REV * 60.0;
    lastCalcTime = now;
  }
}

// ---------- 定時狀態輸出 ----------
void printStatus() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();

    Serial.print("檔位: ");
    if (currentGear > 0) Serial.print("+");
    Serial.print(currentGear);

    Serial.print(" | PWM: ");
    Serial.print(gearToPWM(currentGear));
    Serial.print("/255");

    Serial.print(" | 方向: ");
    if (currentGear == 0) Serial.print("--");
    else Serial.print(currentGear > 0 ? "前進" : "後退");

    Serial.print(" | 剎車: ");
    Serial.print(currentGear == 0 ? "ON" : "OFF");

    Serial.print(" | 轉速: ");
    Serial.print(currentRPM, 1);
    Serial.println(" RPM");
  }
}

// ---------- 初始化 ----------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n===== BLDC 火車油門 (5檔 / 接腳修正) =====");
  Serial.println("W:加檔 | S:減檔 | R:緊急剎車");
  Serial.println("-2:後退最快  -1:後退慢速  0:剎車  1:慢速前進  2:前進最快");
  Serial.println("===========================================\n");

  // PWM 設定 (LEDC)
  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_PIN, PWM_CH);
  ledcWrite(PWM_CH, 255);   // 初始停止 (PWM=255)

  // 方向與剎車接腳
  pinMode(DIR_PIN, OUTPUT);
  pinMode(BRAKE_PIN, OUTPUT);

  // 初始狀態：剎車鎖定
  digitalWrite(BRAKE_PIN, BRAKE_ACTIVE);  // 剎車
  digitalWrite(DIR_PIN, DIR_FORWARD);     // 預設方向

  // FG 轉速回饋接腳 (外部已上拉，設為 INPUT 即可)
  pinMode(FG_PIN, INPUT);
  // 中斷：偵測 FG 腳位的上升緣 (RISING)
  attachInterrupt(digitalPinToInterrupt(FG_PIN), fgISR, RISING);

  lastCalcTime = millis();
}

void loop() {
  handleSerial();
  updateRPM();
  printStatus();
}