#include <Arduino.h>

// ================== 全域參數 ==================
const int freq = 8000;
const int res = 8;
const int ppr = 10;
const int gear_ratio = 369;
const float ttr = ppr * gear_ratio * 2;  // 使用 2 倍頻 (A 相 CHANGE 觸發)
const float delta_time = 0.5;            // 控制週期 0.5 秒

// ================== 馬達腳位結構 ==================
struct DCMotor {
    int id;
    int ena;
    int in1;
    int in2;
    int encA;
    int encB;
    volatile long ticks;      // 編碼器累積計數 (中斷中修改, 須 volatile)
    long delta_ticks;         // 上一個週期內的計數差
};

// ================== 馬達控制用變數 ==================
struct MotorControl {
    float target_rpm;
    float integral;
    float prev_error;
    float kp;
    float ki;
};

// ================== 硬體設定 ==================
DCMotor motordata[] = {
    {0, 0, 0, 0, 0, 0, 0, 0},                // 索引 0 不使用
    {1, 26, 27, 14, 5, 17, 0},               //馬達 1
    {2, 25, 33, 32, 18, 19, 0},              // 馬達 2
    {3, 0, 0, 0, 0, 0, 0, 0},                // 未使用
    {4, 0, 0, 0, 0, 0, 0, 0}                 // 未使用
};

MotorControl ctrl[] = {
    {0, 0, 0, 0, 0},   // 索引 0 (不使用)
    {0, 0, 0, 0, 0},   // 馬達 1 的 PI 參數 (可自行調整)
    {0, 0, 0, 0, 0},   // 馬達 2
    {0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0}
};

const int motorCount = sizeof(motordata) / sizeof(DCMotor);
long previous_ticks[5] = {0};  // 5 個元素對應 5 個馬達 (索引 0~4)

// 全域目標轉速 (從序列埠讀取)
float target_rpm1 = 0.0;
float target_rpm2 = 0.0;

// ================== 中斷服務程式 (ISR) ==================
void IRAM_ATTR TickCounterISR(void* arg) {
    DCMotor* motor = (DCMotor*)arg;
    // 2 倍頻解碼：A 相變化時, 若 A==B 則正向, 否則反向
    if (digitalRead(motor->encA) == digitalRead(motor->encB))
        motor->ticks++;
    else
        motor->ticks--;
}

// 各馬達專屬 ISR (直接綁定，避免動態判斷)
void IRAM_ATTR readEnc1() {
    TickCounterISR(&motordata[1]);
}
void IRAM_ATTR readEnc2() {
    TickCounterISR(&motordata[2]);
}

// ================== 計算 delta_ticks ==================
void updateDeltaTicks() {
    for (int i = 1; i < motorCount; i++) {
        if (motordata[i].ena == 0) continue;
        long current = motordata[i].ticks;
        long delta = current - previous_ticks[i];
        motordata[i].delta_ticks = delta;
        previous_ticks[i] = current;
    }
}

// ================== 計算並輸出 RPM 與角度 ==================
void showMeasurements() {
    for (int i = 1; i < motorCount; i++) {
        if (motordata[i].ena == 0) continue;
        float RPM = (motordata[i].delta_ticks * 60.0) / (delta_time * ttr);
        float Angle = (motordata[i].delta_ticks * 360.0) / ttr;
        Serial.printf("M%d -> RPM: %.2f, Angle: %.2f deg\n", i, RPM, Angle);
    }
}

// ================== PI 速度控制 ==================
void speedControl() {
    for (int i = 1; i < motorCount; i++) {
        if (motordata[i].ena == 0) continue;

        // 計算當前 RPM
        float current_rpm = (motordata[i].delta_ticks * 60.0) / (delta_time * ttr);
        float error = ctrl[i].target_rpm - current_rpm;

        // PI 計算
        ctrl[i].integral += error * delta_time;
        // 簡易積分限幅 (防止 windup)
        if (ctrl[i].integral > 100.0) ctrl[i].integral = 100.0;
        if (ctrl[i].integral < -100.0) ctrl[i].integral = -100.0;

        float output = ctrl[i].kp * error + ctrl[i].ki * ctrl[i].integral;

        // 限制 PWM 輸出範圍
        int pwm = (int)constrain(output, -255.0, 255.0);
        moveMotor(i, pwm);
    }
}

// ================== 馬達驅動 ==================
void moveMotor(int index, int PWM_speed) {
    if (index <= 0 || index >= motorCount) return;
    PWM_speed = constrain(PWM_speed, -255, 255);
    DCMotor &m = motordata[index];

    if (PWM_speed > 0) {
        digitalWrite(m.in1, HIGH);
        digitalWrite(m.in2, LOW);
        ledcWrite(m.id, abs(PWM_speed));
    } else if (PWM_speed < 0) {
        digitalWrite(m.in1, LOW);
        digitalWrite(m.in2, HIGH);
        ledcWrite(m.id, abs(PWM_speed));
    } else {
        digitalWrite(m.in1, LOW);
        digitalWrite(m.in2, LOW);
        ledcWrite(m.id, 0);
    }
}

// ================== 初始化 ==================
void setup() {
    Serial.begin(115200);

    for (int i = 1; i < motorCount; i++) {
        if (motordata[i].ena == 0) continue;

        // GPIO 設定
        pinMode(motordata[i].in1, OUTPUT);
        pinMode(motordata[i].in2, OUTPUT);
        pinMode(motordata[i].encA, INPUT);
        pinMode(motordata[i].encB, INPUT);

        // LEDC PWM 設定
        ledcSetup(motordata[i].id, freq, res);
        ledcAttachPin(motordata[i].ena, motordata[i].id);

        // 初始化 ticks
        motordata[i].ticks = 0;
        motordata[i].delta_ticks = 0;
        previous_ticks[i] = 0;
    }

    // 中斷掛接 (A 相 CHANGE 觸發，實現 2 倍頻)
    attachInterrupt(digitalPinToInterrupt(motordata[1].encA), readEnc1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(motordata[2].encA), readEnc2, CHANGE);

    // 初始化控制目標
    ctrl[1].target_rpm = 0.0;
    ctrl[2].target_rpm = 0.0;

    Serial.println("System Ready! 輸入格式: <rpm1> <rpm2> 或 stop");
}

// ================== 主迴圈 ==================
void loop() {
    // ----- 序列埠接收目標轉速 -----
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.length() > 0) {
            float r1, r2;
            if (sscanf(input.c_str(), "%f %f", &r1, &r2) == 2) {
                target_rpm1 = r1;
                target_rpm2 = r2;
                ctrl[1].target_rpm = target_rpm1;
                ctrl[2].target_rpm = target_rpm2;
                Serial.printf("設定目標 -> M1: %.1f RPM, M2: %.1f RPM\n", target_rpm1, target_rpm2);
            } else if (input.equalsIgnoreCase("stop")) {
                target_rpm1 = 0.0;
                target_rpm2 = 0.0;
                ctrl[1].target_rpm = 0.0;
                ctrl[2].target_rpm = 0.0;
                Serial.println("馬達停止");
            } else {
                Serial.println("格式錯誤。請使用: <rpm1> <rpm2> 或 stop");
            }
        }
    }

    // ----- 定時控制週期 (每 0.5 秒) -----
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate >= (unsigned long)(delta_time * 1000)) {
        lastUpdate = millis();

        updateDeltaTicks();      // 更新計數差
        showMeasurements();      // 顯示 RPM 與角度
        speedControl();          // 執行 PI 速度控制
    }
}