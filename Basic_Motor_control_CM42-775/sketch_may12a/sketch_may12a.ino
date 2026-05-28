#include <Arduino.h>

// ================== 全域參數 ==================
const int freq = 8000;
const int res = 8;
const int ppr = 20;
const int gear_ratio = 721;
const float ttr = ppr * gear_ratio * 2;   // 每轉總脈衝數（2倍頻）
const float delta_time = 0.5;             // 量測週期（秒）

// ================== 馬達結構 ==================
struct DCMotor {
    int id;
    int ena;
    int in1;
    int in2;
    int encA;
    int encB;
    volatile long ticks;      // 編碼器累計脈衝
    long delta_ticks;         // 週期內脈衝差
};

// ================== 馬達硬體設定 ==================
DCMotor motordata[] = {
    {0, 0,  0,  0,  0,  0,  0, 0},                // 索引 0 不使用
    {1, 32, 33, 25, 34, 35, 0, 0},               //馬達 1
    {2, 26, 27, 14, 36, 39, 0, 0},              // 馬達 2
    {3, 4,  16, 17, 13, 5,  0, 0},                // 未使用
    {4, 18, 19, 21, 22, 23, 0, 0}                 // 未用
};

const int motorCount = sizeof(motordata) / sizeof(DCMotor);
long previous_ticks[5] = {0};

// ================== 編碼器中斷 ==================
void IRAM_ATTR TickCounterISR(void* arg) {
    DCMotor* motor = (DCMotor*)arg;
    if (digitalRead(motor->encA) == digitalRead(motor->encB))
        motor->ticks++;
    else
        motor->ticks--;
}

void IRAM_ATTR readEnc1() {
    TickCounterISR(&motordata[1]);
}
void IRAM_ATTR readEnc2() {
    TickCounterISR(&motordata[2]);
}

// ================== 更新脈衝差 ==================
void updateDeltaTicks() {
    for (int i = 1; i < motorCount; i++) {
        if (motordata[i].ena == 0) continue;
        long current = motordata[i].ticks;
        motordata[i].delta_ticks = current - previous_ticks[i];
        previous_ticks[i] = current;
    }
}

// ================== 顯示 RPM 與角度 ==================
void showMeasurements() {
    for (int i = 1; i < motorCount; i++) {
        if (motordata[i].ena == 0) continue;
        float RPM = (motordata[i].delta_ticks * 60.0) / (delta_time * ttr);
        float Angle = (motordata[i].delta_ticks * 360.0) / ttr;
        Serial.printf("M%d -> RPM: %.2f, Angle: %.2f deg\n", i, RPM, Angle);
    }
}

// ================== 馬達驅動（PWM 範圍 -255 ~ 255）==================
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

        pinMode(motordata[i].in1, OUTPUT);
        pinMode(motordata[i].in2, OUTPUT);
        pinMode(motordata[i].encA, INPUT);
        pinMode(motordata[i].encB, INPUT);

        ledcSetup(motordata[i].id, freq, res);
        ledcAttachPin(motordata[i].ena, motordata[i].id);

        motordata[i].ticks = 0;
        motordata[i].delta_ticks = 0;
        previous_ticks[i] = 0;
    }

    // 掛載中斷（CHANGE 觸發，達到 2 倍頻）
    attachInterrupt(digitalPinToInterrupt(motordata[1].encA), readEnc1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(motordata[2].encA), readEnc2, CHANGE);

    Serial.println("系統就緒！輸入 PWM1 PWM2(-255~255),或輸入 stop 停止");
}

// ================== 主迴圈 ==================
void loop() {
    // ----- 接收 PWM 指令 -----
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.length() > 0) {
            if (input.equalsIgnoreCase("stop")) {
                moveMotor(1, 0);
                moveMotor(2, 0);
                Serial.println("馬達已停止");
            } else {
                int pwm1, pwm2;
                if (sscanf(input.c_str(), "%d %d", &pwm1, &pwm2) == 2) {
                    pwm1 = constrain(pwm1, -255, 255);
                    pwm2 = constrain(pwm2, -255, 255);
                    moveMotor(1, pwm1);
                    moveMotor(2, pwm2);
                    Serial.printf("設定 PWM -> M1: %d, M2: %d\n", pwm1, pwm2);
                } else {
                    Serial.println("格式錯誤。請輸入：<PWM1> <PWM2> 或 stop");
                }
            }
        }
    }

    // ----- 定時回報量測值（每 0.5 秒）-----
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate >= (unsigned long)(delta_time * 1000)) {
        lastUpdate = millis();
        updateDeltaTicks();
        showMeasurements();
    }
}