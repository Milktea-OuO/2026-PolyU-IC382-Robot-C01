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
    volatile long ticks;
    long delta_ticks;
};

DCMotor motordata[] = {
    {0, 0,  0,  0,  0,  0,  0, 0},               // 索引 0 不使用
    {1, 32, 33, 25, 34, 35, 0, 0},               // 馬達 1
    {2, 18, 19, 21, 22, 23, 0, 0},               // 馬達 2
    {3, 4,  16, 17, 13, 5,  0, 0},               // 馬達 3
    {4, 26, 27, 14, 36, 39, 0, 0}                // 馬達 4
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

void IRAM_ATTR readEnc1() { TickCounterISR(&motordata[1]); }
void IRAM_ATTR readEnc2() { TickCounterISR(&motordata[2]); }
void IRAM_ATTR readEnc3() { TickCounterISR(&motordata[3]); }
void IRAM_ATTR readEnc4() { TickCounterISR(&motordata[4]); }

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

    attachInterrupt(digitalPinToInterrupt(motordata[1].encA), readEnc1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(motordata[2].encA), readEnc2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(motordata[3].encA), readEnc3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(motordata[4].encA), readEnc4, CHANGE);

    Serial.println("系統就緒！輸入 PWM1 PWM2 [PWM3 PWM4] (-255~255), 或輸入 stop 停止");
}

// ================== 主迴圈 ==================
void loop() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.length() > 0) {
            if (input.equalsIgnoreCase("stop")) {
                for (int i = 1; i <= 4; i++) moveMotor(i, 0);
                Serial.println("所有馬達已停止");
            } else {
                int pwm[4] = {0};
                int count = sscanf(input.c_str(), "%d %d %d %d", &pwm[0], &pwm[1], &pwm[2], &pwm[3]);
                if (count >= 2) {
                    for (int i = 0; i < count; i++) {
                        pwm[i] = constrain(pwm[i], -255, 255);
                        moveMotor(i + 1, pwm[i]);
                    }
                    if (count == 2) {   // 只給兩個 PWM 時，自動停止 3、4
                        moveMotor(3, 0);
                        moveMotor(4, 0);
                    }
                    // 輸出排版：每個馬達獨立一行
                    Serial.println("設定 PWM:");
                    Serial.printf("M1: %d\n", pwm[0]);
                    Serial.printf("M2: %d\n", pwm[1]);
                    Serial.printf("M3: %d\n", pwm[2]);
                    Serial.printf("M4: %d\n", pwm[3]);
                } else {
                    Serial.println("格式錯誤。請輸入至少 2 個 PWM (可多至 4 個)，或 stop");
                }
            }
        }
    }


    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate >= (unsigned long)(delta_time * 1000)) {
        lastUpdate = millis();
        updateDeltaTicks();
        showMeasurements();
    }
}