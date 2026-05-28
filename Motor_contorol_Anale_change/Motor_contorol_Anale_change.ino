#include <Arduino.h>

// 硬體參數
const int freq = 5000;
const int res = 8;
const int ppr = 20;
const int gear_ratio = 721;
const float ttr = ppr * gear_ratio;

struct DCMotor {
    int id, ena, in1, in2, encA, encB;
    volatile long ticks;
};

DCMotor motordata[] = {
    {0, 0, 0, 0, 0, 0, 0},
    {1, 26, 27, 14, 33, 25, 0},              //5 ，17
    {2, 17, 16, 4, 18, 5, 0},              //25, 33, 32, 18, 19,
    {3, 0, 0, 0, 0, 0, 0},
    {4, 0, 0, 0, 0, 0, 0}
};

const int motorCount = sizeof(motordata) / sizeof(DCMotor);

// 位置控制變數
long target_ticks[5] = {0};
bool motor_busy[5] = {false};
const int TOLERANCE = 3;
const int MAX_PWM = 200;
const float KP = 0.6;

// 中斷服務
void IRAM_ATTR TickCounterISR(void* arg) {
   DCMotor* motor = (DCMotor*)arg;
  if (digitalRead(motor->encA) == digitalRead(motor->encB))
    motor->ticks++;
  else
    motor->ticks--;
}
void IRAM_ATTR readEnc1() { TickCounterISR(&motordata[1]); }
void IRAM_ATTR readEnc2() { TickCounterISR(&motordata[2]); }

// 馬達驅動
void moveMotor(int index, int pwm) {
    if (index <= 0 || index >= motorCount || motordata[index].ena == 0) return;
    pwm = constrain(pwm, -255, 255);
    DCMotor &m = motordata[index];
    if (pwm > 0) {
        digitalWrite(m.in1, HIGH);
        digitalWrite(m.in2, LOW);
        ledcWrite(m.id, pwm);
    } else if (pwm < 0) {
        digitalWrite(m.in1, LOW);
        digitalWrite(m.in2, HIGH);
        ledcWrite(m.id, -pwm);
    } else {
        digitalWrite(m.in1, LOW);
        digitalWrite(m.in2, LOW);
        ledcWrite(m.id, 0);
    }
}

// 設定目標角度
void setTargetAngle(int motorID, float angle_deg) {
    if (motorID <= 0 || motorID >= motorCount || motordata[motorID].ena == 0) return;
    long delta = (long)(angle_deg / 360.0 * ttr);
    target_ticks[motorID] = motordata[motorID].ticks + delta;
    motor_busy[motorID] = true;
    Serial.printf("M%d target: %.2f° (%ld ticks)\n", motorID, angle_deg, target_ticks[motorID]);
}

// 非阻塞位置控制（已修正）
void positionControl() {
    for (int i = 1; i < motorCount; i++) {
        if (!motor_busy[i] || motordata[i].ena == 0) continue;
        long current = motordata[i].ticks;
        long error = target_ticks[i] - current;

        if (abs(error) <= TOLERANCE) {
            moveMotor(i, 0);
            motor_busy[i] = false;
            Serial.printf("M%d reached target.\n", i);
        } else {
            int pwm = constrain((int)(error * KP), -MAX_PWM, MAX_PWM);
            moveMotor(i, pwm);
        }
    }
}

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
    }
    attachInterrupt(digitalPinToInterrupt(motordata[1].encA), readEnc1, RISING);
    attachInterrupt(digitalPinToInterrupt(motordata[2].encA), readEnc2, RISING);

    Serial.println("Ready. Input: <angle1> <angle2> or stop");
}

void loop() {
    if (Serial.available()) {
      static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1000) {
        lastPrint = millis();
        Serial.printf("M1 ticks: %ld, M2 ticks: %ld\n",
          motordata[1].ticks, motordata[2].ticks);
      }
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.length() > 0) {
            float a1, a2;
            if (sscanf(input.c_str(), "%f %f", &a1, &a2) == 2) {
                setTargetAngle(1, a1);
                setTargetAngle(2, a2);
            } else if (input.equalsIgnoreCase("stop")) {
                for (int i = 1; i < motorCount; i++) {
                    moveMotor(i, 0);
                    motor_busy[i] = false;
                }
                Serial.println("All motors stopped.");
            } else {
                float a;
                if (sscanf(input.c_str(), "%f", &a) == 1) {
                    setTargetAngle(1, a);
                } else {
                    Serial.println("Invalid. Use: <angle1> <angle2> or stop");
                }
            }
        }
    }

    positionControl();
    delay(2);
}