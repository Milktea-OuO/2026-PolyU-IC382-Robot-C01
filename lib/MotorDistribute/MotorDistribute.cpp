#include <MotorDistribute.h>

// 靜態成員初始化
MotorConfig MotorDistribute::motors[MAX_MOTORS];
uint8_t MotorDistribute::motorCount = 0;
uint8_t MotorDistribute::nextPwmChannel = 0;

void MotorDistribute::begin()
{
    // 初始化所有馬達配置
    for (int i = 0; i < MAX_MOTORS; i++)
    {
        motors[i].initialized = false;
        motors[i].id = -1;
    }
    motorCount = 0;
    nextPwmChannel = 0;
}

bool MotorDistribute::addMotor(int id, uint8_t in1Pin, uint8_t in2Pin, uint8_t enaPin)
{
    // 檢查是否已達最大數量
    if (motorCount >= MAX_MOTORS)
    {
        Serial.println("Error: Maximum motor count reached!");
        return false;
    }

    // 檢查 ID 是否已存在
    if (findMotorIndex(id) != -1)
    {
        Serial.printf("Error: Motor ID %d already exists!\n", id);
        return false;
    }

    // 檢查 PWM 頻道是否足夠
    if (nextPwmChannel >= 16)
    {
        Serial.println("Error: No available PWM channels!");
        return false;
    }

    // 初始化引腳模式
    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);
    pinMode(enaPin, OUTPUT);

    // 初始狀態：馬達停止
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    digitalWrite(enaPin, LOW);

    // 設定 PWM
    ledcSetup(nextPwmChannel, PWM_FREQ, PWM_RES);
    ledcAttachPin(enaPin, nextPwmChannel);

    // 儲存馬達配置
    motors[motorCount].id = id;
    motors[motorCount].in1Pin = in1Pin;
    motors[motorCount].in2Pin = in2Pin;
    motors[motorCount].enaPin = enaPin;
    motors[motorCount].pwmChannel = nextPwmChannel;
    motors[motorCount].initialized = true;

    // 更新計數器
    motorCount++;
    nextPwmChannel++;

    Serial.printf("Motor ID %d added (IN1:%d, IN2:%d, ENA:%d, PWM Chan:%d)\n",
                  id, in1Pin, in2Pin, enaPin, nextPwmChannel - 1);

    return true;
}

int MotorDistribute::findMotorIndex(int  id)
{
    for (int i = 0; i < motorCount; i++)
    {
        if (motors[i].initialized == true && motors[i].id == id)
        {
            return i;
        }
    }
    return -1;
}

void MotorDistribute::motorControl(int id, float speed)
{
    int index = findMotorIndex(id);

    if (index == -1)
    {
        Serial.printf("Error: Motor ID %d not found!\n", id);
        return;
    }
    else
    {
        Serial.printf("[%d] Motor speed: %d \n", id, speed);
    }

    // 限制速度範圍
    speed = constrain(speed, -255, 255);

    MotorConfig &motor = motors[index];
    int16_t pwmValue = abs(speed);
    Serial.printf("[%d] PWNValue: %d \n",id, pwmValue);

    if (speed > 0)
    {
        // 正轉
        digitalWrite(motor.in1Pin, HIGH);
        digitalWrite(motor.in2Pin, LOW);
        ledcWrite(motor.pwmChannel, pwmValue);
    }
    else if (speed < 0)
    {
        // 反轉
        digitalWrite(motor.in1Pin, LOW);
        digitalWrite(motor.in2Pin, HIGH);
        ledcWrite(motor.pwmChannel, pwmValue);
    }
    else
    {
        // 停止
        digitalWrite(motor.in1Pin, LOW);
        digitalWrite(motor.in2Pin, LOW);
        ledcWrite(motor.pwmChannel, 0);
    }
}

void MotorDistribute::motorStop(int id)
{
    motorControl(id, 0);
}

void MotorDistribute::stopAll()
{
    for (int i = 0; i < motorCount; i++)
    {
        if (motors[i].initialized == true)
        {
            motorControl(motors[i].id, 0);
        }
    }
    Serial.println("All motors stopped");
}

bool MotorDistribute::motorExists(int id)
{
    return findMotorIndex(id) != -1;
}

int MotorDistribute::getMotorCount()
{
    return motorCount;
}