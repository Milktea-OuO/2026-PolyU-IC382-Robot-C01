#ifndef __MOTOR_DISTRIBUTE_H
#define __MOTOR_DISTRIBUTE_H

#include <Arduino.h>

#define MAX_MOTORS 8

struct MotorConfig {
    int id;
    uint8_t in1Pin;
    uint8_t in2Pin;
    uint8_t enaPin;      // PWM 引腳
    uint8_t pwmChannel;  // ESP32 PWM 頻道
    bool initialized;
};

class MotorDistribute {
private:
    static MotorConfig motors[MAX_MOTORS];
    static uint8_t motorCount;
    static uint8_t nextPwmChannel;
    static const int PWM_FREQ = 200;
    static const uint8_t PWM_RES = 8;
    
    // 查找馬達配置
    static int findMotorIndex(int id);
    
public:
    // 初始化庫（可選，會自動初始化）
    static void begin();
    
    // 新增馬達：id(自訂編號), in1, in2, ena(PWM引腳)
    static bool addMotor(int id, uint8_t in1Pin, uint8_t in2Pin, uint8_t enaPin);
    
    // 控制馬達：id(馬達編號), speed(-255 到 255)
    static void motorControl(int  id, float speed);
    
    // 停止單一馬達
    static void motorStop(int id);
    
    // 停止所有馬達
    static void stopAll();
    
    // 檢查馬達是否存在
    static bool motorExists(int id);
    
    // 獲取馬達數量
    static int getMotorCount();
};

#endif