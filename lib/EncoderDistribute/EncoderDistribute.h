#ifndef ENCODER_DISTRIBUTE_H
#define ENCODER_DISTRIBUTE_H

#include <Arduino.h>

#define MAX_ENCODERS 8

// Constants
const int PPR = 20;
const int GEAR_RATIO = 721;
const float TTR = PPR * GEAR_RATIO * 2;  // Total ticks per revolution
const float DELTA_TIME = 0.5;             // 0.5s sampling time

struct EncoderConfig {
    int motorId;
    uint8_t encAPin;
    uint8_t encBPin;
    volatile long ticks;
    long delta_ticks;
    long previous_ticks;
    bool initialized;
};

class EncoderDistribute {
private:
    static EncoderConfig encoders[MAX_ENCODERS];
    static uint8_t encoderCount;
    
    static int findEncoderIndex(int motorId);
    static void IRAM_ATTR tickCounterISR(EncoderConfig* encoder);
    
public:
    static void begin();
    static bool addEncoder(int motorId, uint8_t encAPin, uint8_t encBPin);
    static void removeEncoder(int motorId);
    static void updateDeltaTicks(int motorId);
    static void updateAllDeltaTicks();
    static long getTicks(int motorId);
    static long getDeltaTicks(int motorId);
    static float getRPM(int motorId);
    static float getAngle(int motorId);
    static void resetTicks(int motorId);
    static bool encoderExists(int motorId);
    static int getEncoderCount();
    
    // Static ISR handlers
    static void IRAM_ATTR readEnc1();
    static void IRAM_ATTR readEnc2();
    static void IRAM_ATTR readEnc3();
    static void IRAM_ATTR readEnc4();
};

#endif // ENCODER_DISTRIBUTE_H