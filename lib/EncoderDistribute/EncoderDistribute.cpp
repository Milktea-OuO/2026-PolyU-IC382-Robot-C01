#include "EncoderDistribute.h"

// Static members initialization
EncoderConfig EncoderDistribute::encoders[MAX_ENCODERS];
uint8_t EncoderDistribute::encoderCount = 0;

void EncoderDistribute::begin()
{
    // Initialize all encoder configurations
    for (int i = 0; i < MAX_ENCODERS; i++)
    {
        encoders[i].initialized = false;
        encoders[i].motorId = -1;
        encoders[i].ticks = 0;
        encoders[i].delta_ticks = 0;
        encoders[i].previous_ticks = 0;
    }
    encoderCount = 0;
    
    Serial.println("Encoder system initialized");
}

int EncoderDistribute::findEncoderIndex(int motorId)
{
    for (int i = 0; i < encoderCount; i++)
    {
        if (encoders[i].initialized == true && encoders[i].motorId == motorId)
        {
            return i;
        }
    }
    return -1;
}

void IRAM_ATTR EncoderDistribute::tickCounterISR(EncoderConfig* encoder)
{
    if (digitalRead(encoder->encAPin) == digitalRead(encoder->encBPin))
        encoder->ticks++;
    else
        encoder->ticks--;
}

// Individual ISR handlers for each encoder
void IRAM_ATTR EncoderDistribute::readEnc1()
{
    // Find encoder with motor ID 1
    for (int i = 0; i < encoderCount; i++)
    {
        if (encoders[i].initialized && encoders[i].motorId == 1)
        {
            tickCounterISR(&encoders[i]);
            break;
        }
    }
}

void IRAM_ATTR EncoderDistribute::readEnc2()
{
    for (int i = 0; i < encoderCount; i++)
    {
        if (encoders[i].initialized && encoders[i].motorId == 2)
        {
            tickCounterISR(&encoders[i]);
            break;
        }
    }
}

void IRAM_ATTR EncoderDistribute::readEnc3()
{
    for (int i = 0; i < encoderCount; i++)
    {
        if (encoders[i].initialized && encoders[i].motorId == 3)
        {
            tickCounterISR(&encoders[i]);
            break;
        }
    }
}

void IRAM_ATTR EncoderDistribute::readEnc4()
{
    for (int i = 0; i < encoderCount; i++)
    {
        if (encoders[i].initialized && encoders[i].motorId == 4)
        {
            tickCounterISR(&encoders[i]);
            break;
        }
    }
}

bool EncoderDistribute::addEncoder(int motorId, uint8_t encAPin, uint8_t encBPin)
{
    // Check if maximum encoders reached
    if (encoderCount >= MAX_ENCODERS)
    {
        Serial.println("Error: Maximum encoder count reached!");
        return false;
    }
    
    // Check if encoder for this motor already exists
    if (findEncoderIndex(motorId) != -1)
    {
        Serial.printf("Error: Encoder for Motor ID %d already exists!\n", motorId);
        return false;
    }
    
    // Initialize encoder pins
    pinMode(encAPin, INPUT_PULLUP);
    pinMode(encBPin, INPUT_PULLUP);
    
    // Store encoder configuration
    encoders[encoderCount].motorId = motorId;
    encoders[encoderCount].encAPin = encAPin;
    encoders[encoderCount].encBPin = encBPin;
    encoders[encoderCount].ticks = 0;
    encoders[encoderCount].delta_ticks = 0;
    encoders[encoderCount].previous_ticks = 0;
    encoders[encoderCount].initialized = true;
    
    // Attach interrupt based on motor ID
    switch (motorId)
    {
        case 1:
            attachInterrupt(digitalPinToInterrupt(encAPin), readEnc1, RISING);
            break;
        case 2:
            attachInterrupt(digitalPinToInterrupt(encAPin), readEnc2, RISING);
            break;
        case 3:
            attachInterrupt(digitalPinToInterrupt(encAPin), readEnc3, RISING);
            break;
        case 4:
            attachInterrupt(digitalPinToInterrupt(encAPin), readEnc4, RISING);
            break;
        default:
            Serial.printf("Error: Motor ID %d not supported for interrupt!\n", motorId);
            return false;
    }
    
    encoderCount++;
    
    Serial.printf("Encoder added for Motor ID %d (EncA:%d, EncB:%d)\n",
                  motorId, encAPin, encBPin);
    
    return true;
}

void EncoderDistribute::removeEncoder(int motorId)
{
    int index = findEncoderIndex(motorId);
    
    if (index == -1)
    {
        Serial.printf("Error: Encoder for Motor ID %d not found!\n", motorId);
        return;
    }
    
    // Detach interrupt (optional - would need to track which pin was used)
    // For simplicity, we'll just mark as not initialized
    
    // Shift remaining encoders
    for (int i = index; i < encoderCount - 1; i++)
    {
        encoders[i] = encoders[i + 1];
    }
    
    encoderCount--;
    
    Serial.printf("Encoder removed for Motor ID %d\n", motorId);
}

void EncoderDistribute::updateDeltaTicks(int motorId)
{
    int index = findEncoderIndex(motorId);
    
    if (index == -1)
    {
        Serial.printf("Error: Encoder for Motor ID %d not found!\n", motorId);
        return;
    }
    
    EncoderConfig &encoder = encoders[index];
    long current = encoder.ticks;
    long delta = current - encoder.previous_ticks;
    encoder.delta_ticks = delta;
    encoder.previous_ticks = current;
}

void EncoderDistribute::updateAllDeltaTicks()
{
    for (int i = 0; i < encoderCount; i++)
    {
        if (encoders[i].initialized)
        {
            long current = encoders[i].ticks;
            long delta = current - encoders[i].previous_ticks;
            encoders[i].delta_ticks = delta;
            encoders[i].previous_ticks = current;
        }
    }
}

long EncoderDistribute::getTicks(int motorId)
{
    int index = findEncoderIndex(motorId);
    
    if (index == -1)
    {
        Serial.printf("Error: Encoder for Motor ID %d not found!\n", motorId);
        return 0;
    }
    
    return encoders[index].ticks;
}

long EncoderDistribute::getDeltaTicks(int motorId)
{
    int index = findEncoderIndex(motorId);
    
    if (index == -1)
    {
        Serial.printf("Error: Encoder for Motor ID %d not found!\n", motorId);
        return 0;
    }
    
    return encoders[index].delta_ticks;
}

float EncoderDistribute::getRPM(int motorId)
{
    int index = findEncoderIndex(motorId);
    
    if (index == -1)
    {
        Serial.printf("Error: Encoder for Motor ID %d not found!\n", motorId);
        return 0;
    }
    
    float RPM = (encoders[index].delta_ticks * 60.0) / (DELTA_TIME * TTR);
    return RPM;
}

float EncoderDistribute::getAngle(int motorId)
{
    int index = findEncoderIndex(motorId);
    
    if (index == -1)
    {
        Serial.printf("Error: Encoder for Motor ID %d not found!\n", motorId);
        return 0;
    }
    
    float Angle = (encoders[index].delta_ticks * 360.0) / TTR;
    return Angle;
}

void EncoderDistribute::resetTicks(int motorId)
{
    int index = findEncoderIndex(motorId);
    
    if (index == -1)
    {
        Serial.printf("Error: Encoder for Motor ID %d not found!\n", motorId);
        return;
    }
    
    // Disable interrupts temporarily
    noInterrupts();
    encoders[index].ticks = 0;
    encoders[index].delta_ticks = 0;
    encoders[index].previous_ticks = 0;
    interrupts();
    
    Serial.printf("Ticks reset for Motor ID %d\n", motorId);
}

bool EncoderDistribute::encoderExists(int motorId)
{
    return findEncoderIndex(motorId) != -1;
}

int EncoderDistribute::getEncoderCount()
{
    return encoderCount;
}