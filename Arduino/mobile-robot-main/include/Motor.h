#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

const float TICKS_PER_METER = 1949;

class Motor
{
private:
    int en, in1, in2, maxPwm;
    double prevTime = 0;
    int prevLeftCount = 0;
public:
    Motor();
    void init(int enIn, int in1In, int in2In, int maxPwmIn);
    void rotate(int pwm);
    void cal_velocity(int encoder_tick);
    double v = 0;
};

Motor::Motor(){}

void Motor::init(int enIn, int in1In, int in2In, int maxPwmIn)
{
    en = enIn;
    in1 = in1In;
    in2 = in2In;
    maxPwm = maxPwmIn;

    pinMode(en, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

    rotate(0);
}

void Motor::rotate(int pwm)
{
    pwm = constrain(pwm, -maxPwm, maxPwm);
    if (pwm < -1)
    {
        digitalWrite(in1, 1);
        digitalWrite(in2, 0);
        analogWrite(en, -pwm);
    }
    else if (pwm > 1)
    {
        digitalWrite(in1, 0);
        digitalWrite(in2, 1);
        analogWrite(en, pwm);
    }
    else
    {
        digitalWrite(in1, 0);
        digitalWrite(in2, 0);
        analogWrite(en, 0);
    }
}

void Motor::cal_velocity(int encoder_tick)
{
    // Manage rollover and rollunder when we get outside the 16-bit integer range
    float numOfTicks = (65535 + encoder_tick - prevLeftCount) % 65535;

    // If we have had a big jump, it means the tick count has rolled over.
    if (numOfTicks > 10000)
    {
        numOfTicks = 0 - (65535 - numOfTicks);
    }

    // Calculate wheel velocity in meters per second
    v = (numOfTicks / TICKS_PER_METER) / ((millis() / 1000.0) - prevTime);
    // Keep track of the previous tick count
    prevLeftCount = encoder_tick;

    // Update the timestamp
    prevTime = (millis() / 1000.0);
}
#endif