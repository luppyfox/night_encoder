#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID
{
private:
    float kp, ki, kd, outMax;
    float eprev, eintegral;
public:
    PID(float kpIn, float kiIn, float kdIn, float outMaxIn);
    int evalu(float pos, float setPoint, float deltaT);
};

PID::PID(float kpIn, float kiIn, float kdIn, float outMaxIn)
{
    kp=kpIn; ki=kiIn; kd=kdIn; outMax=outMaxIn;
}

int PID::evalu(float pos, float setPoint, float deltaT)
{
    // error
    float error = setPoint - pos;
    //derivative term
    float dedt = (error - eprev) / deltaT;
    // integral term
    eintegral = eintegral + error * deltaT;
    // compute control signal
    float u = kp*error + ki*eintegral + kd*dedt;
    //store previous error
    eprev = error;
    //control motor
    return constrain(u, -outMax, outMax);
}

#endif