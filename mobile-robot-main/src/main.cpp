#include <Arduino.h>
#include "helper.h"
#include "Encoder.h"
#include "Motor.h"
#include "PID.h"

#define MAX_PWM 100

Motor motorl;
Motor motorr;

Encoder encl;
Encoder encr;

float kp = 2.0;
float ki = 0.01;
float kd = 0.01;

PID leftPID(kp, ki, kd, MAX_PWM);
PID rightPID(kp, ki, kd, MAX_PWM);

volatile int counterLeft = 0;  // This variable will increase or decrease depending on the rotation of encoder
volatile int counterRight = 0; // This variable will increase or decrease depending on the rotation of encoder

void enc_left()
{
  encl.update();
}

void enc_right()
{
  encr.update();
}

velocity v;
velocity v_target;

pid_param k;
pwm p;

int mode = 0; // 0: PWM, 1: PID

void setup()
{
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  while (!Serial)
    ;
  // waitStartCommand("mobile");
  digitalWrite(13, 1);
  motorl.init(5, 7, 4, 150);
  motorr.init(6, 9, 8, 150);

  encl.init(2, A1, false);
  encr.init(3, A0, true);

  attachInterrupt(digitalPinToInterrupt(encl.ena), enc_left, RISING);
  attachInterrupt(digitalPinToInterrupt(encr.ena), enc_right, RISING);
}

int c = 0;
const int interval = 50;
long previousMillis = 0;
long currentMillis = 0;
float setPoint = -20.0;
//float setPoint = getPosition;

void loop()
{
  currentMillis = millis();

  if (c > 3000)
  {
    motorl.rotate(0);
    motorr.rotate(0);
  }
  else
  {
    if (currentMillis - previousMillis > interval)
    {
      float deltaT = currentMillis - previousMillis;
      previousMillis = currentMillis;
      motorl.cal_velocity(encl.position);
      motorr.cal_velocity(encr.position);
      float v_l = motorl.v * 100.0;
      float v_r = motorr.v * 100.0;
      int pwmL = leftPID.evalu(v_l, setPoint, deltaT);
      int pwmR = rightPID.evalu(v_r, setPoint, deltaT);

      motorl.rotate(pwmL);
      motorr.rotate(pwmR);

      Serial.print(v_l);
      Serial.print("\t");
      Serial.print(v_r);
      Serial.print("\t");
      Serial.println(setPoint);

      c += interval;
    }
  }

  // while (c < 500)
  // {
  //   motorl.rotate(100);
  //   motorl.cal_velocity(encl.position);
  //   Serial.println(motorl.v * 100);
  //   delay(20);
  //   c++;
  // }

  // eventHandler(v_target, k, p, mode);

  // if(mode == 0)
  // {
  //   v_target.wl = 0;
  //   v_target.wr = 0;
  // }
  // else if (mode == 1)
  // {
  //   v = v_target;
  // }

  // sentMsg(v, k, p);
  // // Serial.println("X");
  // delay(100);
}
/*
  {"vl":1.00,"vr":2.00}
*/