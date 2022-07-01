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

int mode = 1; // 0: PWM, 1: PID

void setup()
{
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  while (!Serial)
    ;
  // waitStartCommand("mobile");
  digitalWrite(13, 1);
  motorl.init(5, 7, 4, 20);
  motorr.init(6, 9, 8, 20);

  encl.init(2, A1, false);
  encr.init(3, A0, true);

  attachInterrupt(digitalPinToInterrupt(encl.ena), enc_left, RISING);
  attachInterrupt(digitalPinToInterrupt(encr.ena), enc_right, RISING);
}

int c = 0;
const int interval = 50;
long previousMillis = 0;
long currentMillis = 0;
float setPoint = -5.0;

float Position = 0;
const unsigned int WordLength = 12;

void loop()
{
  //  while(1)
  //    {
  //    boolean EndCommand = 0;
  //    while(Serial.available()>0)
  //      {
  //      static char Command[WordLength];
  //      static unsigned int WordPosition = 0;
  //      char InByte = Serial.read();
  //      if(InByte != '\n')
  //        {
  //        Command[WordPosition] = InByte;
  //        WordPosition++;
  //        }
  //      else
  //        {
  //        Command[WordPosition] = '\0';
  //        Serial.print("input = ");
  //        Serial.println(Command);
  //        int Data0 = (Command[0]);
  //        int Data1 = (Command[1]-'0')*1000;
  //        int Data2 = (Command[2]-'0')*100;
  //        int Data3 = (Command[3]-'0')*10;
  //        int Data4 = (Command[4]-'0');
  //        int getPosition;
  //        if(Data0 == 43)
  //          {
  //          getPosition = Data1+Data2+Data3+Data4;
  //          }
  //        else if (Data0 == 45)
  //          {
  //          getPosition = -1*(Data1+Data2+Data3+Data4);
  //          }
  //        EndCommand = 1;
  //        }
  //      if(EndCommand == 1)
  //        {
  //        break;
  //        }
  //      }
  //    }
  float getPosition = 2000; //test send value

  Position = getPosition / 1000 * 1350; //change mm unit to m unit
  currentMillis = millis();

  if (c > Position) //1350 unit = 1m
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
      float v_l = motorl.v * 100;
      float v_r = motorr.v * 110;
      int pwmL = leftPID.evalu(v_l, setPoint, deltaT);
      int pwmR = rightPID.evalu(v_r, setPoint, deltaT);

      motorl.rotate(pwmL);
      motorr.rotate(pwmR+15);

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
