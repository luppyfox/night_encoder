#include <AccelStepper.h> 
#include <Servo.h>
AccelStepper stepper1(1, 3, 13);
AccelStepper stepper2(1, 5, 15);
AccelStepper stepper3(1, 7, 17);
AccelStepper stepper4(1, 9, 19);
Servo servo1;
Servo servo2;
//Set Default Speed
int SDSp = 5000;
//Set Default Acceleration                         
int SDAc = 5000;
int pos = 0;
const unsigned int WordLength = 12;

void setup() 
  {
  Serial.begin(9600);
  stepper1.disableOutputs();
  stepper2.disableOutputs();
  stepper3.disableOutputs();
  stepper4.disableOutputs();
  pinMode(A0,INPUT_PULLUP);
  pinMode(A1,INPUT_PULLUP);
  pinMode(A2,INPUT_PULLUP);
  pinMode(A3,INPUT_PULLUP);
  servo1.attach(31);
  servo2.attach(30);
  for (pos = 500; pos <= 2500; pos += 20) 
    {
    servo1.writeMicroseconds(pos);
    servo2.writeMicroseconds(pos);
    delay(25);                              
    }
  delay(1000);
  for (pos = 2500; pos >= 500; pos -= 20) 
    { 
    servo1.writeMicroseconds(pos);
    servo2.writeMicroseconds(pos);
    delay(25);                            
    }
  delay(1000);
  set0();
  ArmRunToDeg(91,105,-75,5000,SDAc);
  delay(1000);
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);
  stepper4.setCurrentPosition(0);
  stepper1.setMaxSpeed(SDSp);
  stepper1.setAcceleration(SDAc);
  stepper2.setMaxSpeed(SDSp);
  stepper2.setAcceleration(SDAc);
  stepper3.setMaxSpeed(SDSp);
  stepper3.setAcceleration(SDAc);
  stepper4.setMaxSpeed(SDSp);
  stepper4.setAcceleration(SDAc);
  Serial.println("StandBy");
  }

void loop() 
  {
  boolean EndCommand = 0;
  while(Serial.available()>0)
    {
    static char Command[WordLength];
    static unsigned int WordPosition = 0;
    char InByte = Serial.read();
    if(InByte != '\n')
      {
      Command[WordPosition] = InByte;
      WordPosition++;
      }
    else
      {
      Command[WordPosition] = '\0';
      Serial.print("input = ");
      Serial.println(Command);
      int Data0 = (Command[0]);
      int Data1 = (Command[1]-'0')*100;
      int Data2 = (Command[2]-'0')*10;
      int Data3 = (Command[3]-'0');
      int DataX;
      if(Data0 == 43)
        { 
        DataX = Data1+Data2+Data3;
        }
      else if (Data0 == 45)
        { 
        DataX = -1*(Data1+Data2+Data3);
        }
      Serial.print("X = ");
      Serial.println(DataX);
      int Data4 = (Command[4]);
      int Data5 = (Command[5]-'0')*100;
      int Data6 = (Command[6]-'0')*10;
      int Data7 = (Command[7]-'0');
      int DataY;
      if(Data4 == 43)
        { 
        DataY = Data5+Data6+Data7;
        }
      else if (Data4 == 45)
        { 
        DataY = -1*(Data5+Data6+Data7);
        }
      Serial.print("Y = ");
      Serial.println(DataY);
      int Data8 = (Command[8]);
      int Data9 = (Command[9]-'0')*100;
      int Data10 = (Command[10]-'0')*10;
      int Data11 = (Command[11]-'0');
      int DataZ;
      if(Data8 == 43)
        { 
        DataZ = Data9+Data10+Data11;
        }
      else if (Data8 == 45)
        { 
        DataZ = (-1)*(Data9+Data10+Data11);
        }
      Serial.print("Z = ");
      Serial.println(DataZ);
      int Deg;
      if(DataY <= 0)
        {
        Deg = -90;
        }
      else
        {
        Deg = 90;
        }
      boolean ErrH = 0;
      boolean ErrA = 0;
      ErrH = LimitHigh(DataZ);
      ErrA = LimitArm(DataX,DataY,Deg);
      if(ErrH || ErrA)
        {
        Serial.println("OutRange");
        delay(1000);
        }
      else
        {
        int BDataY = DataY+50;
        HighRunto(DataZ,SDSp,SDAc);
        IKArmRun(DataX,BDataY,Deg,SDSp,SDAc);
        for(int j = 0; j <= 50; j += 1)
          {
          int NDataY = BDataY - j;
          IKArmRun(DataX,NDataY,Deg,1000,30000);
          }
        for (pos = 500; pos <= 2000; pos += 20) 
          {
          servo1.writeMicroseconds(pos);
          delay(25);  
          }
        for (pos = 500; pos <= 2500; pos += 20) 
          {
          servo2.writeMicroseconds(pos);
          delay(25);  
          }
        for (pos = 2500; pos >= 500; pos -= 20) 
          {
          servo2.writeMicroseconds(pos);
          delay(25); 
          }
        for(int j = 0; j <= 50; j += 1)
          {
          int MDataY = DataY + j;
          IKArmRun(DataX,MDataY,Deg,1000,30000);
          }
        stepper2.setMaxSpeed(SDSp);
        stepper2.setAcceleration(SDAc);
        stepper3.setMaxSpeed(SDSp);
        stepper3.setAcceleration(SDAc);
        stepper4.setMaxSpeed(SDSp);
        stepper4.setAcceleration(SDAc);
        stepper2.moveTo(-2666);
        stepper3.moveTo(1770);
        stepper4.moveTo(1165);
        while (stepper2.distanceToGo()!=0 || stepper3.distanceToGo()!=0 || stepper4.distanceToGo()!=0)
          {
          stepper2.run();
          stepper3.run();
          stepper4.run();
          }  
        ArmRunToDeg(80,90,70,SDSp,SDAc);
        for (pos = 2000; pos >= 500; pos -= 20) 
          {
          servo1.writeMicroseconds(pos);
          delay(25);
          } 
        delay(1000);
        ArmRunToDeg(0,0,0,SDSp,SDAc);  
        Serial.println("StandBy");
        WordPosition = 0;
        }
      }
    }
  }


void set0()
  {
  stepper1.setMaxSpeed(2000);
  stepper1.setAcceleration(SDAc);
  stepper2.setMaxSpeed(500);
  stepper2.setAcceleration(SDAc);
  stepper3.setMaxSpeed(500);
  stepper3.setAcceleration(SDAc);
  stepper4.setMaxSpeed(500);
  stepper4.setAcceleration(SDAc);
  stepper1.moveTo(1000000);
  stepper2.moveTo(100000);
  stepper3.moveTo(-100000);
  stepper4.moveTo(100000);
  while(1)
    {
    stepper1.run();
    if(digitalRead(A0)==LOW)
      {
      stepper1.setCurrentPosition(0);
      delay(1000);
      break;
      }
    }
  while(1)
    {
    stepper2.run();
    if(digitalRead(A1)==LOW)
      {
      stepper2.setCurrentPosition(0);
      delay(1000);
      break;
      }
    }
  while(1)
    {
    stepper3.run();
    if(digitalRead(A2)==LOW)
      {
      stepper3.setCurrentPosition(0);
      delay(1000);
      break;
      }
    }
  while(1)
    {
    stepper4.run();
    if(digitalRead(A3)==LOW)
      {
      stepper4.setCurrentPosition(0);
      delay(1000);
      break;
      }
    }
  }
  
void ArmRunToDeg(float DEG1,float DEG2, float DEG3, int Speed, int Acceleration)
  {
  float DegToStepRate1, DegToStepRate2, DegToStepRate3, DegToStepRate4;
  float Step2, Step3, Step4;
  stepper2.setMaxSpeed(Speed);
  stepper2.setAcceleration(Acceleration);
  stepper3.setMaxSpeed(Speed);
  stepper3.setAcceleration(Acceleration);
  stepper4.setMaxSpeed(Speed);
  stepper4.setAcceleration(Acceleration);
  DegToStepRate1 = -33.3333333;   //5950step/180deg = 33.33333333step/1deg
  DegToStepRate2 = -18.1777777;   //3272step/180deg = 18.17777777step/1deg
  DegToStepRate3 = 35.8333333;  //6450step/180deg = 35.83333333333step/1deg
  DegToStepRate4 = 16.6444444;  //2996step/180deg = 16.64444444444step/1deg
  Step2 = DEG1*DegToStepRate1;
  Step3 = (DEG1*DegToStepRate2)+(DEG2*DegToStepRate3);
  Step4 = DEG3*DegToStepRate4;
  stepper2.moveTo(Step2);
  stepper3.moveTo(Step3);
  stepper4.moveTo(Step4);
  while (stepper2.distanceToGo()!=0 || stepper3.distanceToGo()!=0 || stepper4.distanceToGo()!=0)
    {
    stepper2.run();
    stepper3.run();
    stepper4.run();
    }
  }

void HighRunto (float Hig, int Speed, int Acceleration)
  {
  float HigToStepRate, Step1;
  stepper1.setMaxSpeed(Speed);
  stepper1.setAcceleration(Acceleration);
  HigToStepRate = (-1)*(379.81308411);   //10160step/26.75mm = 379.81308411step/1mm
  Step1 = Hig*HigToStepRate;
  stepper1.moveTo(Step1);
  while (stepper1.distanceToGo()!=0)
    {
    stepper1.run();
    }
  }

boolean LimitHigh(int Hig)
  {
  if (Hig < 0 || Hig > 360)
    {
    return 1;
    }
  else 
    {
    return 0;
    }
  }

void IKArmRun (int xi, int yi, int QA, int Speed, int Acceleration)
  {
  float L1,L2,L3,x2,y2,QR0,QR1,QR2,QR3,QA0,QA1,QA2,QA3,C1,C2,y3;    //C1 = Cos(QR1), C2 = Cos(QR2)
  L1 = 210;
  L2 = 150;
  L3 = 200;
  if (yi<=0)
    {
    y3 = 0-yi;
    QA0 = 0-QA;
    }
  else if(yi>=0)
    {
    y3 = yi;
    QA0 = QA;
    }
  QR0 = QA0*(M_PI/180);
  x2 = xi-(L3*cos(QR0));
  y2 = y3-(L3*sin(QR0));
  C2 = (((x2*x2)+(y2*y2)-(L1*L1)-(L2*L2))/(2*L1*L2));
  QR2 = acos(C2);
  C1 = (((L1+(L2*C2))*x2)+(L2*y2*sin(QR2)))/((x2*x2)+(y2*y2));
  QR1 = acos(C1);
  QR3 = QR0-(QR1+QR2);
  if (yi<=0)
    {
    QA1 = QR1*(-180/M_PI);
    QA2 = QR2*(180/M_PI);
    QA3 = QR3*(180/M_PI);
    }
  else if(yi>=0)
    {
    QA1 = QR1*(180/M_PI);
    QA2 = QR2*(-180/M_PI);
    QA3 = QR3*(-180/M_PI);
    }
  ArmRunToDeg(QA1,QA2,QA3,Speed,Acceleration);
  }

boolean LimitArm(int xi, int yi, int QA)
  {
  float L1,L2,L3,x2,y2,QR0,QR1,QR2,QR3,QA0,QA1,QA2,QA3,C1,C2,y3;    //C1 = Cos(QR1), C2 = Cos(QR2)
  L1 = 210;
  L2 = 150;
  L3 = 200;
  if (yi<=0)
    {
    y3 = 0-yi;
    QA0 = 0-QA;
    }
  else if(yi>=0)
    {
    y3 = yi;
    QA0 = QA;
    }
  QR0 = QA0*(M_PI/180);
  x2 = xi-(L3*cos(QR0));
  y2 = y3-(L3*sin(QR0));
  C2 = (((x2*x2)+(y2*y2)-(L1*L1)-(L2*L2))/(2*L1*L2));
  QR2 = acos(C2);
  C1 = (((L1+(L2*C2))*x2)+(L2*y2*sin(QR2)))/((x2*x2)+(y2*y2));
  QR1 = acos(C1);
  QR3 = QR0-(QR1+QR2);
  if (yi<=0)
    {
    QA1 = QR1*(-180/M_PI);
    QA2 = QR2*(180/M_PI);
    QA3 = QR3*(180/M_PI);
    }
  else if(yi>=0)
    {
    QA1 = QR1*(180/M_PI);
    QA2 = QR2*(-180/M_PI);
    QA3 = QR3*(-180/M_PI);
    }
  delay(100);
  if(isnan(QA1) || isnan(QA2) || isnan(QA1) || QA1 < -90 || QA2 > 110 || QA3 < -75 || QA1 > 90 || QA2 < -110 || QA3 > 120)
    {
    return 1;
    }
  else if(QA1 == QA1 && QA2 == QA2 && QA3 == QA3)
    {
    return 0;
    }
  }

//void CommandInputArmRun()
//  {
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
//        int Data1 = (Command[1]-'0')*100;
//        int Data2 = (Command[2]-'0')*10;
//        int Data3 = (Command[3]-'0');
//        int DataX;
//        if(Data0 == 43)
//          { 
//          DataX = Data1+Data2+Data3;
//          }
//        else if (Data0 == 45)
//          { 
//          DataX = -1*(Data1+Data2+Data3);
//          }
//        Serial.print("X = ");
//        Serial.println(DataX);
//        int Data4 = (Command[4]);
//        int Data5 = (Command[5]-'0')*100;
//        int Data6 = (Command[6]-'0')*10;
//        int Data7 = (Command[7]-'0');
//        int DataY;
//        if(Data4 == 43)
//          { 
//          DataY = Data5+Data6+Data7;
//          }
//        else if (Data4 == 45)
//          { 
//          DataY = -1*(Data5+Data6+Data7);
//          }
//        Serial.print("Y = ");
//        Serial.println(DataY);
//        int Data8 = (Command[8]);
//        int Data9 = (Command[9]-'0')*100;
//        int Data10 = (Command[10]-'0')*10;
//        int Data11 = (Command[11]-'0');
//        int DataZ;
//        if(Data8 == 43)
//          { 
//          DataZ = Data9+Data10+Data11;
//          }
//        else if (Data8 == 45)
//          { 
//          DataZ = (-1)*(Data9+Data10+Data11);
//          }
//        Serial.print("Z = ");
//        Serial.println(DataZ);
//        int Deg;
//        if(DataY <= 0)
//          {
//          Deg = -90;
//          }
//        else
//          {
//          Deg = 90;
//          }
//        int BDataY = DataY+50;
//        IKArmRun(DataZ,DataX,BDataY,Deg,SDSp,SDAc);
//        for(int j = 0; j <= 50; j += 1)
//          {
//          int NDataY = BDataY - j;
//          IKArmRun(DataZ,DataX,NDataY,Deg,1000,30000);
//          }
//        delay(100);
//        EndCommand = 1;
//        WordPosition = 0;
//        }
//      }
//    if(EndCommand == 1)
//      {
//      break;
//      }
//    }
//  }
