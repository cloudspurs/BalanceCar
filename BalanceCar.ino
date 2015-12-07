#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

/********************************************
  mengqinggang
********************************************/
int Pwm;
int PwmLeft, PwmRight;

// L298N
const int ENA = 9;
const int ENB = 11;
const int IN1 = 7;
const int IN2 = 8;
const int IN3 = 10;
const int IN4 = 12;

// Encoder 
const int Encoder0_PinA = 0;
const int Encoder0_PinB = 3;
const int Encoder1_PinA = 1;
const int Encoder1_PinB = 2;
const int LEFT = 0;
const int RIGHT = 1;
double LeftSpeed = 0;
double RightSpeed= 0;
double Speed;
double SpeedLast;
double Distance;
const int SpeedControl = 50;
const int MaxDistance = 5000;

// dead speed
const int MDS = 0;
int TempSpeed;

// delay time
const int t = 12;

// PID Parameters
const double K1 = 60; // 60 
const double K2 = 1.8;    // 3
const double K3 = 15; 
const double K4 = 0.015;

MPU6050 AccelGyro;             // 陀螺仪
double AngleAy, GyroGy;        // 加速度计算的角度（与x轴夹角）和x轴角速度
int16_t AX, AY, AZ, GX, GY, GZ; // 3个加速度和3个角速度

const double AngleBalance = -1.3;

// kalman para
double Angle, AngleDot;    // 卡尔曼滤波后的角度和角速度
const double dt = 0.02;          // 卡尔曼滤波采样时间
double P[2][2] = {{1, 0}, {0, 1}};
double Pdot[4] = {0, 0, 0, 0};
double Q_angle = 0.001, Q_gyro = 0.005;  // 角度(AngleAx)置信度，角速度(GyroGx)置信度
double R_angle = 0.5, C_0 = 1;
double q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
/********************************************************
  mengqinggang
********************************************************/


/*******************************************************
  linlei
********************************************************/

/*******************************************************
  linlei
********************************************************/


/*******************************************************
  suwenyuan
********************************************************/

/*******************************************************
  suwenyuan
********************************************************/



void setup() {
  Wire.begin();
  Serial.begin(9600);
  AccelGyro.initialize();
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  pinMode(Encoder0_PinA, INPUT);
  pinMode(Encoder0_PinB, INPUT);
  pinMode(Encoder1_PinA, INPUT);
  pinMode(Encoder1_PinB, INPUT);
  digitalWrite(Encoder0_PinA, HIGH);
  digitalWrite(Encoder0_PinB, HIGH);
  digitalWrite(Encoder1_PinA, HIGH);
  digitalWrite(Encoder1_PinB, HIGH);
  
  attachInterrupt(LEFT, LeftWheelSpeed, FALLING);
  attachInterrupt(RIGHT, RightWheelSpeed, FALLING);
  
/*******************************************************
  linlei
********************************************************/

/*******************************************************
  linlei
********************************************************/


/*******************************************************
  suwenyuan
********************************************************/

/*******************************************************
  suwenyuan
********************************************************/

  
}

void loop() {
  //Serial.println(micros()/1000);
  Protect();
  CalAngle();
  CalSpeed();
  CalPwm();
  OutPwm();
  
  printout();

/*******************************************************
  linlei
********************************************************/

/*******************************************************
  linlei
********************************************************/
  
  
/*******************************************************
  suwenyuan
********************************************************/

/*******************************************************
  suwenyuan
********************************************************/
  
  delay(t);
}

void Drive() {
  char drive;
  if (drive == 'w') 
    Distance += SpeedControl;
  if (drive == 's')
    Distance -= SpeedControl;
  if (Distance > MaxDistance)
    Distance = MaxDistance;
  if (Distance < -MaxDistance)
    Distance = -MaxDistance;
}

void CalAngle() {
  AccelGyro.getMotion6(&AX, &AY, &AZ, &GX, &GY, &GZ);
  AngleAy = atan2(AX, AZ)*180/PI;
  GyroGy = -GY/131.00;
  KalmanFilter(AngleAy, GyroGy);
}

void CalSpeed() {
  SpeedLast = (LeftSpeed + RightSpeed) - 0;
  Speed *= 0.7;
  Speed += SpeedLast * 0.3;
  Distance += Speed;
  LeftSpeed = RightSpeed = 0;
}

void Protect() {
  if (abs(Angle) >  10 )
  Pwm = 0;
}

void CalPwm() {
  if (Angle > 0)
    TempSpeed = MDS;
  else
    TempSpeed = -MDS;
  
  Pwm = K1 * (Angle + AngleBalance) + K2 * AngleDot + K3 * Speed + K4 * Distance + TempSpeed;
  
  if (Pwm > 255)
    Pwm = 255;
  if (Pwm < -255)
    Pwm = -255;
}

void OutPwm() {
  if (Pwm > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else if (Pwm < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  
  PwmLeft = abs(Pwm);
  PwmRight = abs(Pwm);

  analogWrite(ENB, PwmRight);
  analogWrite(ENA, PwmLeft);
}

void printout() {
  double temp = Angle+AngleBalance;
  Serial.print(temp);
  Serial.print("    ");
  Serial.print(AngleDot);
  Serial.print("    ");
  Serial.print(Pwm);
  Serial.print("    ");
  Serial.print(Speed);
  Serial.print("    ");
  Serial.println(Distance);
}

void KalmanFilter(double angle_m, double gyro_m)
{
  Angle += (gyro_m - q_bias) * dt;
  angle_err = angle_m - Angle;
  Pdot[0] = Q_angle - P[0][1] - P[1][0];
  Pdot[1] = - P[1][1];
  Pdot[2] = - P[1][1];
  Pdot[3] = Q_gyro;
  P[0][0] += Pdot[0] * dt;
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
  E = R_angle + C_0 * PCt_0;
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  t_0 = PCt_0;
  t_1 = C_0 * P[0][1];
  P[0][0] -= K_0 * t_0;
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
  Angle += K_0 * angle_err;     // 最优角度
  q_bias += K_1 * angle_err;
  AngleDot = gyro_m - q_bias;   // 最优角速度
}

void LeftWheelSpeed() {
  if(digitalRead(Encoder0_PinB))
    LeftSpeed++;
  else
    LeftSpeed--;
}

void RightWheelSpeed() {
  if(digitalRead(Encoder1_PinB))
    RightSpeed--;
  else
    RightSpeed++;
}

/*******************************************************
  linlei
********************************************************/

/*******************************************************
  linlei
********************************************************/


/*******************************************************
  suwenyuan
********************************************************/

/*******************************************************
  suwenyuan
********************************************************/



