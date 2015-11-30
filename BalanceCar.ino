#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

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
const int Encoder0_PinA = 2;
const int Encoder0_PinB = 3;
const int Encoder1_PinA = 4;
const int Encoder1_PinB = 5;
int LeftSpeed = 0;
int RightSpeed= 0;

boolean Encoder0_ALast = LOW;
boolean Encoder1_ALast = LOW;

// PID Parameters
double K1, K2, K3, K4;

MPU6050 AccelGyro;             // 陀螺仪
double AngleAx, GyroGx;        // 加速度计算的角度（与x轴夹角）和x轴角速度
int16_t AX, AY, AZ, GX, GY, GZ; // 3个加速度和3个角速度

// kalman para
double Angle, AngleDot;    // 卡尔曼滤波后的角度和角速度
double dt = 0.02;          // 卡尔曼滤波采样时间
double P[2][2] = {{1, 0}, {0, 1}};
double Pdot[4] = {0, 0, 0, 0};
double Q_angle = 0.001, Q_gyro = 0.005;  // 角度(AngleAx)置信度，角速度(GyroGx)置信度
double R_angle = 0.5, C_0 = 1;
double q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  AccelGyro.initialize();
  
  K1 = 12;
  K2 = 0.5;
  
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
}

void loop() {
  LeftWheelSpeed();
  CalAngle();
  CalPwm();
  OutPwm();
  
  printout();
  
  delay(5);
}

void printout()
{
  Serial.print(AngleAx);
  Serial.print("    ");
  Serial.print(Angle);
  Serial.print("    ");
  Serial.print(AngleDot);
  Serial.print("    ");
  Serial.print(Pwm);
  Serial.print("    ");
  Serial.println(LeftSpeed);
}

void LeftWheelSpeed() {
   boolean Encoder0_A = digitalRead(Encoder0_PinA);
   if ((Encoder0_ALast == HIGH) && (Encoder0_A == LOW)) {
      if (digitalRead(Encoder0_PinB) == LOW)
        LeftSpeed--;
      else
        LeftSpeed++;
   }
   
   Encoder0_ALast = Encoder0_A; 
}

void CalPwm() {
  
  Pwm = K1 * Angle + K2 *AngleDot;
  
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

void CalAngle() {
  AccelGyro.getMotion6(&AX, &AY, &AZ, &GX, &GY, &GZ);
  AngleAx = atan2(AY, AZ)*180/PI;
  GyroGx = GX/131.00;
  KalmanFilter(AngleAx, GyroGx);
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


