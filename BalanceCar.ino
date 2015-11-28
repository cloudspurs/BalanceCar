#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

const int L298N_ENA = 5;
const int L298N_IN1 = 8;
const int L298N_IN2 = 7;
const int L298N_EncoderLA = 4;
const int L298N_EncoderLB = 2;
int L298N_EncoderLPos = 0;
boolean L298N_EncoderLLast = LOW;

MPU6050 Gyroscope;

void setup() {
  Wire.begin();
  Gyroscope.begin();
  pinMode(L298N_IN1, OUTPUT);
  pinMode(L298N_IN2, OUTPUT);
  pinMode(L298N_EncoderLA, INPUT);
  pinMode(L298N_EncoderLB, INPUT);
  digitalWrite(L298N_EncoderLA, HIGH);
  digitalWrite(L298N_EncoderLB, HIGH);

  Serial.begin(9600);
}

void loop() {
  boolean L298N_EncoderL = digitalRead(L298N_EncoderLA);
  if ((L298N_EncoderLLast == HIGH) && (L298N_EncoderL == LOW)) {
    if (digitalRead(L298N_EncoderLB) == LOW)
      L298N_EncoderLPos--;
    else
      L298N_EncoderLPos++;
  }
  Serial.println(L298N_EncoderLPos);
  L298N_EncoderLLast = L298N_EncoderL;

  double CarAngle = asin(Gyroscope.getAccelY()) * 180 / PI;
  int speed = map(abs(CarAngle), 0, 90, 100, 255);
  analogWrite(L298N_ENA, speed);
  if (CarAngle >= 0) {
    digitalWrite(L298N_IN1, LOW);
    digitalWrite(L298N_IN2, HIGH);
  }
  else {
    digitalWrite(L298N_IN1, HIGH);
    digitalWrite(L298N_IN2, LOW);
  }
}

