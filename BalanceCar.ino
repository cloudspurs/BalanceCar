#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

const int EnAPin = 5;
const int in1Pin = 4;
const int in2Pin = 7;

MPU6050 Gyroscope;

void setup() {
  Wire.begin();
  Gyroscope.begin();
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  double CarAngle = asin(Gyroscope.getAccelX())*180/PI;
  Serial.println(CarAngle);
  
  int speed = map(CarAngle, 0, 90, 0, 255);
  analogWrite(EnAPin, speed);
  if (CarAngle >= 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    }
    
  delay(1000);
}

