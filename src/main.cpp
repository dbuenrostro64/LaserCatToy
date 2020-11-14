#include <Arduino.h>
#include <Servo.h>
#include <LSM6.h>
#include <Adafruit_LIS3MDL.h>

const int LASER_PIN = 8;
const int SERVO_PIN = 9;
const int MOTOR_PWM_1 = 10;
const int MOTOR_PWM_2 = 11;

Servo laserServo;

void setup() 
{
  pinMode(LASER_PIN,OUTPUT);
  laserServo.attach(SERVO_PIN);
}

void loop() 
{
  laserServo.write(0);
  delay(500);
  laserServo.write(180);
  delay(500);
}