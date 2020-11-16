#include <Arduino.h>
#include <Servo.h>
//#include <LSM6.h>
//#include <Adafruit_LIS3MDL.h>

const int LASER_PIN = 8;
const int SERVO_PIN = 9;
const int MOTOR_PWM_1 = 5;
const int MOTOR_PWM_2 = 6;

Servo laserServo;

void setup() 
{
  //Serial.begin(9600);
  pinMode(LASER_PIN,OUTPUT);
  pinMode(MOTOR_PWM_1, OUTPUT);
  pinMode(MOTOR_PWM_2, OUTPUT);
  digitalWrite(LASER_PIN, HIGH);
  laserServo.attach(SERVO_PIN);
}

void loop() 
{
  laserServo.write(165);
  analogWrite(MOTOR_PWM_1, 0);
  analogWrite(MOTOR_PWM_2, 40);
}