#include <Arduino.h>
#include <String.h>
#include <Servo.h>
#include <LSM6.h>
#include <Adafruit_LIS3MDL.h>

const int BUTTON_PIN = 2;
const int RED_LED_PIN = 10;
const int GREEN_LED_PIN = 11;

const int LASER_PIN = 8;
const int SERVO_PIN = 9;
const int MOTOR_PWM_1 = 5;
const int MOTOR_PWM_2 = 6;

unsigned long startMillisSpin;
unsigned long currentMillisSpin;
const unsigned long SPIN_PERIOD = 3000;
unsigned long startMillisServo;
unsigned long currentMillisServo;
const unsigned long SERVO_PERIOD = 1500;
String spinDirection = "cw";
int servoAngle = 125;
int spinSpeed = 50;
int buttonState = 0;
volatile byte buttonMode = LOW;

int lastButtonState = 0;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
unsigned long timeHeld = 0;
int holdButtonTime = 3000;

char report[80];

Servo laserServo;
Adafruit_LIS3MDL magnet;
LSM6 gyroAccel;

String pressButton(void);
void buttonDirectionChange(String);
void autoDirectionChange(int);
void autoServoChange(int);
void readMagnetometer(void);
void readGyroAndAccel(void);
void mode();

void setup() 
{
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(LASER_PIN,OUTPUT);
  pinMode(MOTOR_PWM_1, OUTPUT);
  pinMode(MOTOR_PWM_2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), mode, FALLING); // run mode routine whenever pin goes high to low
  digitalWrite(LASER_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, HIGH); // red LED on as long as device is on
  laserServo.attach(SERVO_PIN);

  //-----IMU initialize-----

  // magnetometer
  // if (! magnet.begin_I2C()) 
  // {          // hardware I2C mode, can pass in address & alt Wire
  //   Serial.println("Failed to find LIS3MDL chip");
  //   while (1) { delay(10); }
  // }
  // magnet.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  // magnet.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  // magnet.setDataRate(LIS3MDL_DATARATE_155_HZ);
  // magnet.setRange(LIS3MDL_RANGE_4_GAUSS);
  // magnet.setIntThreshold(500);
  // magnet.configInterrupt(false, false, true, // enable z axis
  //                         true, // polarity
  //                         false, // don't latch
  //                         true); // enabled!

  //gyroscope and accelerometer
  // if (!gyroAccel.init())
  // {
  //   Serial.println("Failed to detect and initialize IMU!");
  //   while (1);
  // }
  // gyroAccel.enableDefault();

}

void loop() 
{
  spinDirection = pressButton();
  buttonDirectionChange(spinDirection);
  autoServoChange(SERVO_PERIOD);
}

String pressButton(void)
{
  static String spinDirection = "a";
  static bool buttonPressed = 0;
  buttonState = digitalRead(BUTTON_PIN);
  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    if(buttonState == LOW && buttonPressed == 0)
    {
      if (spinDirection == "a")
      {spinDirection = "b";}
      else if (spinDirection == "b")
      {spinDirection = "a";}
      buttonPressed = 1;
      lastDebounceTime = millis();
    }
    if(buttonState == HIGH)
      {
        buttonPressed = 0;
        lastDebounceTime = millis();
      }
  }
  //Serial.print(spinDirection);
  return spinDirection;
}

void buttonDirectionChange(String spinDirection)
{
  if(spinDirection == "a")
  {
    analogWrite(MOTOR_PWM_1, spinSpeed);
    analogWrite(MOTOR_PWM_2, 0);
  }
  else if(spinDirection == "b")
  {
    analogWrite(MOTOR_PWM_1, 0);
    analogWrite(MOTOR_PWM_2, spinSpeed);
  }
}

void autoDirectionChange(int interval)
{
  currentMillisSpin = millis();
    if(currentMillisSpin - startMillisSpin >= interval)
    {
      if(spinDirection=="cw")
      {
        
        spinDirection = "ccw";
        analogWrite(MOTOR_PWM_1, 0);
        analogWrite(MOTOR_PWM_2, spinSpeed);
      }
      else if(spinDirection=="ccw")
      {
        spinDirection = "cw";
        analogWrite(MOTOR_PWM_1, spinSpeed);
        analogWrite(MOTOR_PWM_2, 0);
      }
      startMillisSpin = currentMillisSpin;
    }
}

void autoServoChange(int interval)
{
  currentMillisServo = millis();
  if(currentMillisServo - startMillisServo >= interval)
  {
    if (servoAngle==125)
    {
      servoAngle=95;
      laserServo.write(servoAngle);
    }
    else if (servoAngle==95)
    {
      servoAngle=125;
      laserServo.write(servoAngle);
    }
    startMillisServo = currentMillisServo;
  }
}

void readMagnetometer(void)
{
  magnet.read();      // get X Y and Z data at once
  // Then print out the raw data
  Serial.print("\nX:  "); Serial.print(magnet.x); 
  Serial.print("  \tY:  "); Serial.print(magnet.y); 
  Serial.print("  \tZ:  "); Serial.println(magnet.z);
}

void readGyroAndAccel(void)
{
  gyroAccel.read();
  // print data
  snprintf(report, sizeof(report), "A: %6d %6d %6d    G: %6d %6d %6d",
  gyroAccel.a.x, gyroAccel.a.y, gyroAccel.a.z,
  gyroAccel.g.x, gyroAccel.g.y, gyroAccel.g.z);
  Serial.println(report);
}

void mode()
{
  buttonMode = !buttonMode;
}