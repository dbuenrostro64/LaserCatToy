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
String spinDirection = "a";
//servo angles
//0 = servo points up
//180 = servo points down
int servoAngle = 125;
// 25 is minimum spin speed
int spinSpeed = 25;
int spinSpeed1 = 25;
int spinSpeed2 = 40;
int spinSpeed3 = 60;
int buttonState = 0;
int buttonPos = 0;
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
int speedChangeButton(void);
void oneDirectionSpin(int);
void buttonDirectionChange(String, int);
void autoDirectionChange(int, int);
void autoServoChange(int);
void readMagnetometer(void);
void readGyroAndAccel(void);
void mode();
void ledSpeedStatus(int);
void drawSquare(void);

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
  digitalWrite(LASER_PIN, HIGH);
  digitalWrite(RED_LED_PIN, HIGH); // red LED on as long as device is on
  laserServo.attach(SERVO_PIN);
  laserServo.write(servoAngle);

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
  drawSquare();
  //spinDirection = pressButton();
  //spinSpeed = speedChangeButton();
  //oneDirectionSpin(spinSpeed);
  //ledSpeedStatus(spinSpeed);
  //buttonDirectionChange(spinDirection, spinSpeed);
  //autoServoChange(SERVO_PERIOD);
}

//---------------ROBOT CONTROL FUNCTIONS---------------//

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

int speedChangeButton(void)
{
  static int speed = 25;
  static bool buttonPressed = 0;
  buttonState = digitalRead(BUTTON_PIN);
  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    if(buttonState == LOW && buttonPressed == 0)
    {
      if(buttonPos == 2)
        {buttonPos = 0;}
      else
        {buttonPos++;}
      if(buttonPos == 0)
        {speed = spinSpeed1;}
      else if(buttonPos == 1)
        {speed = spinSpeed2;}
      else if(buttonPos == 2)
        {speed = spinSpeed3;}
      buttonPressed = 1;
      lastDebounceTime = millis();
    }
    if(buttonState == HIGH)
      {
        buttonPressed = 0;
        lastDebounceTime = millis();
      }
  }
  return speed;
}

void oneDirectionSpin(int speed)
{
  analogWrite(MOTOR_PWM_1, speed);
  analogWrite(MOTOR_PWM_2, 0);
}

void buttonDirectionChange(String spinDirection, int speed)
{
  if(spinDirection == "a")
  {
    analogWrite(MOTOR_PWM_1, speed);
    analogWrite(MOTOR_PWM_2, 0);
  }
  else if(spinDirection == "b")
  {
    analogWrite(MOTOR_PWM_1, 0);
    analogWrite(MOTOR_PWM_2, speed);
  }
}

void autoDirectionChange(int interval, int speed)
{
  currentMillisSpin = millis();
    if(currentMillisSpin - startMillisSpin >= interval)
    {
      if(spinDirection=="a")
      {
        
        spinDirection = "b";
        analogWrite(MOTOR_PWM_1, 0);
        analogWrite(MOTOR_PWM_2, speed);
      }
      else if(spinDirection=="b")
      {
        spinDirection = "a";
        analogWrite(MOTOR_PWM_1, speed);
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

void mode()
{
  buttonMode = !buttonMode;
}

void ledSpeedStatus(int speed)
{
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  const long interval = 500;
  static int ledState = LOW;

  if (speed == spinSpeed1)
    {digitalWrite(GREEN_LED_PIN, LOW);}
  else if (speed == spinSpeed2)
    {digitalWrite(GREEN_LED_PIN, HIGH);}
  else if (speed == spinSpeed3)
    {
      Serial.println(currentMillis);
      if (currentMillis - previousMillis >= interval)
      {
        previousMillis = currentMillis;
        if (ledState == LOW)
        {ledState = HIGH;}
        else
        {ledState = LOW;}
        digitalWrite(GREEN_LED_PIN, ledState);
      }
    }
}

//---------------IMU FUNCTIONS-------------------//

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


//--------------LASER ROUTINES----------------//

//laser pointer draws shape of square on wall
void drawSquare(void)
{
  for(int angle1 = 65; angle1 < 90; angle1+= 1)
    {
      laserServo.write(angle1);
      delay(50);
    }
  analogWrite(MOTOR_PWM_1, 0);
  analogWrite(MOTOR_PWM_2, 25);
  delay(250);
  analogWrite(MOTOR_PWM_1, 0);
  analogWrite(MOTOR_PWM_2, 0);
  for(int angle2 = 90; angle2 > 65; angle2 -= 1)
    {
      laserServo.write(angle2);
      delay(50);
    }
  analogWrite(MOTOR_PWM_1, 25);
  analogWrite(MOTOR_PWM_2, 0);
  delay(250);
  analogWrite(MOTOR_PWM_1, 0);
  analogWrite(MOTOR_PWM_2, 0);
}