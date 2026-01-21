//#include <L298NX2.h>
#include <SparkFun_TB6612.h>
#include <Wire.h>
#include <PID_v1.h>
#include "JY901_Serial.h"

// ---------------    Parameters   ---------------

const double BLOCK_SIZE = 50.0;          // length of one grid square (in centimeters)
const double ENCODER_PER_CENTIMETER = 0.0;          // encoder pulses per 1 cm
const double MOTOR_SPEED = 100.0;         // Base speed of both motors
const double MOTOR_SPEED_MULTIPLIER = 1.13;         // In case one motor is slower than the other

// ---------------   Arduino Pins   ---------------

// Start Button
const int BUTTON_PIN = 11;

// Motor Controller
const int STBY = 5;
// Left Motor
const int PWMA = 2;
const int AIN2 = 3;
const int AIN1 = 4;
// Right Motor
const int BIN1 = 6;
const int BIN2 = 7;
const int PWMB = 8;

// Left Encoder
const int ENCODER_LEFT_PIN_A = 21;
const int ENCODER_LEFT_PIN_B = 9;
// Right Encoder
const int ENCODER_RIGHT_PIN_A = 20;
const int ENCODER_RIGHT_PIN_B = 10;

// ---------------   Motor Setup   ---------------

// Motor Controller
Motor motorLeft(AIN1, AIN2, PWMA, 1, STBY);
Motor motorRight(BIN1, BIN2, PWMB, 1, STBY);

// Encoders
long encoderEnd; // Variable to hold the amount of encoder pulses to reach the target
// Left Encoder
boolean encoderLeftDirection;          // tracks the direction of the left motor (TRUE is foward)
long encoderPulsesLeft;         // tracks the amount of encoder pulses in the left motor
// Right Encoder
boolean encoderRightDirection;         // tracks the direction of the right motor (TRUE is foward)
long encoderPulsesLeft;         // tracks the amount of encoder pulses in the left motor

// ---------------  Miscellaneous  ---------------

const long PRINT_DEBUG_COOLDOWN = 100;          // Time between printing debug information

int buttonState;          // Variable for reading the start button status
int lastButtonState;          // Variable for reading the previous start button status

double heading = 0.0;

bool beginPath = false;         // If set to TRUE, the set movement primitives will run

unsigned long previousMillis = millis();          // Variable that holds the previous timestamp

void setup(){
  Serial.begin(9600);

  
}

void loop(){
  
}


void EncodersInit() {
  encoderLeftDirection = true;                           // true -> Forward
  encoderRightDirection = true;                           // true -> Forward

  pinMode(ENCODER_LEFT_PIN_B,INPUT);
  pinMode(ENCODER_RIGHT_PIN_B,INPUT);

  attachInterrupt(2, EncoderLeftCounter, CHANGE);
  attachInterrupt(3, EncoderRightCounter, CHANGE);
}

void EncoderLeftCounter() {
  int leftStateA = digitalRead(ENCODER_LEFT_PIN_A);
  if(encoderLeftLastState == LOW && leftStateA == HIGH)
  {
    int leftStateB = digitalRead(ENCODER_LEFT_PIN_B);

    // Check for change in direction
    if (leftStateB == LOW && encoderLeftDirection)
    {
      encoderLeftDirection = false;                      // Backward
    }
    else if (leftStateB == HIGH && !encoderLeftDirection)
    {
      encoderLeftDirection = true;                       // Forward
    }
  }
  encoderLeftLastState = leftStateA;

  if(encoderLeftDirection)  encoderPulsesLeft++;                // NOTE: This is opposite of the right encoder since they are inverted
  else  encoderPulsesLeft--;
}

void EncoderRightCounter() {
  int rightStateA = digitalRead(ENCODER_RIGHT_PIN_A);
  if(encoderRightLastState == LOW && rightStateA == HIGH)
  {
    int rightStateB = digitalRead(ENCODER_RIGHT_PIN_B);

    // Check for change in direction
    if (rightStateB == LOW && encoderRightDirection)
    {
      encoderRightDirection = false;                      // Backward
    }
    else if (rightStateB == HIGH && !encoderRightDirection)
    {
      encoderRightDirection = true;                       // Forward
    }
  }
  encoderRightLastState = rightStateA;

  if(encoderRightDirection)  encoderPulsesRight++;                // NOTE: This is opposite of the right encoder since they are inverted
  else  encoderPulsesRight--;
}


void hit_brakes(){

  brake(motorLeft,motorRight);

}
