//#include <L298NX2.h>
#include <SparkFun_TB6612.h>
#include <Wire.h>
#include <PID_v1.h>
#include "JY901_Serial.h"

// ---------------    Parameters   ---------------

const double BLOCK_SIZE = 50.0;          // Length of one grid square (in centimeters)
const double ENCODER_PER_CENTIMETER = = 60.8475;          // Encoder pulses per 1 cm
const double MOTOR_SPEED = 100.0;         // Base speed of both motors
const double MOTOR_SPEED_MULTIPLIER = 1.13;         // In case one motor is slower than the other, only applies to the left motor

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
boolean encoderLeftDirection;          // Tracks the direction of the left motor (TRUE is foward)
long encoderPulsesLeft;         // Tracks the amount of encoder pulses in the left motor
// Right Encoder
boolean encoderRightDirection;         // Tracks the direction of the right motor (TRUE is foward)
long encoderPulsesLeft;         // Tracks the amount of encoder pulses in the left motor

// ---------------       PID       ---------------


// ~~~~~~~~~~~~~~~ TODO: Add PID variables ~~~~~~~~~~~~~~~


// ---------------  Miscellaneous  ---------------

const long PRINT_DEBUG_COOLDOWN = 100;          // Time between printing debug information

int buttonState;          // Variable for reading the start button status
int lastButtonState;          // Variable for reading the previous start button status

double heading = 0.0;  // The heading of the compass

bool beginPath = false;         // If set to TRUE, the set movement primitives will run

unsigned long previousMillis = millis();          // Variable that holds the previous timestamp

void fw(double blocks = 1.0);         // Instantiate primitives to allow for 
void bw(double blocks = 1.0);         // a default parameter of 1 block

// -----------------------------------------------



// Main

void setup(){

  // Start debug serial
  Serial.begin(115200);
  // Start compass serial
  Serial1.begin(9600);

  // Initialize start button pin as an input
  pinMode(BUTTON_PIN, INPUT);

  // Initialize encoders
  encoderPulsesLeft = 0;
  encoderPulsesRight = 0;
  encodersInit();
  
  // Initialize PID

  // Initialize compass
  JY901.attach(Serial1);

  
}

void loop(){

  // Read the state of the start button
  buttonState = digitalRead(BUTTON_PIN); 

  // Determine if the start button has been press (HIGH means the button is being pressed)
  if (buttonState == HIGH && lastButtonState == LOW){

    // Give time to take finger off of the button
    delay(2000); 

    // Begin the path
    beginPath = true;

  }
  // Save the start button state for the next loop
  lastButtonState = buttonState;

  if (beginPath){
    Serial.print("##### RUNNING ######");

    // Move into the first square
    moveDistance(20.0);

    // ---------------  Create Path Here  ---------------



    // --------------------------------------------------

    // Move dowel to the ending location
    moveDistance(5.0);

    // Stop running the path
    beginPath = false;
  }
}

// Primitives

void fw(double blocks){
  moveDistance(blocks * BLOCK_SIZE);
}

void bw(double blocks){
  moveDistance(-blocks * BLOCK_SIZE);
}

void left(){
  turnDegrees(-90.0);
}

void right(){
  turnDegrees(90.0);
}

// Movement Functions

void moveDistance(double distance){

  // Set encoders
  encoderEnd = abs(distance * ENCODER_PER_CENTIMETER);
  encoderPulsesLeft = 0;
  encoderPulsesRight = 0;

  // Determine direction of target distance
  int direction;
  if (distance >= 0){
    direction = 1; // Target is ahead
  } else {
    direction = -1; // Target is behind
  }

  // Tell motors to drive in the direction of the target distance
  motorLeft.drive(MOTOR_SPEED * MOTOR_SPEED_MULTIPLIER * direction);
  motorRight.drive(MOTOR_SPEED * direction);

  while (abs(encoderPulsesLeft) < encoderEnd && abs(encoderPulsesRight) < encoderEnd){ // Wait for the encoders to count to the target pulse count
    // Wait

    // ~~~~~~~~~~~~~~~ TODO: Fix the PID / keep the robot moving straight without relying on the initial angle of the robot ~~~~~~~~~~~~~~~

  }

  // Stop moving after target distance is reached
  hitBrakes();

}

void turnDegrees(double degrees){

  // Determine direction of target angle
  int direction;
  if (degrees >= 0){
    direction = 1; // Turn to the right
  } else {
    direction = -1; // Turn to the left
  }

  // Tell motors to drive according to the direction of the angle
  motorLeft.drive(MOTOR_SPEED * MOTOR_SPEED_MULTIPLIER * direction);
  motorRight.drive(MOTOR_SPEED * -direction); // Moves the opposite direction in order to turn

  // ~~~~~~~~~~~~~~~ TODO: Create a reliable way to turn a certain amount of degrees ~~~~~~~~~~~~~~~
  
  delay(2000); // Wait two seconds (temporary)

  // Stop moving after reaching target angle
  hitBrakes();
}

void hitBrakes(){

  brake(motorLeft,motorRight);

}

// Encoder Functions

void encodersInit() {
  encoderLeftDirection = true;                           // TRUE -> Forward
  encoderRightDirection = true;                           // TRUE -> Forward

  pinMode(ENCODER_LEFT_PIN_B,INPUT);
  pinMode(ENCODER_RIGHT_PIN_B,INPUT);

  attachInterrupt(2, encoderLeftCounter, CHANGE);
  attachInterrupt(3, encoderRightCounter, CHANGE);
}

void encoderLeftCounter() {
  int leftStateA = digitalRead(ENCODER_LEFT_PIN_A);
  if(encoderLeftLastState == LOW && leftStateA == HIGH)
  {
    int leftStateB = digitalRead(ENCODER_LEFT_PIN_B);

    // Check for change in direction
    if (leftStateB == LOW && encoderLeftDirection)
    {
      encoderLeftDirection = false; // Backward
    }
    else if (leftStateB == HIGH && !encoderLeftDirection)
    {
      encoderLeftDirection = true; // Forward
    }
  }
  encoderLeftLastState = leftStateA;

  if(encoderLeftDirection)  encoderPulsesLeft++; // NOTE: This is opposite of the right encoder since they are inverted
  else  encoderPulsesLeft--;
}

void encoderRightCounter() {
  int rightStateA = digitalRead(ENCODER_RIGHT_PIN_A);
  if(encoderRightLastState == LOW && rightStateA == HIGH)
  {
    int rightStateB = digitalRead(ENCODER_RIGHT_PIN_B);

    // Check for change in direction
    if (rightStateB == LOW && encoderRightDirection)
    {
      encoderRightDirection = false; // Backward
    }
    else if (rightStateB == HIGH && !encoderRightDirection)
    {
      encoderRightDirection = true; // Forward
    }
  }
  encoderRightLastState = rightStateA;

  if(encoderRightDirection)  encoderPulsesRight++;
  else  encoderPulsesRight--;
}
