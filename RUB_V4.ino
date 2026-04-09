#include <SparkFun_TB6612.h>
#include <Wire.h>
#include <PID_v1.h>
#include "JY901.h"
// ---------------    Parameters   ---------------

// ~~~~~~~~~
const double TARGET_TIME = 1000.0;          // How much time should it take to travel the whole path (in milliseconds)
const char PATH[] =
"f3"
"b3"
"r3"
;
// ~~~~~~~~~~

// PARAMETERS OPTIMIZED FOR: ≈11.20 V (across both battery packs)

const double BLOCK_SIZE = 50.0;          // Length of one grid square (in centimeters)
const double ENCODER_PER_CENTIMETER = 38.60784314;          // Encoder pulses per 1 cm

const double MOTOR_SPEED_MIN = 50.0;         // Starting and ending speed of both motors
const double MOTOR_SPEED_MAX = 220.0;         // Maximum speed of both motors

const double FW_MOTOR_SPEED_MULTIPLIER = 1.02;         // In case one motor is slower than the other, only applies to the left motor when moving fowards
const double BW_MOTOR_SPEED_MULTIPLIER = 1.008;         // In case one motor is slower than the other, only applies to the left motor when moving backwards

const double MOTOR_SPEED = 100.0;         // Base speed of both motors

const double TURN_TOLERANCE = 5.0;          // The maximum difference between the target heading and heading before completing a turn

const double TURN_TIME_LEFT = 1190.0;
const double TURN_TIME_RIGHT = 1200.0;

// ---------------     Settings     ---------------

boolean TOGGLE_SLOW_TURN = false;
boolean TOGGLE_PID = false;
boolean TOGGLE_TIME_TURN = false;
boolean TOGGLE_TIME_MOVE = false;

// ---------------   Arduino Pins   ---------------

// Start Button
const int BUTTON_PIN = 11;

// Motor Controller
const int STBY = 7; // White
// Left Motor
const int PWMA = 4; // Blue
const int AIN2 = 5; // Green
const int AIN1 = 6; // Yellow
// Right Motor
const int BIN1 = 8; // Yellow
const int BIN2 = 9; // Green
const int PWMB = 10; // Blue

// Left Encoder
const int ENCODER_LEFT_PIN_A = 44; // Yellow
const int ENCODER_LEFT_PIN_B = 3; // White
// Right Encoder
const int ENCODER_RIGHT_PIN_A = 46; // Yellow
const int ENCODER_RIGHT_PIN_B = 2; // White

// ---------------   Motor Setup   ---------------

// Motor Controller
Motor motorLeft(AIN1, AIN2, PWMA, 1, STBY);
Motor motorRight(BIN2, BIN1, PWMB, 1, STBY);

// Encoders
long encoderEnd; // Variable to hold the amount of encoder pulses to reach the target
// Left Encoder
boolean encoderLeftDirection;          // Tracks the direction of the left motor (TRUE is foward)
long encoderLeftPulses;         // Tracks the amount of encoder pulses in the left motor
int encoderLeftLastState;
// Right Encoder
boolean encoderRightDirection;         // Tracks the direction of the right motor (TRUE is foward)
long encoderRightPulses;         // Tracks the amount of encoder pulses in the left motor
int encoderRightLastState;

// ---------------     Compass     ---------------

// Variables
double compass = 0.0;         // Actual reading of the compass module
double heading = 0.0;         // The current heading 
int direction_flag = 0;          // Tracks which region the heading currently is in order to track when heading wraps from 0 to 360
int revolutions = 0;            // Revolutions
double targetHeading = 0.0;       // Heading to align with (for moving and turning)

// ---------------       PID       ---------------

// Alignment
double motorSpeedOffset = 0;
double Kp = 1.0, Ki = 0, Kd = 0.0;
PID alignPID(&targetHeading, &motorSpeedOffset, &heading, Kp, Ki, Kd, DIRECT);

// Time
double optimal_cm_per_second; 
double motorSpeedOffsetOnTime = 0;
double optimalDistance = 0.0;
double actualDistance = 0.0;
PID timePID(&optimalDistance, &motorSpeedOffset, &actualDistance, Kp, Ki, Kd, DIRECT);

// ---------------  Miscellaneous  ---------------

const long PRINT_DEBUG_COOLDOWN = 100;          // Time between printing debug information

int buttonState;          // Variable for reading the start button status
int lastButtonState;          // Variable for reading the previous start button status

double moveSpeed = 0.0;         // The current speed of the motors (while moving with moveDistance)

bool beginPath = false;         // If set to TRUE, the set movement primitives will run

unsigned long previousMillis = millis();          // Variable that holds the previous timestamp

void fw(double blocks = 1.0);         // Instantiate primitives to allow for 
void bw(double blocks = 1.0);         // a default parameter of 1 block

// -----------------------------------------------



// Main

void setup(){

  // Start debug serial
  Serial.begin(115200);
  
	Wire.begin();

  // Initialize start button pin as an input
  pinMode(BUTTON_PIN, INPUT);

  // Initialize encoders
  encoderLeftPulses = 0.0;
  encoderRightPulses = 0.0;
  encodersInit();

  // Initialize compass (Unused)
  /*
  JY901.StartIIC();
  Serial.println("Compass ready");
  */

  // Initialize PID
  alignPID.SetOutputLimits(-MOTOR_SPEED_MAX, 240-MOTOR_SPEED_MAX);
  alignPID.SetSampleTime(100);

  timePID.SetOutputLimits(-MOTOR_SPEED_MAX, 240-MOTOR_SPEED_MAX);
  timePID.SetSampleTime(100);

  //
  
}

void loop(){

  beginPath = false;

  // Read the state of the start button
  buttonState = digitalRead(BUTTON_PIN); 

  // Determine if the start button has been press (HIGH means the button is being pressed)
  if (buttonState == HIGH && lastButtonState == LOW){
    

    // Give time to take finger off of the button
    delay(1440); 

    // Begin the path
    beginPath = true;

  }
  // Save the start button state for the next loop
  lastButtonState = buttonState;

  if (beginPath){
   
    // Move into the first square
    
    moveDistance(20.0);

    // ---------------  Create Path Here  ---------------
    
    fw();

    // --------------------------------------------------

    // Move dowel to the ending location
    moveDistance(5.0);

    // Stop running the path
    beginPath = false;
  }
  readHeading();
  printDebugInfo();

  // Make sure your target heading is aligned wiht your initial heading
  targetHeading = heading;
  
}

// Path Calculations
void calculatePathTime(){
  int amount;
  int linear_count = 0;
  int right_rotation_count = 0;
  int left_rotation_count = 0;
  for (byte index = 0; index < sizeof(PATH); index += 2){
    amount = PATH[index+1];
    switch (PATH[index]) {
      case 'f':
        linear_count += amount;
        break;
      case 'b':
        linear_count += amount;
        break;
      case 'r':
        right_rotation_count += amount;
        break;
      case 'l':
        left_rotation_count += amount;
        break;
      default:
        Serial.print("Calculation Path Error: unknown movement function");
        break;
    }
  }

  optimal_cm_per_second = BLOCK_SIZE / ((TARGET_TIME - (left_rotation_count*TURN_TIME_LEFT) - (right_rotation_count*TURN_TIME_RIGHT)) / linear_count);
}

void runPath(){
  int amount;
  for (byte index = 0; index < sizeof(PATH); index += 2){
    amount = PATH[index+1];
    switch (PATH[index]) {
      case 'f':
        fw(amount);
        break;
      case 'b':
        bw(amount);
        break;
      case 'r':
        right();
        break;
      case 'l':
        left();
        break;
      default:
        Serial.print("Run Path Error: unknown movement function");
        break;
    }
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
  turnDegrees(90.0);
}

void right(){
  turnDegrees(-90.0);
}

// Movement Functions

void moveDistance(double distance){

  // Set encoders
  encoderEnd = distance * ENCODER_PER_CENTIMETER;
  encoderLeftPulses = 0.0;
  encoderRightPulses = 0.0;
  long avgPulses = 0.0;
  long distanceFromStart = 0.0;
  double midpointSpeed = 0.0;

  long initialTime = millis();
  long time = 0.0;

  double left_motor_speed_multiplier;

  boolean accelerate = true;

  // PID
  alignPID.SetMode(1);

  // Determine direction of target distance
  int direction;
  if (distance >= 0){
    direction = 1; // Target is ahead
    left_motor_speed_multiplier = FW_MOTOR_SPEED_MULTIPLIER;
  } else {
    direction = -1; // Target is behind
    left_motor_speed_multiplier = BW_MOTOR_SPEED_MULTIPLIER;
  }


  while (avgPulses < abs(encoderEnd)){ // Wait for the encoders to count to the target pulse count
    
    time = millis() - initialTime;

    avgPulses = abs(encoderRightPulses);
    
    
    if (avgPulses < (abs(encoderEnd) / 2.0) && moveSpeed < MOTOR_SPEED_MAX){
      moveSpeed = constrain(
        MOTOR_SPEED_MIN + ((MOTOR_SPEED_MAX - MOTOR_SPEED_MIN) * time * 0.001),
        MOTOR_SPEED_MIN,
        MOTOR_SPEED_MAX
      );
      //Serial.println("STATE: ACCEL");
      distanceFromStart = avgPulses;
    } else if (avgPulses > (abs(encoderEnd) - distanceFromStart) && !accelerate){
      moveSpeed = constrain(
        midpointSpeed - ((midpointSpeed - 0.0) * time * 0.001),
        MOTOR_SPEED_MIN,
        MOTOR_SPEED_MAX
      );
      //Serial.println("STATE: DECCEL");
    } else {
      initialTime = millis();
      midpointSpeed = moveSpeed;
      accelerate = false;
      //Serial.println("STATE: COAST");
    }
    
    
    readHeading();
    if (TOGGLE_PID){
      alignPID.Compute();
    } else {
      motorSpeedOffset = 0.0;
    }

    if (TOGGLE_TIME_MOVE){
      moveSpeed = MOTOR_SPEED;
      motorSpeedOffset = 0.0;
    }

    
    motorLeft.drive((motorSpeedOffset + moveSpeed) * left_motor_speed_multiplier * direction);
    motorRight.drive(moveSpeed * direction);

    printDebugInfo();  
  }

  // Stop moving after target distance is reached
  hitBrakes();
  alignPID.SetMode(0);

  delay(200);

}

void turnDegrees(double degrees){
  // Update the heading to the current compass reading
  readHeading();

  // Determine direction of target angle (counterclockwise is positive)
  int direction;
  if (degrees >= 0){
    direction = 1; // Turn to the left
  } else {
    direction = -1; // Turn to the right
  }

  // Calculate the target heading in the determined direction
  targetHeading = targetHeading + degrees;
  

  // Tell motors to drive according to the direction of the angle
  motorLeft.drive(MOTOR_SPEED * FW_MOTOR_SPEED_MULTIPLIER * direction);
  motorRight.drive(MOTOR_SPEED * -direction); // Moves the opposite direction in order to turn

  // Turn until the heading reaches the target heading
  if (TOGGLE_TIME_TURN){
    if (direction == 1){
      delay(TURN_TIME_RIGHT);
    } else {
      delay(TURN_TIME_LEFT);
    }
  } else {
    while (direction * (heading - targetHeading) > TURN_TOLERANCE){
      readHeading();
      printDebugInfo();
    }
  }

  // Stop moving after reaching target angle
  hitBrakes();

  delay(200);
}

void hitBrakes(){
  brake(motorLeft,motorRight);
}

// Compass

void readHeading(){
  // Compass Unused
  return;
  JY901.GetAngle();
  compass = 180 + ((double)JY901.stcAngle.Angle[2] / 32768 * 180);
  
  if (direction_flag == 1 && compass < 180.0){
    revolutions += 1;
    direction_flag = 0;
  } else if (direction_flag == -1 && compass > 180){
    revolutions -= 1;
    direction_flag = 0;
  }
  if ((compass < 100.0)){
    direction_flag = -1;
  } else if ((compass > 260.0)){
    direction_flag = 1;
  } else {
    direction_flag = 0;
  }

  // Calculates the true accumulative heading
  heading = (revolutions * 360) + compass; 
}

// Encoder Functions

void encodersInit() {
  encoderLeftDirection = true;                           // TRUE -> Forward
  encoderRightDirection = true;                           // TRUE -> Forward

  pinMode(ENCODER_LEFT_PIN_B,INPUT);
  pinMode(ENCODER_RIGHT_PIN_B,INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN_B), encoderLeftCounter, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN_B), encoderRightCounter, CHANGE);
}

void encoderLeftCounter() {
  int leftStateA = digitalRead(ENCODER_LEFT_PIN_A);
  int leftStateB;
  if(encoderLeftLastState == LOW && leftStateA == HIGH)
  {
    leftStateB = digitalRead(ENCODER_LEFT_PIN_B);

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

  if(encoderLeftDirection)  encoderLeftPulses--; // NOTE: This is opposite of the right encoder since they are inverted
  else  encoderLeftPulses++;
}

void encoderRightCounter() {

  int rightStateA = digitalRead(ENCODER_RIGHT_PIN_A);
  int rightStateB;
  if(encoderRightLastState == LOW && rightStateA == HIGH)
  {
    rightStateB = digitalRead(ENCODER_RIGHT_PIN_B);

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

  if(encoderRightDirection)  encoderRightPulses++;
  else  encoderRightPulses--;
}

// Debug
void printDebugInfo(){
  // Serial Studio
  Serial.println();
  Serial.print("/*");
  Serial.print(compass);
  Serial.print(",");
  Serial.print(heading);
  Serial.print(",");
  Serial.print(targetHeading);
  Serial.print(",");
  Serial.print(encoderEnd);
  Serial.print(",");
  Serial.print(encoderLeftPulses);
  Serial.print(",");
  Serial.print(encoderRightPulses);
  Serial.print(",");
  Serial.print(revolutions);
  Serial.print(",");
  Serial.print(motorSpeedOffset);
  Serial.print("*/");
  
}
