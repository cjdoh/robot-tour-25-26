#include <SparkFun_TB6612.h>
#include <Wire.h>
#include <PID_v1.h>
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

const double MOTOR_SPEED_MIN = 1.0;         // Starting and ending speed of both motors
const double MOTOR_SPEED_MAX = 50.0;         // Maximum speed of both motors

const double FW_MOTOR_SPEED_MULTIPLIER = 1.02;         // In case one motor is slower than the other, only applies to the left motor when moving fowards
const double BW_MOTOR_SPEED_MULTIPLIER = 1.008;         // In case one motor is slower than the other, only applies to the left motor when moving backwards

const double LEFT_MOTOR_SPEED_SHIFT = 0.29;

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
long encoderLeftPulsesPrevious;          // Holds the amount of pulses in the left motor at the time of the last speed PID calculation
int encoderLeftLastState;
// Right Encoder
boolean encoderRightDirection;         // Tracks the direction of the right motor (TRUE is foward)
long encoderRightPulses;         // Tracks the amount of encoder pulses in the right motor
long encoderRightPulsesPrevious;          // Holds the amount of pulses in the right motor at the time of the last speed PID calculation
int encoderRightLastState;

// ---------------       PID       ---------------
double Kp = 0.2, Ki = 5.0, Kd = 8.0;
double optimalSpeed;
double zero = 0.0;

// Left Motor Speed Calibration PID
long previousSpeedPIDMillis;          // Timestamp of the last speed PID calculation 
double leftMotorOffset;
double leftMotorCMPS;          // Centimeter Per Second
PID leftSpeedPID(&zero, &leftMotorOffset, &leftMotorCMPS, Kp, Ki, Kd, DIRECT);

// Right Motor Speed Calibration PID
double rightMotorOffset;
double rightMotorCMPS;          // Centimeter Per Second
PID rightSpeedPID(&zero, &rightMotorOffset, &rightMotorCMPS, Kp, Ki, Kd, DIRECT);

// Time

double optimal_cm_per_second; 
double motorSpeedOffsetOnTime = 0;
double optimalDistance = 0.0;
double actualDistance = 0.0;
//PID timePID(&optimalDistance, &motorSpeedOffset, &actualDistance, Kp, Ki, Kd, DIRECT);

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
  clearEncoderCount();
  encodersInit();

  // Initialize PID
  leftSpeedPID.SetMode(1);
  rightSpeedPID.SetMode(1);
  leftSpeedPID.SetOutputLimits(-250.0, 250.0);
  rightSpeedPID.SetOutputLimits(-250.0, 250.0);
  
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
  
    right();

    // --------------------------------------------------

    // Move dowel to the ending location
    moveDistance(5.0);

    // Stop running the path
    beginPath = false;
  }
  
  adjustMotorSpeed();
  printDebugInfo();
  
  
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
  encoderEnd = abs(distance * ENCODER_PER_CENTIMETER);
  
  clearEncoderCount();

  long averagePulses = 0.0;
  long distanceFromStart = 0.0;
  double midpointSpeed = 0.0;

  long initialTime = millis();
  long time = 0.0;

  double left_motor_speed_multiplier;

  optimalSpeed = 25.0;

  boolean accelerate = true;

  


  // Determine direction of target distance
  int direction;
  if (distance >= 0){
    direction = 1; // Target is ahead
    left_motor_speed_multiplier = FW_MOTOR_SPEED_MULTIPLIER;
  } else {
    direction = -1; // Target is behind
    left_motor_speed_multiplier = BW_MOTOR_SPEED_MULTIPLIER;
  }


  while (averagePulses < abs(encoderEnd)){ // Wait for the encoders to count to the target pulse count
    
    time = millis() - initialTime;

    averagePulses = abs(encoderRightPulses);
    
    /*
    if (averagePulses < (encoderEnd / 2.0) && moveSpeed < MOTOR_SPEED_MAX){
      moveSpeed = constrain(
        MOTOR_SPEED_MIN + ((MOTOR_SPEED_MAX - MOTOR_SPEED_MIN) * time * 0.001),
        MOTOR_SPEED_MIN,
        MOTOR_SPEED_MAX
      );
      distanceFromStart = averagePulses;
    } else if (averagePulses > (encoderEnd - distanceFromStart) && !accelerate){
      moveSpeed = constrain(
        midpointSpeed - ((midpointSpeed - 0.0) * time * 0.001),
        MOTOR_SPEED_MIN,
        MOTOR_SPEED_MAX
      );
    } else {
      initialTime = millis();
      midpointSpeed = moveSpeed;
      accelerate = false;
    }
    */

    if (averagePulses < (encoderEnd/2.0)){
      moveSpeed = MOTOR_SPEED_MAX;
    } else {
      moveSpeed = MOTOR_SPEED_MIN;
    }
    
    
    


    if (TOGGLE_TIME_MOVE){
      moveSpeed = MOTOR_SPEED;
      leftMotorOffset = 0.0;
      rightMotorOffset = 0.0;
    } else {
      adjustMotorSpeed();
    }
    optimalSpeed = moveSpeed;
    

    motorLeft.drive((leftMotorOffset) * direction);
    motorRight.drive((rightMotorOffset) * direction);

    printDebugInfo();
  }

  // Stop moving after target distance is reached
  hitBrakes();

  delay(200);

}

void turnDegrees(double degrees){
  // Update the heading to the current compass reading
  

  // Determine direction of target angle (counterclockwise is positive)
  int direction;
  if (degrees >= 0){
    direction = -1; // Turn to the right
    encoderEnd = ((3.14159*14.3)/2.0)*ENCODER_PER_CENTIMETER;
  } else {
    direction = 1; // Turn to the left
    encoderEnd = ((3.14159*14.2)/2.0)*ENCODER_PER_CENTIMETER;
  }

  optimalSpeed = 10;

  

  clearEncoderCount();

  while (abs(encoderRightPulses) < encoderEnd){
    adjustMotorSpeed();
    motorLeft.drive(leftMotorOffset * direction);
    motorRight.drive(rightMotorOffset * -direction); // Moves the opposite direction in order to turn
  }

  // Stop moving after reaching target angle
  hitBrakes();

  delay(200);
}

void hitBrakes(){
  brake(motorLeft,motorRight);
}

// Movement Calculations

void clearEncoderCount(){
  leftSpeedPID.SetOutputLimits(0.0, 1.0);
  rightSpeedPID.SetOutputLimits(0.0, 1.0);
  leftSpeedPID.Compute();
  rightSpeedPID.Compute();
  leftSpeedPID.SetOutputLimits(-200.0, 200.0);
  rightSpeedPID.SetOutputLimits(-200.0, 200.0);
  encoderLeftPulses = 0.0;
  encoderRightPulses = 0.0;
  encoderLeftPulsesPrevious = 0.0;
  encoderRightPulsesPrevious = 0.0;
  previousSpeedPIDMillis = 0.0;
}

void adjustMotorSpeed(){
  
  // Make sure there is a previous position to compare to
  if (true) {
    double deltaTime = (double) (millis() - previousSpeedPIDMillis);

    // Stop calculation if last calculation happened too recently
    if (deltaTime < 10.0) {
      return;
    }
    Serial.println(optimalSpeed);

    double deltaPoisitionLeft = (encoderLeftPulses - encoderLeftPulsesPrevious) / ENCODER_PER_CENTIMETER;
    double deltaPoisitionRight = (encoderRightPulses - encoderRightPulsesPrevious) / ENCODER_PER_CENTIMETER;

    leftMotorCMPS = optimalSpeed + LEFT_MOTOR_SPEED_SHIFT - abs(deltaPoisitionLeft / (deltaTime / 1000.0));
    rightMotorCMPS = optimalSpeed - abs(deltaPoisitionRight / (deltaTime / 1000.0));

    leftSpeedPID.Compute();
    rightSpeedPID.Compute();
  }
  encoderLeftPulsesPrevious = encoderLeftPulses;
  encoderRightPulsesPrevious = encoderRightPulses;
  previousSpeedPIDMillis = millis();
  
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
  Serial.print(encoderEnd);
  Serial.print(", ");
  Serial.print(encoderLeftPulses);
  Serial.print(", ");
  Serial.print(encoderRightPulses);
  Serial.print(", ");
  Serial.print(leftMotorCMPS);
  Serial.print(", ");
  Serial.print(rightMotorCMPS);
  Serial.print(", ");
  Serial.print(leftMotorOffset);
  Serial.print(", ");
  Serial.print(rightMotorOffset);
  Serial.print(", ");
  Serial.print(optimalSpeed);
  Serial.print("*/");
  
}
