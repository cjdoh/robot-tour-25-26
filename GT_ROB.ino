 //Gyroscopic Automatic Vehicular Intelligence
#include <SparkFun_TB6612.h>
#include <Wire.h>
#include <PID_v1.h>
#include "witmotion_i2c.h"

JY901_I2C JY901(Wire);

// Pin Definitions 
const int buttonPin = A0;

// Configuration Parameters 
const double ENC_PER_CM = 58.3727699;
const int BASE_SPEED = 70;
const double BLOCK_SIZE_CM = 50.0;

// Motor Setup
const unsigned int PWMA = 5; 
const unsigned int AIN2 = 7; 
const unsigned int AIN1 = 8; 
const unsigned int BIN1 = 9; 
const unsigned int BIN2 = 12; 
const unsigned int PWMB = 6; 

Motor motorLeft(AIN1, AIN2, PWMA, 1, 4);
Motor motorRight(BIN1, BIN2, PWMB, 1, 4);

// Encoder Configuration
const byte encoderLpinA = 2;
const byte encoderLpinB = A1;
const byte encoderRpinA = 3;
const byte encoderRpinB = A2;

volatile long encLpulses = 0;
volatile long encRpulses = 0;
volatile byte encoderLPinALast = LOW;
volatile byte encoderRPinALast = LOW;
volatile bool encLdir = true;
volatile bool encRdir = true;

// Navigation Variables 
double currentHeading = 0;
double straightHeading = 0;
double goalHeading = 0;
bool pathActive = false;

// PID Configuration 
double headingInput = 0;      // Current heading
double pidOutput = 0;          // Speed correction
double headingSetpoint = 0;    // Target heading
double Kp = 2.0, Ki = 0.05, Kd = 0.5;

PID headingPID(&headingInput, &pidOutput, &headingSetpoint, Kp, Ki, Kd, DIRECT);

// UI Variables 
int buttonState = 0;
int lastButtonState = 0;
unsigned long lastDebugTime = 0;
const long DEBUG_INTERVAL = 1000;

// Function Prototypes 
void initEncoders();
void updateHeading();
void moveStraight(double distanceCM, bool forward);
void Turn(double degrees);
void forward(double blocks);
void backward(double blocks);
void moveINITIALIZE();
void moveFINALIZE();
void executePath();
double normalizeAngle(double angle);
double getAngleError(double target, double current);
void stopMotors();
void printDebugInfo();
void encoderLISR();
void encoderRISR();

void setup() {
  Serial.begin(115200);
  while (!Serial) { }
  
 Wire.begin();      // Initialize I2C
  JY901.begin();     // Initialize gyro on I2C

  
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  
  pinMode(buttonPin, INPUT);
  
  initEncoders();
  
  // Initialize PID
  headingPID.SetOutputLimits(-50, 50);  // Correction range
  headingPID.SetSampleTime(50);
  headingPID.SetMode(AUTOMATIC);
  
  delay(500);
  updateHeading();
  straightHeading = currentHeading;
  
  Serial.println("=================================");
  Serial.println("Robot Initialized");
  Serial.println("Press button to start path");
  Serial.println("=================================");
}

void loop() {
  updateHeading();
  printDebugInfo();
  
  // Button handling with debounce
  buttonState = digitalRead(buttonPin);
  
  if (buttonState == HIGH && lastButtonState == LOW) {
    delay(50);  // Debounce
    if (digitalRead(buttonPin) == HIGH) {
      Serial.println("\n>>> Button pressed! Starting in 2 seconds...");
      delay(2000);
      
      updateHeading();
      straightHeading = currentHeading;
      pathActive = true;
      
      executePath();
      
      pathActive = false;
      Serial.println(">>> Path completed!");
    }
  }
  
  lastButtonState = buttonState;
}

void executePath() {
  /* ---------------- DO NOT TOUCH ----------------- */
  moveINITIALIZE();
  /* ---------------- DO NOT TOUCH ----------------- */
  
 
  /* Examples:
   forward(1);      // Move 1 block forward (50cm)
   backward(1);     //Move 1 block backward (50cm)
   backward(0.5);   // Release (25cm)
   forward(0.5);    // Capture (25cm)
   Turn(90);        // Turn 90 degrees right
   Turn(-90);       // Turn 90 degrees left
  
  
  
  /* ---------------- DO NOT TOUCH ----------------- */
  moveFINALIZE();
  /* ---------------- DO NOT TOUCH ----------------- */
}

void moveINITIALIZE() {
  moveStraight(22.0 + 9.0, true);
}

void moveFINALIZE() {
  moveStraight(9.0, false);
}

void forward(double blocks) {
  moveStraight(blocks * BLOCK_SIZE_CM, true);
}

void backward(double blocks) {
  moveStraight(blocks * BLOCK_SIZE_CM, false);
}

void Turn(double degrees) {
  const int TURN_SPEED = 50;
  const double TOLERANCE = 2.0;
  const unsigned long TIMEOUT = 3000;
  
  updateHeading();
  double startHeading = currentHeading;
  double targetHeading = normalizeAngle(currentHeading + degrees);
  
  Serial.print("Turning ");
  Serial.print(degrees);
  Serial.print(" degrees to ");
  Serial.println(targetHeading);
  
  unsigned long startTime = millis();
  
  while (millis() - startTime < TIMEOUT) {
    updateHeading();
    double error = getAngleError(targetHeading, currentHeading);
    
    // Determine turn direction
    if (degrees > 0) {  // Right turn
      motorLeft.drive(TURN_SPEED);
      motorRight.drive(-TURN_SPEED);
    } else {  // Left turn
      motorLeft.drive(-TURN_SPEED);
      motorRight.drive(TURN_SPEED);
    }
    
    // Check if we've reached target
    if (abs(error) < TOLERANCE) {
      Serial.println("Turn complete!");
      break;
    }
  }
  
  stopMotors();
  delay(300);
  
  // Update straight heading for next movement
  updateHeading();
  straightHeading = currentHeading;
}

void moveStraight(double distanceCM, bool forward) {
  const unsigned long TIMEOUT = abs(distanceCM) * 100;  // ms per cm
  
  // Reset encoders
  noInterrupts();
  encLpulses = 0;
  encRpulses = 0;
  interrupts();
  
  long targetPulses = abs(distanceCM * ENC_PER_CM);
  
  // Set PID target to current straight heading
  headingSetpoint = straightHeading;
  pidOutput = 0;
  headingPID.SetMode(AUTOMATIC);
  
  Serial.print(forward ? "Forward " : "Backward ");
  Serial.print(distanceCM);
  Serial.println(" cm");
  
  unsigned long startTime = millis();
  
  while (millis() - startTime < TIMEOUT) {
    updateHeading();
    
    // Update PID input
    headingInput = currentHeading;
    headingPID.Compute();
    
    // Calculate motor speeds with PID correction
    int leftSpeed = constrain(BASE_SPEED + pidOutput, 0, 255);
    int rightSpeed = constrain(BASE_SPEED - pidOutput, 0, 255);
    
    // Drive motors
    if (forward) {
      motorLeft.drive(leftSpeed);
      motorRight.drive(rightSpeed);
    } else {
      motorLeft.drive(-leftSpeed);
      motorRight.drive(-rightSpeed);
    }
    
    // Check if target reached
    if (abs(encRpulses) >= targetPulses) {
      Serial.println("Distance reached!");
      break;
    }
  }
  
  stopMotors();
  delay(100);
}

void stopMotors() {
  motorLeft.brake();
  motorRight.brake();
}

void initEncoders() {
  pinMode(encoderLpinB, INPUT);
  pinMode(encoderRpinB, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(encoderLpinA), encoderLISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRpinA), encoderRISR, CHANGE);
}

void encoderLISR() {
  byte state = digitalRead(encoderLpinA);
  
  if (encoderLPinALast == LOW && state == HIGH) {
    encLdir = (digitalRead(encoderLpinB) == LOW);
  }
  
  encoderLPinALast = state;
  
  if (encLdir) encLpulses++;
  else encLpulses--;
}

void encoderRISR() {
  byte state = digitalRead(encoderRpinA);
  
  if (encoderRPinALast == LOW && state == HIGH) {
    encRdir = (digitalRead(encoderRpinB) == HIGH);
  }
  
  encoderRPinALast = state;
  
  if (encRdir) encRpulses++;
  else encRpulses--;
}

void updateHeading() {
  JY901.fetchAngles();                      // Fetch via I2C
  currentHeading = JY901.getAngles().yaw + 180.0;  // Get yaw angle
  currentHeading = normalizeAngle(currentHeading);
}

double normalizeAngle(double angle) {
  while (angle >= 360.0) angle -= 360.0;
  while (angle < 0.0) angle += 360.0;
  return angle;
}

double getAngleError(double target, double current) {
  double error = target - current;
  
  // Shortest path calculation
  if (error > 180.0) error -= 360.0;
  else if (error < -180.0) error += 360.0;
  
  return error;
}

void printDebugInfo() {
  if (millis() - lastDebugTime < DEBUG_INTERVAL) return;
  
  Serial.print("\033[2J\033[H");  // Clear screen
  
  Serial.println("=== Robot Status ===");
  Serial.print("Heading: ");
  Serial.print(currentHeading, 1);
  Serial.print("° | Target: ");
  Serial.print(headingSetpoint, 1);
  Serial.print("° | Error: ");
  Serial.println(getAngleError(headingSetpoint, currentHeading), 1);
  
  Serial.print("PID Output: ");
  Serial.print(pidOutput, 1);
  Serial.print(" | Straight: ");
  Serial.println(straightHeading, 1);
  
  Serial.print("Encoders L: ");
  Serial.print(encLpulses);
  Serial.print(" | R: ");
  Serial.println(encRpulses);
  
  Serial.print("PID: Kp=");
  Serial.print(Kp, 2);
  Serial.print(" Ki=");
  Serial.print(Ki, 3);
  Serial.print(" Kd=");
  Serial.println(Kd, 2);
  
  Serial.println("====================");
  
  lastDebugTime = millis();
}
    
