#include <L298NX2.h>
#include <Wire.h>
//#include "JY901.h"
#include <PID_v1.h>
#include "JY901_Serial.h"

// ---- Set pin numbers: ----
const int buttonPin = 53;                 // the number of the pushbutton pin

// ---- Key Parameters ----
    //const int encLend = 10*1920;             // pulses for left motor - not used 
const double travelDist = -50;           // travel distance in CM  
const double ENCperCM = 58.3727699;                //Number of encoder counts per cm // Chris's value: 59.00727699 // Isaac's value: 59.6107603336
int motor_A_speed = 70;                // motor A speed (left Motor)
int motor_B_speed = 70;                // motor B speed (right Motor)
int run_forward_cnt = 0;               // howmany times we have run forward
int run_backward_cnt = 0;
const double turnAngle = 80.0;

const double blockSize = 50.0; // Size of one of side of a "block" on the grid (centimeters)

// ---- Motor Setup ----
  // Left Motor
    const unsigned int EN_A = 2; 
    const unsigned int IN1_A = 29; 
    const unsigned int IN2_A = 27; 
  // Right Motor
    const unsigned int EN_B = 3; 
    const unsigned int IN1_B = 25; 
    const unsigned int IN2_B = 23; 

// ---- Motor Variables ---- 
  // Initialize both motors
    L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);
  // Encoders Setup
    // Left Motor Encoder
      const byte encoderLpinA = 21;            // A pin -> the interrupt pin 0
      const byte encoderLpinB = 33;           // B pin -> the digital pin 3
      byte encoderLPinALast;
      long encLpulses;                        // the number of the pulses
      boolean encLdir;                        // the rotation direction
    // Right Motor Encoder
      const byte encoderRpinA = 35;            // A pin -> the interrupt pin 0
      const byte encoderRpinB = 20;           // B pin -> the digital pin 3
      byte encoderRPinALast;
      long encRpulses;                        // the number of the pulses
      boolean encRdir;                        // the rotation direction
      long encEnd;                            // encoder endpoint for move

  // General variables:
    const long printDebugCooldown = 1000;                // Minimum time (milliseconds) between printing updates to the console
    int buttonState = 0;                      // Variable for reading the pushbutton status
    int lastbuttonState = 0;                  // Variable for reading the last pushbutton status
    int buttonSpeedState = 0;                      // Variable for reading the pushbutton speed status
    int lastbuttonSpeedState = 0;                  // Variable for reading the last pushbutton speed status
    
    int isMovingFoward = false;                // TRUE if robot is moving foward
    int isMovingBackward = false;               // TRUE if robot is backward
    unsigned long previousMillis = millis();         // will store last time Motor was run

    double heading = 0;                        // Compass heading
    double startHeading = 0;                        // Compass starting heading (when button is pressed)
    double goalHeading = 0;                       // Heading the robot wants to be on
    double straightHeading = 0;             // The heading the robot should be on to be moving straight

    double currentHeading = 0;                // Temporary heading variable used while rotating
    

    bool beginPath = true;               // If set to TRUE, the set movement primitives will run

// ---- PID (for going straight) ----
  // PID variables
    double goalHeading, motorA_offset=motor_A_speed;
    double Kp=1.0, Ki=1.0 , Kd=0;
  // Specify the links and initial tuning parameters
    double blankZero = 0;
    double myError = 0;
  PID myPID(&blankZero, &motorA_offset, &myError, Kp, Ki, Kd, DIRECT);



void setup() {
  // Setup Serial Output
    Serial.begin(115200);
    
    // Wait for Serial Monitor to be opened
    while (!Serial)
    {
      // do nothing
    }

    Serial1.begin(9600);

    JY901.attach(Serial1);

  // Initialize the pushbutton pin as an input:
    pinMode(buttonPin, INPUT);

  // Initialize encoders
    encLpulses = 0;
    encRpulses = 0;
    EncodersInit();
    delay(500);
    update_heading();
    startHeading = heading;
    myPID.SetOutputLimits(0, 255);
    myPID.SetSampleTime(100);
}

void loop() {
  update_heading();
  printDebugInfo();
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);



  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH && lastbuttonState == LOW) {
    delay(2000);                 //time to get finger off button

    update_heading();
    beginPath = true;
    straightHeading = heading;
    motors.setSpeedA(motor_A_speed);
    motors.setSpeedB(motor_B_speed);
  }  
  lastbuttonState = buttonState;
  

  // Run Motors Forward


  if (beginPath) {
    /* ---------------- DO NOT TOUCH ----------------- */
    moveStraightFoward(22.0+9.0);
    /* ---------------- DO NOT TOUCH ----------------- */
    
    // run primitives here
    forward();
    left();
    forward();
    left();
    forward();
    left();
    forward();
    left();
    forward();
    left();
  


    /* ---------------- DO NOT TOUCH ----------------- */
    moveStraightBackward(-9.0);
    /* ---------------- DO NOT TOUCH ----------------- */

    beginPath = false;
    
  }

}

// Move foward n blocks
void foward(int blocks = 1){
  move(blocks);
}

// Move a half step foward
void capture(){
  moveStraightFoward(blockSize/2.0)
}

// Move backward n blocks
void backward(int blocks = 1){
  move(-blocks);
}

// Move a half step backward
void release(){
  moveStraightBackward(-blockSize/2.0);
}

// Move n blocks (positive for forward, negative for backward)
void move(int blocks) {
  double distanceCentimeters = (double)blocks * blockSize.0;
  if (blocks >= 0){
    moveStraightFoward(distanceCentimeters);
  } else {
    moveStraightBackward(-distanceCentimeters);
  }  
}

void right() {
  // update_heading();
  currentHeading = read_compass();
  goalHeading = currentHeading - turnAngle;
  if (goalHeading < 0.0) {
    goalHeading += 360.0;
  }

  motors.setSpeedA(50);
  motors.backwardA();
  motors.setSpeedB(50);
  motors.forwardB();

  while ((currentHeading > goalHeading) || (abs(currentHeading-goalHeading) > turnAngle)) {
    // update_heading();
    currentHeading = read_compass();
    printDebugInfo();
  }
  motors.stop();
  delay(200);

  straightHeading -= 90;
  if (straightHeading < -180) {
    straightHeading += 360;
  }
}

void left() {
  // update_heading();
  currentHeading = read_compass();
  goalHeading = currentHeading + turnAngle;
  if (goalHeading > 360.0) {
    goalHeading -= 360.0;
  }

  motors.setSpeedA(70);
  motors.setSpeedB(70);
  motors.forwardA();
  motors.backwardB();

  while ((currentHeading < goalHeading) || (abs(currentHeading-goalHeading) > turnAngle)) {
    // update_heading();
    currentHeading = read_compass();
    printDebugInfo();
  }
  motors.stop();

  straightHeading += 90;
  if (straightHeading > 180) {
    straightHeading -= 360;
  }

  delay(200);

}

void moveStraightFoward(double distance) {
  update_heading();
  encEnd = distance * ENCperCM;
  encLpulses = 0;
  encRpulses = 0;
  goalHeading = straightHeading;
  while (encRpulses < encEnd && encLpulses < encEnd) {

    // Check to see if you are moving or if you need to start moving
      if (!isMovingFoward) {
        // Start Moving 
        myPID.SetMode(1);
        motors.forward();
        isMovingFoward = true;
      }
      update_heading();
      KeepStraightF();
      printDebugInfo();
  }
  // Stop moving and Clean up
  hit_breaks();
  motors.stop();
  myPID.SetMode(0);
  isMovingFoward = false;
  beginPath = 0;
  printDebugInfo();
  delay(200);
}

void moveStraightBackward(double distance) {
  update_heading();
  encEnd = distance * ENCperCM;
  encLpulses = 0;
  encRpulses = 0;
  goalHeading = straightHeading;
  while (encRpulses > encEnd && encLpulses > encEnd) {

    // Check to see if you are moving or if you need to start moving
      if (!isMovingBackward) {
        // Start Moving
        // Make robot drive backward
        myPID.SetMode(1);
        motors.backward();
        isMovingBackward = true;
      }
      update_heading();
      KeepStraightB();
      printDebugInfo();
  }
  // Stop moving and Clean up
  hit_breaks();
  motors.stop();
  myPID.SetMode(0);
  isMovingBackward = false;
  beginPath = 0;
  printDebugInfo();
  delay(200);
}

void KeepStraightF() {
  update_heading();
  myPID.Compute();
  motors.setSpeedA(motorA_offset);
  motors.setSpeedB(motor_B_speed);
  motors.forward();
  printDebugInfo();
}

void KeepStraightB() {
  update_heading();
  myPID.Compute();
  motors.setSpeedB(motorA_offset);
  motors.setSpeedA(motor_B_speed);
  motors.backward();
  printDebugInfo();
}

void EncodersInit() {
  encLdir = true;                           //default -> Forward
  encRdir = true;                           //default -> Forward

  pinMode(encoderLpinB,INPUT);
  pinMode(encoderRpinB,INPUT);

  attachInterrupt(2, EncoderLCounter, CHANGE);
  attachInterrupt(3, EncoderRCounter, CHANGE);
}

void EncoderLCounter() {
  int encLLstate = digitalRead(encoderLpinA);
  if((encoderLPinALast == LOW) && encLLstate==HIGH)
  {
    int val = digitalRead(encoderLpinB);
    if(val == LOW && encLdir)
    {
      encLdir = false;                      //Reverse
    }
    else if(val == HIGH && !encLdir)
    {
      encLdir = true;                       //Forward
    }
  }
  encoderLPinALast = encLLstate;

  if(encLdir)  encLpulses++;                // NOTE This is opposite of the right encoder since they are inverted
  else  encLpulses--;
}

void EncoderRCounter() {
  int encRLstate = digitalRead(encoderRpinA);
  if((encoderRPinALast == LOW) && encRLstate==HIGH)
  {
    int val = digitalRead(encoderRpinB);
    if(val == LOW && encRdir)
    {
      encRdir = false;                      //Reverse
    }
    else if(val == HIGH && !encRdir)
    {
      encRdir = true;                       //Forward
    }
  }
  encoderRPinALast = encRLstate;

  if(!encRdir) encRpulses--;
  else encRpulses++;
}


void hit_breaks(){

  motors.stop();

}

double read_compass() {
  JY901.receiveSerialData();

  return JY901.getYaw() + 180.0;
}

void update_heading(){
  JY901.receiveSerialData();

  heading = JY901.getYaw();
  myError = goalHeading - heading;
  if (myError > 180)
    myError -= 360;
  else if (myError < -180)
    myError += 360;

}

void printDebugInfo() {
  if(millis() - previousMillis > printDebugCooldown)
    {
    // Print motor info in Serial Monitor
      Serial.print("\033[0H\033[0J");       //Clear terminal window
      
      Serial.print("Bytes Avalible = ");
      Serial.print(Serial.availableForWrite());
      Serial.print("    Run Forward Count = ");
      Serial.println(run_forward_cnt);
      Serial.print("    Goal Heading/PID Heading = ");
      Serial.print(goalHeading);
      Serial.print("    Compass Heading = ");
      Serial.print(heading);
      Serial.print("    Compass Delta = ");
      Serial.println(startHeading-heading);
      Serial.print("Turn Compass Heading = ");
      Serial.print(currentHeading);
      Serial.print("    Straight Heading = ");
      Serial.println(straightHeading);
      Serial.print("Left Motor Direction = ");
      Serial.print(motors.getDirectionA() ? "F" : "R");
      Serial.print(", Moving = ");
      Serial.print(motors.isMovingA() ? "YES" : "NO");
      Serial.print(", Speed = ");
      Serial.print(motors.getSpeedA());
      Serial.print(", Setpoint = ");
      Serial.println(motor_A_speed);
      // Start New Line
      Serial.print("Right Motor direction = ");
      Serial.print(motors.getDirectionB() ? "F" : "R");
      Serial.print(", Moving = ");
      Serial.print(motors.isMovingB() ? "YES" : "NO");
      Serial.print(", Speed = ");
      Serial.print(motors.getSpeedB());
      Serial.print(", Setpoint = ");
      Serial.println(motor_B_speed);
      // Start new Line
      Serial.print("Left Encoder Count = ");
      Serial.print(encLpulses);
      Serial.print("  Right Encoder Count = ");
      Serial.print(encRpulses);
      Serial.print("  Target = ");
      Serial.println(encEnd);
     // Start New Line
      Serial.print("Kp = ");
      Serial.print(myPID.GetKp());
      Serial.print(" Ki = ");
      Serial.print(myPID.GetKi());
      Serial.print(" Kd = ");
      Serial.print(myPID.GetKd());
      Serial.print(" Mode = ");
      Serial.println(myPID.GetMode());
      Serial.print("Motor A Offset = ");
      Serial.println(motorA_offset);
      previousMillis = millis();                  // update the time we last printed
  }
}
