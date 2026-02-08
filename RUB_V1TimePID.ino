#include <SparkFun_TB6612.h>
#include <Wire.h>
#include <PID_v1.h>
#include <EEPROM.h>
#include <Adafruit_QMC5883P.h>

// --------------- Parameters ---------------
const double BLOCK_SIZE = 50.0;            // Length of one grid square (cm)
const double ENCODER_PER_CM = 60.8475;     // Encoder pulses per cm
const double MOTOR_SPEED_MIN = 50.0;       // Minimum motor speed
const double MOTOR_SPEED_MAX = 220.0;      // Maximum motor speed
double MOTOR_SPEED_MULTIPLIER = 1.2;      // Left motor compensation

const double TURN_TOLERANCE = 1.0;         // Degrees tolerance for turn

// --------------- Arduino Pins ---------------
const int BUTTON_PIN = 11;

// Motor Controller
const int STBY = 7;

// Left Motor
const int PWMA = 4;
const int AIN2 = 5;
const int AIN1 = 6;

// Right Motor
const int BIN1 = 8;
const int BIN2 = 9;
const int PWMB = 10;

// Encoders
const int ENCODER_LEFT_PIN_A = 12;
const int ENCODER_LEFT_PIN_B = 2;
const int ENCODER_RIGHT_PIN_A = 13;
const int ENCODER_RIGHT_PIN_B = 3;

// --------------- Motor Setup ---------------
Motor motorLeft(AIN1, AIN2, PWMA, 1, STBY);
Motor motorRight(BIN1, BIN2, PWMB, 1, STBY);

// Encoders
volatile long encoderLeftPulses = 0;
volatile long encoderRightPulses = 0;
int encoderLeftLastState = 0;
int encoderRightLastState = 0;
bool encoderLeftDirection = true;
bool encoderRightDirection = true;

// --------------- Compass ---------------
float compass = 0.0;
double heading = 0.0;
double trueHeading = 0.0;
int direction_flag = 0;
int revolutions = 0;
double targetHeading = 0.0;
Adafruit_QMC5883P mag;

#define EEPROM_MAGIC 0x42
#define EEPROM_ADDR 0

struct CalData {
  byte magic;
  int16_t offsetX;
  int16_t offsetY;
  int16_t offsetZ;
  float scaleX;
  float scaleY;
  float scaleZ;
};

CalData cal;

// --------------- PID ---------------
double motorSpeedOffset = 0;
double Kp = 1.5, Ki = 1.0, Kd = 0.0;
PID alignPID(&heading, &motorSpeedOffset, &targetHeading, Kp, Ki, Kd, DIRECT);

// --------------- Misc -----------------
int buttonState;
int lastButtonState;
bool beginPath = false;
unsigned long previousMillis = 0;

// ---------------- Function Prototypes -----------------
void fw(double blocks = 2.0, double timeSec = 5.0);
void bw(double blocks = 1.0, double timeSec = 5.0);
void left(double timeSec = 2.0);
void right(double timeSec = 2.0);
void moveDistance(double distance, double timeSec);
void turnDegrees(double degrees, double timeSec);
void hitBrakes();
void readHeading();
void encodersInit();
void encoderLeftCounter();
void encoderRightCounter();
void printDebugInfo();

// ---------------- Main -----------------
void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT);

  encoderLeftPulses = 0;
  encoderRightPulses = 0;
  encodersInit();

  if (!mag.begin()) {
    Serial.println("Compass (QMC5883P) not found");
    while (1);
  }

  mag.setMode(QMC5883P_MODE_NORMAL);
  mag.setODR(QMC5883P_ODR_50HZ);
  mag.setOSR(QMC5883P_OSR_4);
  mag.setRange(QMC5883P_RANGE_8G);

  EEPROM.get(EEPROM_ADDR, cal);
  if (cal.magic != EEPROM_MAGIC) {
    Serial.println("No calibration data");
    while (1);
  }

  alignPID.SetOutputLimits(-50, 50); // small correction for heading
  alignPID.SetSampleTime(50);
  alignPID.SetMode(AUTOMATIC);
}

// ---------------- Loop -----------------
void loop() {
  buttonState = digitalRead(BUTTON_PIN);
  unsigned long currentMillis = millis();

  if (buttonState == HIGH && lastButtonState == LOW) {
    delay(500); // debounce
    beginPath = true;
  }
  lastButtonState = buttonState;

  if (beginPath) {
    fw(1, 10.0);   // Forward 1 block in 5s
    right();   // Turn right in 2s
    fw(2, 3.0);   // Forward 2 blocks in 8s
    beginPath = false;
    

  }

  readHeading();
  printDebugInfo();
}

// ---------------- Movement Primitives -----------------
void fw(double blocks, double timeSec) {
  moveDistance(blocks * BLOCK_SIZE, timeSec);
}

void bw(double blocks, double timeSec) {
  moveDistance(-blocks * BLOCK_SIZE, timeSec);
}

void left(double timeSec = 2.0) { turnDegrees(-90.0, timeSec); }
void right(double timeSec = 2.0) { turnDegrees(90.0, timeSec); }

// ---------------- Move Distance -----------------
void moveDistance(double distance, double timeSec) {
  long targetPulses = abs(distance * ENCODER_PER_CM);
  encoderLeftPulses = 0;
  encoderRightPulses = 0;
  double direction = (distance >= 0) ? 1.0 : -1.0;
  unsigned long startTime = millis();

  while (true) {
    unsigned long elapsedMs = millis() - startTime;
    double elapsedSec = elapsedMs / 1000.0;
    double progress = constrain(elapsedSec / timeSec, 0.0, 1.0);

    // Target pulses based on elapsed time
    long targetPulsesNow = targetPulses * progress;

    // Average encoder pulses
    long avgPulses = (abs(encoderLeftPulses) + abs(encoderRightPulses)) / 2;

    // Error in pulses
    double error = targetPulsesNow - avgPulses;

    // Smooth proportional power
    double power = constrain(error * 2.0, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);

    // Heading correction
    readHeading();
    alignPID.Compute();

    motorLeft.drive((power * MOTOR_SPEED_MULTIPLIER + motorSpeedOffset) * direction);
    motorRight.drive((power + motorSpeedOffset) * direction);

    if (progress >= 1.0) break;
    delay(10);
  }

  hitBrakes();
  delay(200);
}

// ---------------- Turn -----------------
void turnDegrees(double degrees, double timeSec) {
  readHeading();
  double direction = (degrees >= 0) ? 1.0 : -1.0;
  double startHeading = trueHeading;
  double target = startHeading + degrees;
  unsigned long startTime = millis();

  while (true) {
    readHeading();
    double elapsedSec = (millis() - startTime) / 1000.0;
    double progress = constrain(elapsedSec / timeSec, 0.0, 1.0);
    double desiredHeading = startHeading + degrees * progress;

    // Heading error
    double error = desiredHeading - heading;
    double power = constrain(abs(error) * 2.0, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);

    if (error > 0) {
      motorLeft.drive(power * MOTOR_SPEED_MULTIPLIER);
      motorRight.drive(-power);
    } else {
      motorLeft.drive(-power * MOTOR_SPEED_MULTIPLIER);
      motorRight.drive(power);
    }

    if (progress >= 1.0 || abs(error) <= TURN_TOLERANCE) break;
    delay(10);
  }

  hitBrakes();
  delay(200);
}

// ---------------- Brake -----------------
void hitBrakes() {
  motorLeft.drive(0);
  motorRight.drive(0);
}

// ---------------- Compass -----------------
void readHeading() {
  int16_t x, y, z;
  if (mag.getRawMagnetic(&x, &y, &z)) {
    float xf = (x - cal.offsetX) * cal.scaleX;
    float yf = (y - cal.offsetY) * cal.scaleY;
    float zf = (z - cal.offsetZ) * cal.scaleZ;

    compass = atan2(yf, xf) * 180.0 / PI;
    if (compass < 0) compass += 360;
  }

  if (direction_flag == 1 && compass < 180.0) {
    revolutions += 1;
    direction_flag = 0;
  } else if (direction_flag == -1 && compass > 180) {
    revolutions -= 1;
    direction_flag = 0;
  }

  if (compass < 100.0) direction_flag = -1;
  else if (compass > 260.0) direction_flag = 1;
  else direction_flag = 0;

  trueHeading = (revolutions * 360) + compass;
  heading += (trueHeading - heading) * 0.2; // smoothing
}

// ---------------- Encoder Functions -----------------
void encodersInit() {
  encoderLeftDirection = true;
  encoderRightDirection = true;

  pinMode(ENCODER_LEFT_PIN_A, INPUT);
  pinMode(ENCODER_LEFT_PIN_B, INPUT);
  pinMode(ENCODER_RIGHT_PIN_A, INPUT);
  pinMode(ENCODER_RIGHT_PIN_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN_B), encoderLeftCounter, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN_B), encoderRightCounter, CHANGE);
}

void encoderLeftCounter() {
  int leftStateA = digitalRead(ENCODER_LEFT_PIN_A);

  if (encoderLeftLastState == LOW && leftStateA == HIGH) {
    int leftStateB = digitalRead(ENCODER_LEFT_PIN_B);

    if (leftStateB == LOW && encoderLeftDirection) encoderLeftDirection = false;
    else if (leftStateB == HIGH && !encoderLeftDirection) encoderLeftDirection = true;
  }

  encoderLeftLastState = leftStateA;

  if (encoderLeftDirection) encoderLeftPulses++;
  else encoderLeftPulses--;
}

void encoderRightCounter() {
  int rightStateA = digitalRead(ENCODER_RIGHT_PIN_A);
  int rightStateB;

  if (encoderRightLastState == LOW && rightStateA == HIGH) {
    rightStateB = digitalRead(ENCODER_RIGHT_PIN_B);

    if (rightStateB == LOW && encoderRightDirection) encoderRightDirection = false;
    else if (rightStateB == HIGH && !encoderRightDirection) encoderRightDirection = true;
  }

  encoderRightLastState = rightStateA;

  if (encoderRightDirection) encoderRightPulses++;
  else encoderRightPulses--;
}

// ---------------- Debug -----------------
void printDebugInfo() {
  long avgPulses = (abs(encoderLeftPulses) + abs(encoderRightPulses)) / 2;
  double avgDistance = avgPulses / ENCODER_PER_CM;

  Serial.print("Heading: "); Serial.print(heading);
  Serial.print(" | TrueHeading: "); Serial.print(trueHeading);
  Serial.print(" | AvgDist: "); Serial.print(avgDistance);
  Serial.print(" | EncL: "); Serial.print(encoderLeftPulses);
  Serial.print(" | EncR: "); Serial.println(encoderRightPulses);
}
