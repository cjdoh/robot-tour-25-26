//#include <L298NX2.h>
#include <SparkFun_TB6612.h>
#include <Wire.h>
#include <PID_v1.h>
#include "JY901_Serial.h"


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

  if(encRdir) encRpulses--;
  else encRpulses++;
}


void hit_breaks(){

  brake(motorLeft,motorRight);

}

