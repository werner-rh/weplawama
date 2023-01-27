/***
 * Project: Record Cleaner
 * File   : BrushServo.cpp
 * Author : Werner Riemann 
 * Created: 03.10.2022
 * Board: Arduino Nano
 * 
 * Description: Class implementation for controlling the brush servo lift
 * 
 * Pins : D11
 * 
 */

#include "BrushServo.h"



 void BrushServo::Setup()
 {
    servoMotor.attach(servoPin, 900, 1500);
    curServoPos = BRUSH_SERVO_MAXPOS;
    destServoPos = servoMaxHight;
    servoMotor.write(BRUSH_SERVO_MAXPOS);  
    //servoMotor.detach(); // damit das Servo nicht rumwackelt
 }

 void BrushServo::servoStartUp() {
    servoMotor.attach(servoPin, 900, 1500);
}

void BrushServo::servoTearDown() {
  servoMotor.detach();
}

void BrushServo::liftUp() {
  destServoPos = servoMaxHight;
}


void BrushServo::liftDown() {
  destServoPos = BRUSH_SERVO_MINPOS;
}

bool BrushServo::isLiftDown() {
    if(curServoPos == BRUSH_SERVO_MINPOS)
        return true;
    else
        return false;
}

bool BrushServo::isLiftUp() {
    if(curServoPos == servoMaxHight)
        return true;
    else
        return false;
}

void BrushServo::servoLiftStep() {
  uint8_t oldPos = curServoPos;
  if(curServoPos < destServoPos) {
    curServoPos ++;
  }

  if(curServoPos > destServoPos) {
    curServoPos --;
  }

  if(oldPos != curServoPos) {
    servoMotor.write(curServoPos);
  } 
  
}