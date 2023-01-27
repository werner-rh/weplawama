/***
 * Project: Vinyl Record Cleaner (Plattenwaschmaschine)
 * File   : VRCArmMotor.cpp
 * Author : Werner Riemann 
 * Created: 10.08.2022
 * Board: Arduino Nano
 * 
 * Description: Modul for Stepper Motor, Servo and Pump
 * 
 * Pins Steppermotor Saugarm:
 * A0   - IN1
 * A1   - IN2
 * A2   - IN3
 * A3   - IN4
 * 
 */

#include "VRCVacuArm.h"
#include "WRKeyStateDef.h"


VRCVacuArm::VRCVacuArm(/* args */)
{
}


void VRCVacuArm::Setup() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    /* Servo für Lift einrichten */
    servoMotor.attach(liftServoPin, 900, 1500);
    curServoPos = 1;
    servoMotor.write(1);
}


void VRCVacuArm::SetRotateDirection(boolean clockOrCounterClock) {
    Direction = clockOrCounterClock;
}


boolean VRCVacuArm::GetRotateDirection() {
    return Direction;
}


void VRCVacuArm::MotorOff() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

void VRCVacuArm::setNextStep() {
        if (Direction == 1) {
        Steps++;
        currentStepperPos++;
    }

    if (Direction == 0) {
        Steps--;
        currentStepperPos--;
    }

    if (Steps > 7) {
        Steps = 0;
        //MotorOff();
    }

    if (Steps < 0) {
        Steps = 7;
        //MotorOff();
    }
}


void VRCVacuArm::DoSteps(int xw) {

    for (int x = 0; x < xw; x++) {
    // Werte für aktuellen Step aus Tabelle lesen und schreiben
        digitalWrite(IN1, In1Val[Steps]);
        digitalWrite(IN2, In2Val[Steps]);
        digitalWrite(IN3, In3Val[Steps]);
        digitalWrite(IN4, In4Val[Steps]);

        setNextStep();
    }
}


void VRCVacuArm::SetInnerPos(int steppoints) {
    destInnerPos = steppoints;
}

int VRCVacuArm::GetInnerPos() {
    return destInnerPos;
}


void VRCVacuArm::SetBorderPos(int steppoints) {
    destBorderPos = steppoints;
}

int VRCVacuArm::GetBorderPos() {
    return destBorderPos;
}

void VRCVacuArm::SetDelayRangeCounter(uint8_t uindex, uint8_t counterValue) {
    if(uindex <= 4) {
        delayRangeCounter[uindex] = counterValue;
    }
}

uint8_t VRCVacuArm::GetDelayRangeCounter(uint8_t uindex) {
    uint8_t retVal = 25; // default
    if(uindex <= 4) {
        retVal = delayRangeCounter[uindex];
    }

    return retVal;
}

int VRCVacuArm::GetCurrentStepperPos() {
    return currentStepperPos;
}

bool VRCVacuArm::StepToZeroPoint() {
    CheckKeyState(&stepperENDSWITCH_State, STEPPER_ENDSWITCH_PIN);
    if(Direction != DIRECTION_COUNTERCLOCKWISE) {   // Drehrichtung links, von innen nach aussen
        Direction = DIRECTION_COUNTERCLOCKWISE;
    }

    if(stepperENDSWITCH_State != 2) { // Endschalter wird nicht gehalten
        DoSteps(1);
        return false;
    }
    else {
        MotorOff();
        currentStepperPos = 0;
        return true;
    }
}

bool VRCVacuArm::StepZeroToBorder() {
    if(Direction != DIRECTION_CLOCKWISE) {   // Drehrichtung rechts, von aussen nach innen
        Direction = DIRECTION_CLOCKWISE;
    }    

    if(currentStepperPos != destBorderPos) {
        DoSteps(1);
        return false;
    }
    else {
        return true;  // Position erreicht
    }
}

bool VRCVacuArm::StepZeroToInnerPos() {
    if(Direction != DIRECTION_CLOCKWISE) {   // Drehrichtung rechts, von aussen nach innen
        Direction = DIRECTION_CLOCKWISE;
    }  

    if(currentStepperPos != destInnerPos) {
        DoSteps(1);
        return false;
    }
    else {
        return true;  // Position erreicht
    }
}


bool VRCVacuArm::StepInnerToBorderPos() {
    if(Direction != DIRECTION_COUNTERCLOCKWISE) {   // Drehrichtung links, von innen nach aussen
        Direction = DIRECTION_COUNTERCLOCKWISE;
    }

    if(currentStepperPos != destBorderPos) {
        DoSteps(1);
        return false;
    }
    else {
        return true;  // Position erreicht
    }
}

void VRCVacuArm::ResetSpeedRange() {
    speedRange = 1;
}

uint8_t VRCVacuArm::GetSpeedRange() {
    return speedRange;
}

bool VRCVacuArm::IsTimeForNextMove(uint8_t tenMilliSecCounter) {
    bool doStep = false;

    // Bereiche definieren
    if(currentStepperPos > 400 && tenMilliSecCounter >= delayRangeCounter[0]) 
        {doStep = true; speedRange = 1;}
    if(currentStepperPos > 300 && currentStepperPos <= 400 && tenMilliSecCounter >= delayRangeCounter[1])
        {doStep = true; speedRange = 1;}
    if(currentStepperPos > 250 && currentStepperPos <= 300 && tenMilliSecCounter >= delayRangeCounter[2])
        {doStep = true; speedRange = 1;}
    if(currentStepperPos > 200 && currentStepperPos <= 250 && tenMilliSecCounter >= delayRangeCounter[3])
        {doStep = true; speedRange = 1;}
    if(currentStepperPos <= 200 && tenMilliSecCounter >= delayRangeCounter[4])
        {doStep = true; speedRange = 2;}

    return doStep;
}

/***
 * Erhöht die Randposition. Der Arm bewegt sich dabei weiter nach innen. Motor dreht dabei rechts.
 * Die Tasten im Einstellmenü sind dabei gedreht. Taster down=links, Bewegung nach innen.
 */
void VRCVacuArm::IncBorderPos() {
    if(Direction != DIRECTION_CLOCKWISE) {   // Drehrichtung rechts, von aussen nach innen
        Direction = DIRECTION_CLOCKWISE;
    }

    destBorderPos++;
    DoSteps(1); 
}

/***
 * Verinnger die Randposition. Der Arm bewegt sich dabei weiter nach aussen. Motor dreht dabei links.
 * Die Tasten im Einstellmenü sind dabei gedreht. Taster up=rechts, Bewegung nach aussen.
 */
void VRCVacuArm::DecBorderPos() {
    if(Direction != DIRECTION_COUNTERCLOCKWISE) {   // Drehrichtung links, von innen nach aussen
        Direction = DIRECTION_COUNTERCLOCKWISE;
    }

    if(destBorderPos > 0) {
        destBorderPos--;
        DoSteps(1); 
    }
    
}

/***
 * Erhöht die innere Position. Der Arm bewegt sich dabei weiter nach innen. Motor dreht dabei rechts.
 * Die Tasten im Einstellmenü sind dabei gedreht. Taster down=links, Bewegung nach innen.
 */
void VRCVacuArm::IncInnerPos() {
    if(Direction != DIRECTION_CLOCKWISE) {   // Drehrichtung rechts, von aussen nach innen
        Direction = DIRECTION_CLOCKWISE;
    }

    destInnerPos++;
    DoSteps(1); 
}

/***
 * Verinnger die innere Position. Der Arm bewegt sich dabei weiter nach aussen. Motor dreht dabei links.
 * Die Tasten im Einstellmenü sind dabei gedreht. Taster up=rechts, Bewegung nach aussen.
 */
void VRCVacuArm::DecInnerPos() {
    if(Direction != DIRECTION_COUNTERCLOCKWISE) {   // Drehrichtung links, von innen nach aussen
        Direction = DIRECTION_COUNTERCLOCKWISE;
    }

    if(destInnerPos > 0) {
        destInnerPos--;
        DoSteps(1); 
    }
}

// Methoden ArmLift Servo ----------------------------------
void VRCVacuArm::servoStartUp() {
    servoMotor.attach(liftServoPin, 900, 1500);
}

void VRCVacuArm::servoTearDown() {
  servoMotor.detach();
}

void VRCVacuArm::liftUp() {
  destServoPos = servoMaxHight;
}


void VRCVacuArm::liftDown() {
  destServoPos = 1;
}

void VRCVacuArm::incServoMaxHight() {
    servoMaxHight++;
}

void VRCVacuArm::decServoMaxHight() {
    servoMaxHight--;
}

uint8_t VRCVacuArm::getServoMaxHight() {
    return servoMaxHight;
}

void VRCVacuArm::setServoMaxHight(uint8_t maxValue) {
    servoMaxHight = maxValue;
}

bool VRCVacuArm::isLiftDown() {
    if(curServoPos == LIFT_DOWN_POS)
        return true;
    else
        return false;
}

bool VRCVacuArm::isLiftUp() {
    if(curServoPos == servoMaxHight)
        return true;
    else
        return false;
}

void VRCVacuArm::servoLiftStep() {
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
