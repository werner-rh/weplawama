/***
 * Project: NemaSimpleStep
 * File   : NemaSimpleStep.cpp
 * Author : Werner Riemann 
 * Created: 20.09.2022
 * Board: Arduino Nano
 * 
 * Description: implements the class
 * 
 * Pins : DirectionPin, StepPin and SleepPin
 * 
 */

#include "NemaSimpleStep.h"

NemaSimpleStep::NemaSimpleStep()
{
    ActionState = STPST_IDLE;
    motorSignal = 0;
}

NemaSimpleStep::~NemaSimpleStep()
{
}


void NemaSimpleStep::Setup(uint8_t dirPin, uint8_t stepPin, uint8_t sleepPin, int stepsPerRevolution)
{
    directionPin = dirPin;
    steppingPin = stepPin;
    sleepingPin = sleepPin;
    fullRotateSteps = stepsPerRevolution;
    curDirection = STEPDIR_CLOCKWISE;

    pinMode(steppingPin, OUTPUT);
    pinMode(directionPin, OUTPUT);
    pinMode(sleepingPin, OUTPUT);
    digitalWrite(directionPin, STEPDIR_CLOCKWISE);
    digitalWrite(sleepingPin, LOW);  // default motor off
}

void NemaSimpleStep::SetDirection(uint8_t direction)
{
    curDirection = direction;
    digitalWrite(directionPin, curDirection);
}

void NemaSimpleStep::SetSpeed(uint8_t spdelay)
{
    speedDelay = spdelay;
}

void NemaSimpleStep::DecSpeed()
{
    speedDelay ++;
}

int NemaSimpleStep::GetRotations()
{
    return rotationsDone;
}

void NemaSimpleStep::StopMotor()
{
    digitalWrite(sleepingPin, LOW);
    ActionState = STPST_IDLE;
}

void NemaSimpleStep::RunMotor()
{
    digitalWrite(sleepingPin, HIGH);
    rampupCounter = RAMP_START_COUNT;
    rampupDelay = RAMP_START_DELAY;
    //ActionState = STPST_RUN;
    ActionState = STPST_RAMPUP;
    motorSignal = 0;
    rotationsDone = 0;
    stepsDone = 0;
}


void NemaSimpleStep::MotorAction()
{

    switch (ActionState)
    {
    case STPST_IDLE:
        /* code */
        break;
    
    case STPST_RAMPUP:
        if(motorSignal == 0) {
        
        
            motorSignal=1;
            digitalWrite(steppingPin, HIGH);
            delayCounter = rampupDelay;
            stepsDone++;
            if(stepsDone >= fullRotateSteps) {
                rotationsDone++;
                stepsDone = 0;
            }
        } 
        else {
            digitalWrite(steppingPin, LOW);
            if(delayCounter > 0) {
                delayCounter --;
            } 
            else {
                motorSignal=0;
                
                if(rampupCounter > 0) {
                    rampupCounter--;
                }
                else {
                    if(rampupDelay > speedDelay) {
                        rampupDelay --;
                        rampupCounter = RAMP_START_COUNT;
                    }
                    else {
                        ActionState = STPST_RUN;
                    }
                    
                }
            }
        }
            
        break;

    case STPST_RUN:
        if(motorSignal == 0) {
            motorSignal=1;
            digitalWrite(steppingPin, HIGH);
            delayCounter = speedDelay;
            stepsDone++;
            if(stepsDone >= fullRotateSteps) {
                rotationsDone++;
                stepsDone = 0;
            }
        } 
        else {
            digitalWrite(steppingPin, LOW);
            if(delayCounter > 0) {
                delayCounter --;
            } 
            else {
                motorSignal=0;
            }
        }
        
        break;
    
    }
}
