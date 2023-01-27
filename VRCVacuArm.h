/***
 * Project: Vinyl Record Cleaner (Plattenwaschmaschine)
 * File   : VRCArmMotor.h
 * Author : Werner Riemann 
 * Created: 10.08.2022
 * Board: Arduino Nano
 * 
 * Description: Class definition for VacArm Stepper Motor
 * 
 * Pins :
 * 
 */

#include <Arduino.h>
#include <Servo.h>
#include "WRKeyStateDef.h"

// defines for motor pins
#define IN1 A0
#define IN2 A1
#define IN3 A2
#define IN4 A3
#define STEPPER_ENDSWITCH_PIN 5               // Endschalter für Arm Nullpunkt auf der Ablage
#define LIFT_SERVO_SIGNAL_PIN 12

#define DIRECTION_CLOCKWISE true
#define DIRECTION_COUNTERCLOCKWISE false

#define MAX_ARM_CLOCKWISE_POS 1024           // entspricht einer Drehung um 90 Grad bzw. viertel Kreis zum innern
#define LIFT_DOWN_POS 1                      // Wert für List unten (wir fahren das Servo nicht ganz auf 0)

class VRCVacuArm
{
private:
    /* data SteppMotor */
    boolean Direction = true;   // true = Motor dreht rechts herum, fals = links
    int Steps = 0;
    uint8_t In1Val[8] = {LOW, LOW, LOW, LOW, LOW, HIGH, HIGH,  HIGH};
    uint8_t In2Val[8] = {LOW, LOW, LOW, HIGH, HIGH, HIGH, LOW, LOW};
    uint8_t In3Val[8] = {LOW, HIGH, HIGH, HIGH, LOW, LOW, LOW, LOW};
    uint8_t In4Val[8] = {HIGH, HIGH, LOW, LOW, LOW, LOW, LOW,  HIGH};
    int currentStepperPos=0;
    int destInnerPos = 500;      // default viertel Kreis 1024, die Position im Platteninnern beträgt aber ca. 500 Steps vom Nullpunkt
    int destBorderPos = 168;      // default äusserer Rand der Platte bei etwa 160 Steps vom Nullpunkt.

    // Zeiten bzw. Counter für Verzögerung der Armbewegung beim Absaugen
    uint8_t delayRangeCounter[5] = {30,40,60,80,100};
    uint8_t speedRange = 1;

    uint8_t stepperENDSWITCH_State=0;   // Status Endschalter Armnullpunkt

    /* Data Servo ArmLift*/
    Servo servoMotor;
    uint8_t liftServoPin=LIFT_SERVO_SIGNAL_PIN;
    uint8_t curServoPos=0;             // current servo position in degree
    uint8_t destServoPos=0;
    uint8_t servoMaxHight=42;


    /* Methods */
    void setNextStep();

public:
    VRCVacuArm(/* args */);
    void Setup();
    void SetRotateDirection(boolean clockOrCounterClock);
    boolean GetRotateDirection();
    void MotorOff();
    void SetInnerPos(int steppoints);
    void SetBorderPos(int steppoints);
    void SetDelayRangeCounter(uint8_t uindex, uint8_t counterValue);
    uint8_t GetDelayRangeCounter(uint8_t uindex);
    uint8_t GetSpeedRange();
    void ResetSpeedRange();
    void DoSteps(int xw);
    int GetCurrentStepperPos();
    int GetInnerPos();
    int GetBorderPos();
    bool StepToZeroPoint();
    bool StepZeroToBorder();
    bool StepZeroToInnerPos();
    bool StepInnerToBorderPos();
    bool IsTimeForNextMove(uint8_t tenMilliSecCounter);

    void IncBorderPos();
    void DecBorderPos();
    void IncInnerPos();
    void DecInnerPos();

    /* Methoden LiftServo */
    void servoTearDown();
    void servoStartUp();
    uint8_t getServoMaxHight();
    void setServoMaxHight(uint8_t maxValue);
    void incServoMaxHight();
    void decServoMaxHight();
    void liftUp();                  // Set destination for liftUp Vacuumarm, destination 90 degree
    void liftDown();                // Set destination for liftUp Vacuumarm, destination 0 degree
    bool isLiftDown();              // check ob der Lift unten ist
    bool isLiftUp();                // Check ob der Lift oben ist
    void servoLiftStep();               // increases or decreases the Servo angle for one degree, 
                                  // depending on the direction up or down
};


