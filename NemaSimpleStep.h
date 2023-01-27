/***
 * Project: NemaSimpleStep
 * File   : NemaSimpleStep.h
 * Author : Werner Riemann 
 * Created: 20.09.2022
 * Board: Arduino Nano
 * 
 * Description: Class definition for simple controlling a Nema17 Steppermotor
 *              using a A4988 driver board. The class is only to rotate the
 *              step motor in different directions and different speeds.
 * 
 * Pins : DirectionPin, StepPin and SleepPin
 * 
 */
 
 #include <Arduino.h>

#define STEPDIR_CLOCKWISE 1
#define STEPDIR_COUNTERCLOCKWISE 0 
#define RAMP_START_DELAY 5
#define RAMP_START_COUNT 60

#define STPST_IDLE 0
#define STPST_RAMPUP 2
#define STPST_RUN  10

class NemaSimpleStep
{
private:
    /* data */
    uint8_t directionPin;
    uint8_t steppingPin;
    uint8_t sleepingPin;
    int fullRotateSteps;
    int stepsDone;
    int rotationsDone;
    uint8_t motorSignal;
    uint8_t speedDelay;
    uint8_t delayCounter;
    uint8_t rampupCounter;
    uint8_t rampupDelay;

    uint8_t curDirection;
    uint8_t ActionState;

public:
    NemaSimpleStep();
    ~NemaSimpleStep();

    void Setup(uint8_t dirPin, uint8_t stepPin, uint8_t sleepPin, int stepsPerRevolution);

    void SetDirection(uint8_t direction);
    void SetSpeed(uint8_t spdelay);
    void DecSpeed();
    int GetRotations();
    void StopMotor();
    void RunMotor();
    
    void MotorAction();
};



