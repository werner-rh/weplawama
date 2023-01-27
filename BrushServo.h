/***
 * Project: Record Cleaner
 * File   : BrushServo.h
 * Author : Werner Riemann 
 * Created: 03.10.2022
 * Board: Arduino Nano
 * 
 * Description: Class definition for controlling the brush servo lif
 * 
 * Pins : D11
 * 
 */
 
 #include <Arduino.h>
 #include <Servo.h>

#define BRUSH_SERVO_PIN 11
#define BRUSH_SERVO_MINPOS 1
#define BRUSH_SERVO_MAXPOS 136

 class BrushServo
 {
 private:
    /* data */
    Servo servoMotor;
    uint8_t servoPin = BRUSH_SERVO_PIN;
    uint8_t curServoPos=0;             // current servo position in degree
    uint8_t destServoPos=0;
    uint8_t servoMaxHight=BRUSH_SERVO_MAXPOS;

 public:
    void Setup();

    void servoTearDown();
    void servoStartUp();
    void liftUp();                  // Set destination for liftUp Vacuumarm, destination 90 degree
    void liftDown();                // Set destination for liftUp Vacuumarm, destination 0 degree
    bool isLiftDown();              // check ob der Lift unten ist
    bool isLiftUp();                // Check ob der Lift oben ist
    void servoLiftStep();               // increases or decreases the Servo angle for one degree, 
                                  // depending on the direction up or down
 };
 
 
 