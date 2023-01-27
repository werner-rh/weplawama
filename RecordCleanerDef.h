/***
 *  RecordCleanerDef.h 
 *  Contains constant definitons and Prototypes 
 */

#ifndef _RECORDCLEANERDEF_H
  #define _RECORDCLEANERDEF_H

#define FWVERSION "1.11"       // 1. Digit: Major Version, 2. + 3 Digit: Minor Version - eg.: 100 = 1.00

#include <Arduino.h>
// defines für PINs -----------------------

#define startBUTTONPin 1
#define backBUTTONPin 2
#define downBUTTONPin 3
#define upBUTTONPin 4

#define RECMOTOR_DIRECTION_PIN 6
#define RECMOTOR_STEPSIGNAL_PIN 7
#define RECMOTOR_SLEEP_PIN 9
#define RELAIS_VACTURBINE_PIN 8


#define SERVO_BRUSH_PIN 11


// Defines Statemachine ----------------------------------------------
#define STATE_INIT         1

#define AST_IDLE           0
#define AST_STARTUP        1
#define AST_MAINMENU      10
#define AST_WASHMENU      11
#define AST_VACMENU       12
#define AST_SETUPMENU     13

#define AST_RUNWASHING    20
#define AST_RUNVACUCLEAN  25
#define AST_SETUPSELECT   30

#define AST_ADJUSTPART    40
#define AST_ADJUSTLIFT    41
#define AST_ADJUSTBORDER  42
#define AST_ADJUSTINNER   43
#define AST_ADJUSTRANGE   44
#define AST_ADJUSTSAVE    45

#define AST_RUN_BORDER_SET 46
#define AST_RUN_INNER_SET  47
#define AST_RUN_RANGE_SET  48       // Delaybereiche für Saugarm einstellen

// State Defines für Unterroutinen --------------
#define WST_RUN           2
#define WST_WAITREVERS    3
#define WST_RUNREVERS     4
#define WST_WAITFORWARD   5
#define WST_FORWARD       6
#define WST_BRUSHLIFTUP   8

// State Defines für Adjust Ranges --------------
#define RST_ADJUST_RANGES 2
#define RST_EDIT_RANGE    3

// State Defines für Adjust Border --------------
#define BST_WAIT_LIFTUP         2
#define BST_GOTO_BORDER         3
#define BST_WAIT_BORDER         4
#define BST_ADJUST_BORDER       5
#define BST_WAIT_RET_LIFTUP     6
#define BST_WAIT_STEPPER_ZERO   7      // alles in Grundstellung fahren
#define BST_WAIT_LIFTDOWN       8

// State Defines für Absaugung --------------
#define VST_WAIT_LIFTUP         2
#define VST_GOTO_INNER          3      // Arm nach innen fahren
#define VST_WAIT_INNER          4      // Arm inne, Lift senken
#define VST_WAIT_LIFTDOWN       5      // Lift unten,
#define VST_VAC_INNER_TO_BORDER 6      // von innen nach aussen absaugen
#define VST_WAIT_RET_LIFTUP     7
#define VST_WAIT_STEPPER_ZERO   8      // alles in Grundstellung fahren
#define VST_WAIT_RET_LIFTDOWN   9
#define VST_STEP_DELAY          10     // Verzögerung zwischen einzelnen Steps


// Prototypen in RecordCleaner.ino
void EEPROMWriteInt(int p_address, int p_value);
unsigned int EEPROMReadInt(int p_address);
void SaveSettings();
void ReadSettings();

// Unterroutinen der Statemachine
uint8_t RunAdjustArmRanges(uint8_t startParam);
uint8_t RunAdjustBorder(uint8_t startParam);
uint8_t RunAdjustInner(uint8_t startParam);
uint8_t RunVacuClean(uint8_t startParam);
uint8_t RunWashing(uint8_t stateInit, bool breakSignal);


// Prototypen in anderen Dateien ----------





#endif
