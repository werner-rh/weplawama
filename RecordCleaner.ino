/***
 * Project: Vinyl Record Cleaner (Plattenwaschmaschine)
 * Author : Werner Riemann 
 * Created: 23.07.2022
 * Board: Arduino Nano
 * 
 * Description: Steuerung für Plattenwaschmaschine
 * 
 * Pins:
 * A0-A3  - Steppermotor Saugarm
 * A4,A5  - I2C Display control (A4 - SDA, A5 - SCL)
 * 
 * 

 * D1     - Digital in Taster 1 - startButtonPin       - Start, Enter, Auswahl
 * D2     - Digital in Taster 1 - backBUTTONPin      - Back, Setup, save
 * D3     - Digital in Taster 2 - downBUTTONPin , down or move counter clockwise, left
 * D4     - Digital in Taster 3 - upBUTTONPin , up or move clockwise, right
 *
 * D5     - Digital in Endlagenschalter für Nullpunkt Steppermotor, gleichzeitig Ablageposition
 *                                      für Saugarm auf Auflagestütze. 
 * 
 *        Relais - die verwendeten Relais sind alle LOW aktive, Low -> Relais zieht an
 *        ---------------------------------------------------------------------------
 * D6     - Direction Pin Stepper Controller
 * D7     - StepPin Stepper Controller
 * D8     - Relais Saugturbine (Plus-Leitung)
 * D9     - SleepPin Stepper Controller
 * 
 * D11    - Servo Bürstenarm 
 * D12    - Servo Saugarmlift
 * D13    - Digital out LEDPIN 13 (interne LED)
 * 
 */


#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>

#include "VRCVacuArm.h"
#include "VRCDisplay.h"
#include "NemaSimpleStep.h"
#include "BrushServo.h"
#include "RecordCleanerDef.h"
#include "WRKeyStateDef.h"

#define F_CPU 16000000UL  // 16 MHz

//--- globale Daten, Variablen ----------------------
volatile uint8_t B100HzToggle = 0;  // 50 Hertz Signal
uint8_t ui10MilliSekCount = 0;

VRCVacuArm vacuArm;
VRCDisplay display;
NemaSimpleStep recMotor;
BrushServo brushArm;

//-- Statusvaribalen für Taster ----
volatile  uint8_t startBUTTON_State=0;
volatile  uint8_t backBUTTON_State=0;
volatile  uint8_t downBUTTON_State=0;
volatile  uint8_t upBUTTON_State=0;


// Interrupt is called once a millisecond, 
SIGNAL(TIMER0_COMPA_vect) 
{
  unsigned long currentMillis = millis();
  ui10MilliSekCount ++;

  if(ui10MilliSekCount >= 10 ) {
    ui10MilliSekCount = 0;
    B100HzToggle ^= 1;
  }
  // trigger the record stepper motor action
  recMotor.MotorAction();
}


void setup() {
  // setup in- and output pins ------------
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(STEPPER_ENDSWITCH_PIN, INPUT);
  pinMode(startBUTTONPin, INPUT);
  pinMode(backBUTTONPin, INPUT);
  pinMode(downBUTTONPin, INPUT);
  pinMode(upBUTTONPin, INPUT);
  digitalWrite(STEPPER_ENDSWITCH_PIN, HIGH);    // Pullup resistor on
  digitalWrite(startBUTTONPin, HIGH);      // Pullup resistor on
  digitalWrite(backBUTTONPin, HIGH);    // Pullup resistor on
  digitalWrite(downBUTTONPin, HIGH);      // Pullup resistor on
  digitalWrite(upBUTTONPin, HIGH);        // Pullup resistor on

  // Relais - für nicht angezogen muss output auf HIGH
  // Das neue Relais für Turbine ist HIGH aktiv - anzug bei High!
  pinMode(RELAIS_VACTURBINE_PIN, OUTPUT);
  digitalWrite(RELAIS_VACTURBINE_PIN, LOW);   // Turbine aus

  display.Setup();
  vacuArm.Setup();
  recMotor.Setup(RECMOTOR_DIRECTION_PIN, RECMOTOR_STEPSIGNAL_PIN, RECMOTOR_SLEEP_PIN, 800);
  brushArm.Setup();

  display.ScreenOut(SCR_WELCOME);
  // Timer setup --------------------------
  // Timer0 is already used for millis() - we'll just interrupt somewhere
  // in the middle and call the "Compare A" function below
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
}


// Einstellungen speichern und lesen --------------------
//This function will write a 2 byte integer to the eeprom at the specified address and address
void EEPROMWriteInt(int p_address, int p_value)
	{
	byte lowByte = ((p_value >> 0) & 0xFF);
	byte highByte = ((p_value >> 8) & 0xFF);

	EEPROM.write(p_address, lowByte);
	EEPROM.write(p_address + 1, highByte);
	}

//This function will read a 2 byte integer from the eeprom at the specified address and address + 1
unsigned int EEPROMReadInt(int p_address)
	{
	byte lowByte = EEPROM.read(p_address);
	byte highByte = EEPROM.read(p_address + 1);

	return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
	}

void SaveSettings() {
  EEPROM.write(0, vacuArm.getServoMaxHight()); // Max Lifthöhe auf Addresse 0
  EEPROMWriteInt(1, vacuArm.GetBorderPos());   // BorderPos startet auf Adresse 1, belegt 1+2
  EEPROMWriteInt(3, vacuArm.GetInnerPos());   // InnerPos startet auf Adresse 3, belegt 3+4
  // Verzögerungscounter der einzelnen Plattenbereiche (5) - beginnend ab Adresse 5 bis Adresse 9
  EEPROM.write(5, vacuArm.GetDelayRangeCounter(0));
  EEPROM.write(6, vacuArm.GetDelayRangeCounter(1));
  EEPROM.write(7, vacuArm.GetDelayRangeCounter(2));
  EEPROM.write(8, vacuArm.GetDelayRangeCounter(3));
  EEPROM.write(9, vacuArm.GetDelayRangeCounter(4));
}


void ReadSettings() {
  vacuArm.setServoMaxHight(EEPROM.read(0));    // Max Lifthöhe von Adresse 0 lesen
  vacuArm.SetBorderPos(EEPROMReadInt(1));      // BorderPos von Adresse 1 + 2 lesen
  vacuArm.SetInnerPos(EEPROMReadInt(3));       // InnerPos von Addresse 3 + 4 lesen
  // Verzögerungscounter der einzelnen Plattenbereiche (5) - beginnend ab Adresse 5 bis Adresse 9
  vacuArm.SetDelayRangeCounter(0, EEPROM.read(5));
  vacuArm.SetDelayRangeCounter(1, EEPROM.read(6));
  vacuArm.SetDelayRangeCounter(2, EEPROM.read(7));
  vacuArm.SetDelayRangeCounter(3, EEPROM.read(8));
  vacuArm.SetDelayRangeCounter(4, EEPROM.read(9));
}

/***
 * Hauptschleife des Programms. Damit diese nicht zu unübersichtlich wird,
 * werden einzelne Teile in weiteren Funktionen ausgelagert. Die Abfrage
 * die Taster zur Bedienung werden immer hier in der Hauptschleife abgefragt.
 * Den jeweiligen Status können die Unterroutinen in den globalen Variablen
 * abrufen.
 */
void loop() {
  // put your main code here, to run repeatedly:
  static uint8_t AppState=0;
  static uint8_t StateTrigger = 0;
  uint8_t aktStateTrigger;
  uint8_t washDone =0;

  static uint8_t mainMenuNo = 0;
  static uint8_t setupMenuNo = 0;
  static uint8_t autoClean = 0;

  static uint8_t ui100HzSecCounter=0;   // Zähler für den Ablauf einer Sekunde

  aktStateTrigger = B100HzToggle;
  
  // Abarbeitung State-Machine. ----------------------------------------------------------
  // Die State-Machine wird nur bei einem Flankenwechsel, 100 mal je Sekunde, durchlaufen.

  if(aktStateTrigger != StateTrigger) {
    StateTrigger = aktStateTrigger;

    // Taster abfragen
    CheckKeyState(&startBUTTON_State, startBUTTONPin);
    CheckKeyState(&backBUTTON_State, backBUTTONPin);
    CheckKeyState(&downBUTTON_State, downBUTTONPin);
    CheckKeyState(&upBUTTON_State, upBUTTONPin);

    // App-State-Machine abarbeiten und weiterschalten
    switch(AppState) {

      case AST_IDLE:    AppState=AST_STARTUP;
        break;

      case AST_STARTUP: 
        display.ScreenOut(SCR_WELCOME);
        ReadSettings();
        brushArm.servoTearDown();
        autoClean = 0;
        // vorher auf Tastendruck warten
        if(upBUTTON_State == 1) {
          mainMenuNo = 1;
          display.ScreenOut(SCR_WELCOME + mainMenuNo);
          AppState=AST_MAINMENU;  
        } 
        break;

      case AST_MAINMENU: 
         if(upBUTTON_State  == 1 && mainMenuNo < 4) {
          mainMenuNo ++;
          display.ScreenOut(SCR_WELCOME + mainMenuNo);
         }
        
        if(downBUTTON_State == 1 && mainMenuNo > 1) {
          mainMenuNo --;
          display.ScreenOut(SCR_WELCOME + mainMenuNo);
        }

        if(backBUTTON_State == 1) {
          AppState=AST_STARTUP;
        }

        // Menu actions -----
        if(startBUTTON_State == 1 && mainMenuNo == 1) { // AutoClean -> waschen, absaugen
          autoClean = 1;
          AppState=AST_WASHMENU;
          display.ScreenOut(SCR_WASHMENU);
        }

        if(startBUTTON_State == 1 && mainMenuNo == 2) {
          AppState=AST_WASHMENU;
          display.ScreenOut(SCR_WASHMENU);
        }

        if(startBUTTON_State == 1 && mainMenuNo == 3) {  // Absaugen
          AppState=AST_VACMENU;
          display.ScreenOut(SCR_WELCOME + mainMenuNo);
        }

        if(startBUTTON_State == 1 && mainMenuNo == 4) { // Auswahl Einstellungen 
          setupMenuNo = 1;
          AppState=AST_SETUPSELECT;
          display.ScreenOut(SCR_SETUPSELECT + setupMenuNo);
        }
        break;        

      case AST_WASHMENU:
        if(backBUTTON_State == 1) {
          display.ScreenOut(SCR_WELCOME + mainMenuNo);
          AppState=AST_MAINMENU;
        }

        if(upBUTTON_State  == 1)
          display.IncWashTime();

        if(downBUTTON_State == 1)
          display.DecWashTime();

        // Waschen starten, wenn Start Button gedrückt wurde
        if(startBUTTON_State == 1) {
          AppState=AST_RUNWASHING;
          RunWashing(STATE_INIT, false);
        }

        break;

      case AST_VACMENU:
        RunVacuClean(STATE_INIT);
        display.ScreenOut(SCR_VACUPROCESS);
        AppState=AST_RUNVACUCLEAN;
        break;

      case AST_RUNVACUCLEAN:
        if(RunVacuClean(0) == 1) {
          //mainMenuNo = 2;
          display.ScreenOut(SCR_WELCOME + mainMenuNo);
          AppState=AST_MAINMENU;
          autoClean=0;
        }
        break;

      case AST_RUNWASHING:  // Waschen ausführen, Motor mit Laufrichtungswechseln für die eingestellte Zeit drehen
        // das funktioniert hier so nicht meht
        if(backBUTTON_State == 1) {
          //AppState=AST_STARTUP;
          RunWashing(0,true); // Signal Waschen abbrechen
          autoClean = 0;
        }

        washDone = RunWashing(0, false);
        if(washDone == 1) {
          //mainMenuNo = 2;
          display.IncWashCounter();
          display.ScreenOut(SCR_WELCOME + mainMenuNo);
          AppState=AST_MAINMENU;
          if(autoClean == 1) {
            AppState=AST_VACMENU;
          }
        }

        break; //AST_RUNWASHING

      case AST_SETUPSELECT:
        if(backBUTTON_State == 1) {
          display.ScreenOut(SCR_WELCOME + mainMenuNo);
          AppState=AST_MAINMENU;
        }

        if(upBUTTON_State  == 1 && setupMenuNo < 5) {
          setupMenuNo ++;
          display.ScreenOut(SCR_SETUPSELECT + setupMenuNo);
         }

        if(downBUTTON_State  == 1 && setupMenuNo > 1) {
          setupMenuNo --;
          display.ScreenOut(SCR_SETUPSELECT + setupMenuNo);
         }

        if(startBUTTON_State == 1) {
          AppState=AST_ADJUSTPART + setupMenuNo;
         }

        if(vacuArm.getServoMaxHight() !=0) {
          vacuArm.servoLiftStep();
        }
        break;

      case AST_ADJUSTLIFT:
        vacuArm.liftUp();
        vacuArm.servoLiftStep();
        display.UpdateSetupValue(vacuArm.getServoMaxHight());
        if(upBUTTON_State  == 1) {
          vacuArm.incServoMaxHight();
        }

        if(downBUTTON_State  == 1) {
          vacuArm.decServoMaxHight();
        }

        if(backBUTTON_State == 1) {
          vacuArm.liftDown();
          vacuArm.servoLiftStep();
          AppState = AST_SETUPSELECT;
          display.ScreenOut(SCR_SETUPSELECT + setupMenuNo);
        }
        break;

      case AST_ADJUSTBORDER:
        display.ScreenOut(SCR_SETUPSELECT + setupMenuNo);
        RunAdjustBorder(STATE_INIT);
        AppState = AST_RUN_BORDER_SET;
        break;

      case AST_RUN_BORDER_SET:
        if(RunAdjustBorder(0) == 1) {  // Einstellen fertig/abgebrochen
          AppState = AST_SETUPSELECT;
          display.ScreenOut(SCR_SETUPSELECT + setupMenuNo);
        }
        break;

      case AST_ADJUSTINNER:
        display.ScreenOut(SCR_SETUPSELECT + setupMenuNo);
        RunAdjustInner(STATE_INIT);
        AppState = AST_RUN_INNER_SET;
        break;

      case AST_RUN_INNER_SET:
        if(RunAdjustInner(0) == 1) {  // Einstellen fertig/abgebrochen
          AppState = AST_SETUPSELECT;
          display.ScreenOut(SCR_SETUPSELECT + setupMenuNo);
        }
        break;

      // Delaybereiche für SaugArmbewegung
      case AST_ADJUSTRANGE:
        display.ScreenOut(SCR_SETUPSELECT + setupMenuNo);
        RunAdjustArmRanges(STATE_INIT);
        AppState = AST_RUN_RANGE_SET;
        break;

      case AST_RUN_RANGE_SET:
        if(RunAdjustArmRanges(0) == 1) { // Einstellen Ranges fertig/abgebrochen
          AppState = AST_SETUPSELECT;
          display.ScreenOut(SCR_SETUPSELECT + setupMenuNo);
        }
        break;

      case AST_ADJUSTSAVE:
        display.ShowShortMsg(">saved");
        SaveSettings();
        delay(500);
        AppState = AST_SETUPSELECT;
        display.ScreenOut(SCR_SETUPSELECT + setupMenuNo);
        break;
    } // Ende State-Machine
        
  }    // Ende Flankenwechsel
 
}


// --- Unterroutinen der Statemachine ---
// Die Taktung der Unterroutinen erfolgt durch die Hauptschleife. Der Aufruf erfolgt
// dadurch exakt 100 mal je Sekunde. Daher sollten die Unterroutinen keine Delays
// verwenden.


uint8_t RunAdjustArmRanges(uint8_t startParam) {
  static uint8_t RangeState=0;
  uint8_t doneStatus=0;
  static uint8_t ButtonHoldDelay=0;
  static uint8_t curRangeNo=1;
  uint8_t tmpCounterVal;

    if(startParam == STATE_INIT) {
    RangeState = STATE_INIT;
  }

  switch (RangeState)
  {
  case STATE_INIT:
    display.ScreenOut(SCR_ADJUST_RANGES);
    display.ShowRangeSelect(curRangeNo);
    RangeState = RST_ADJUST_RANGES;
    break;
  
  case RST_ADJUST_RANGES:     // Auswahl des Ranges
    if(backBUTTON_State == 1) { // Zurück, Abbruch
      doneStatus=1;
      
      RangeState = 0;
    }  

    if(upBUTTON_State == 1 && curRangeNo < 5) {
      curRangeNo++;
      display.ShowRangeSelect(curRangeNo);
    }

     if(downBUTTON_State == 1 && curRangeNo > 1) {
      curRangeNo--;
      display.ShowRangeSelect(curRangeNo);
    }

    if(startBUTTON_State == 1) {
      display.UpdateSetupValue(vacuArm.GetDelayRangeCounter(curRangeNo-1));
      RangeState = RST_EDIT_RANGE;
    }
    break;
  
  case RST_EDIT_RANGE:
    if(backBUTTON_State == 1) { // Zurück, Abbruch
      display.ScreenOut(SCR_ADJUST_RANGES);
      display.ShowRangeSelect(curRangeNo);  
      RangeState = RST_ADJUST_RANGES;
    }  

    if(upBUTTON_State == 1) {
      tmpCounterVal = vacuArm.GetDelayRangeCounter(curRangeNo-1);
      tmpCounterVal++;
      vacuArm.SetDelayRangeCounter(curRangeNo-1 ,tmpCounterVal);
      display.UpdateSetupValue(tmpCounterVal);
    }

    if(downBUTTON_State == 1) {
      tmpCounterVal = vacuArm.GetDelayRangeCounter(curRangeNo-1);
      tmpCounterVal--;
      vacuArm.SetDelayRangeCounter(curRangeNo-1 ,tmpCounterVal);
      display.UpdateSetupValue(tmpCounterVal);       
    }

    break;
  }


  return doneStatus;
}

/***
 * RunAdjustBorder - Einstellung Aussenrand der Platte
 * param startParam flag to init the state machine
 * return true (1), wenn der Vorgang abgeschlossen bzw. abgebrochen wurde.
 */
uint8_t RunAdjustBorder(uint8_t startParam) {
  static uint8_t BorderState=0;
  uint8_t doneStatus=0;
  static uint8_t ButtonHoldDelay=0;

  if(startParam == STATE_INIT) {
    BorderState = STATE_INIT;
  }

  switch (BorderState)
  {
  case STATE_INIT:
    display.UpdateSetupValue(vacuArm.GetBorderPos());
    vacuArm.liftUp();
    BorderState = BST_WAIT_LIFTUP;
    break;

  case BST_WAIT_LIFTUP:
    vacuArm.servoLiftStep();
    if(vacuArm.isLiftUp() == true) {   // Lift oben 
      vacuArm.servoTearDown();
      BorderState = BST_GOTO_BORDER;
    }
    break;

  case BST_GOTO_BORDER:
    if(vacuArm.StepToZeroPoint() == true) {  // Nullpunkt Stepper checken
      BorderState = BST_WAIT_BORDER;
      }
    break;

  case BST_WAIT_BORDER:
    if(vacuArm.StepZeroToBorder() == true) {
      BorderState = BST_ADJUST_BORDER;
    }
    break;
  
  case BST_ADJUST_BORDER:
    if(backBUTTON_State == 1) { // Zurück, Abbruch
      vacuArm.servoStartUp();
      vacuArm.liftUp();
      BorderState = BST_WAIT_RET_LIFTUP;
    }

    if(downBUTTON_State == 1) { // Border nach innen
      ButtonHoldDelay =0;
      vacuArm.IncBorderPos();
      display.UpdateSetupValue(vacuArm.GetBorderPos());
    }

    if(downBUTTON_State == 2) {
      if(ButtonHoldDelay == 50) {
        vacuArm.IncBorderPos();
        display.UpdateSetupValue(vacuArm.GetBorderPos());
      }
      else {
        ButtonHoldDelay++;
      }
    }

    if(upBUTTON_State == 1) { // Border nach aussen
      ButtonHoldDelay =0;
      vacuArm.DecBorderPos();
      display.UpdateSetupValue(vacuArm.GetBorderPos());
    }

    if(upBUTTON_State == 2) {
      if(ButtonHoldDelay == 50) {
        vacuArm.DecBorderPos();
        display.UpdateSetupValue(vacuArm.GetBorderPos());
      }
      else {
        ButtonHoldDelay++;
      }
    }
    break;

  case BST_WAIT_RET_LIFTUP:
    if(vacuArm.isLiftUp() == true) {
      BorderState = BST_WAIT_STEPPER_ZERO;
    }
    else {
      vacuArm.servoLiftStep();
    }
    
    break;

  case BST_WAIT_STEPPER_ZERO:
     if(vacuArm.StepToZeroPoint() == true) {
        vacuArm.liftDown();    
        BorderState = BST_WAIT_LIFTDOWN;
      }
  break;

  case BST_WAIT_LIFTDOWN:
    if(vacuArm.isLiftDown() == true) {
      doneStatus=1;
      BorderState=0;
    }
    else {
      vacuArm.servoLiftStep();
    }
    break;
  
  }
  
  return doneStatus;
}


uint8_t RunAdjustInner(uint8_t startParam) {
  static uint8_t BorderState=0;
  uint8_t doneStatus=0;
  static uint8_t ButtonHoldDelay=0;

  if(startParam == STATE_INIT) {
    BorderState = STATE_INIT;
  }

  switch (BorderState)
  {
  case STATE_INIT:
    display.UpdateSetupValue(vacuArm.GetInnerPos());
    vacuArm.liftUp();
    BorderState = BST_WAIT_LIFTUP;
    break;

  case BST_WAIT_LIFTUP:
    vacuArm.servoLiftStep();
    if(vacuArm.isLiftUp() == true) {   // Lift oben 
      vacuArm.servoTearDown();
      BorderState = BST_GOTO_BORDER;
    }
    break;

  case BST_GOTO_BORDER:
    if(vacuArm.StepToZeroPoint() == true) {  // Nullpunkt Stepper checken
      BorderState = BST_WAIT_BORDER;
      }
    break;

  case BST_WAIT_BORDER:
    if(vacuArm.StepZeroToInnerPos() == true) {
      BorderState = BST_ADJUST_BORDER;
    }
    break;
  
  case BST_ADJUST_BORDER:
    if(backBUTTON_State == 1) { // Zurück, Abbruch
      vacuArm.servoStartUp();
      vacuArm.liftUp();
      BorderState = BST_WAIT_RET_LIFTUP;
    }

    if(downBUTTON_State == 1) { // Border nach innen
      ButtonHoldDelay =0;
      vacuArm.IncInnerPos();
      display.UpdateSetupValue(vacuArm.GetInnerPos());
    }

    if(downBUTTON_State == 2) {
      if(ButtonHoldDelay == 50) {
        vacuArm.IncInnerPos();
        display.UpdateSetupValue(vacuArm.GetInnerPos());
      }
      else {
        ButtonHoldDelay++;
      }
    }

    if(upBUTTON_State == 1) { // Border nach aussen
      ButtonHoldDelay =0;
      vacuArm.DecInnerPos();
      display.UpdateSetupValue(vacuArm.GetInnerPos());
    }

    if(upBUTTON_State == 2) {
      if(ButtonHoldDelay == 50) {
        vacuArm.DecInnerPos();
        display.UpdateSetupValue(vacuArm.GetInnerPos());
      }
      else {
        ButtonHoldDelay++;
      }
    }
    break;

  case BST_WAIT_RET_LIFTUP:
    if(vacuArm.isLiftUp() == true) {
      BorderState = BST_WAIT_STEPPER_ZERO;
    }
    else {
      vacuArm.servoLiftStep();
    }
    
    break;

  case BST_WAIT_STEPPER_ZERO:
     if(vacuArm.StepToZeroPoint() == true) {
        vacuArm.liftDown();    
        BorderState = BST_WAIT_LIFTDOWN;
      }
  break;

  case BST_WAIT_LIFTDOWN:
    if(vacuArm.isLiftDown() == true) {
      doneStatus=1;
      BorderState=0;
    }
    else {
      vacuArm.servoLiftStep();
    }
    break;
  
  }
  
  return doneStatus;
}

/***
 * RunVacuClean - führt die Absaugung der Platte durch
 */
uint8_t RunVacuClean(uint8_t startParam) {
  static uint8_t VacuState=0;
  static uint8_t NextState=0;
  uint8_t doneStatus=0;
  static uint8_t ButtonHoldDelay=0;
  static uint16_t StateDelay=0;
  static uint8_t ui100HzSecCounter=0;
  static uint8_t progressBarCounter=0;

  if(startParam == STATE_INIT) {
    VacuState = STATE_INIT;
  }

  switch (VacuState)
  {
  case STATE_INIT:
    // Plattenteller Motor einschalten, vorwärts
    recMotor.SetDirection(STEPDIR_CLOCKWISE);
    recMotor.SetSpeed(1);
    recMotor.RunMotor();
    vacuArm.ResetSpeedRange();
    vacuArm.liftUp();
    VacuState = BST_WAIT_LIFTUP;
    break;
  
  case VST_WAIT_LIFTUP:
    vacuArm.servoLiftStep();
    if(vacuArm.isLiftUp() == true) {   // Lift oben 
      vacuArm.servoTearDown();
      VacuState = VST_GOTO_INNER;
    }
  break;

  case VST_GOTO_INNER:
    if(vacuArm.StepToZeroPoint() == true) {  // Nullpunkt Stepper checken
      VacuState = VST_WAIT_INNER;
      }
    break;

  case VST_WAIT_INNER:
    if(vacuArm.StepZeroToInnerPos() == true) {
      // Sauger einschalten, Lift absenken
      digitalWrite(RELAIS_VACTURBINE_PIN, HIGH);
      vacuArm.servoStartUp();
      vacuArm.liftDown();
      VacuState = VST_WAIT_LIFTDOWN;
    }
    break;

  case VST_WAIT_LIFTDOWN:
    if(vacuArm.isLiftDown() == true) {
      // Bewegung nach aussen sollte hier erst nach ca. 2,4s starten, damit die erste
      // Umdrehung komplett abgesaugt wird
      display.ShowVacuProgress(0);
      StateDelay = 240;
      NextState = VST_VAC_INNER_TO_BORDER;
      VacuState = VST_STEP_DELAY;
      //VacuState = VST_VAC_INNER_TO_BORDER;
    }
    else {
      vacuArm.servoLiftStep();
    }
    break;

  // --- Absaug-State -----------------------------
  case VST_VAC_INNER_TO_BORDER:
    if(backBUTTON_State == 1) { // Zurück, Abbruch
      vacuArm.liftUp();
      VacuState = VST_WAIT_RET_LIFTUP;
    }

    ui100HzSecCounter++;
    if(vacuArm.IsTimeForNextMove(ui100HzSecCounter) == true ) {
      ui100HzSecCounter=0;
      recMotor.SetSpeed(vacuArm.GetSpeedRange());
      display.ShowVacuProgress(progressBarCounter);
      progressBarCounter++;
      if(progressBarCounter > 4)
        progressBarCounter=0;
      if(vacuArm.StepInnerToBorderPos() == true) {
        vacuArm.liftUp();
        StateDelay=450; // am Ende den Rand noch eine mindestens eine Umdrehung absaugen
        NextState=VST_WAIT_RET_LIFTUP;
        VacuState = VST_STEP_DELAY;
      }
    }
  
    break;

  case VST_WAIT_RET_LIFTUP:
    if(vacuArm.isLiftUp() == true) {
      digitalWrite(RELAIS_VACTURBINE_PIN, LOW);
      VacuState = VST_WAIT_STEPPER_ZERO;
    }
    else {
      vacuArm.servoLiftStep();
    }
    break;

  case VST_WAIT_STEPPER_ZERO:
    if(vacuArm.StepToZeroPoint() == true) {
      recMotor.StopMotor();
      vacuArm.liftDown();    
      VacuState = VST_WAIT_RET_LIFTDOWN;
      }
    break;

  case VST_WAIT_RET_LIFTDOWN:
    if(vacuArm.isLiftDown() == true) {
      doneStatus = 1;
    }
    else {
      vacuArm.servoLiftStep();
    }
    break;
  
  // Verzögerungs-State zwischen einzelnen States. Es muss vorher Verwendung immer die Variable NextState
  // gesetzt werden
  case VST_STEP_DELAY: 
    if(StateDelay > 1) {
      StateDelay--;
    }
    else {
      VacuState = NextState;
    }
    break;
  }

  return doneStatus;
}

/***
 * RunWashing - Abarbeitung des Waschvorgangs. Dazu wird der Plattentellermotor
 * jeweils ein Drittel der gewählten Waschzeit vorwärts, rückwärts und wieder
 * vorwärts gedreht.
 * 
 * param startParam flag to init the state machine
 * return: true (1), wenn der Vorgang abgeschlossen bzw. abgebrochen wurde.
 */
uint8_t RunWashing(uint8_t startParam, bool breakSignal) {
  static uint8_t WashState=0;
  uint8_t doneStatus=0;
  uint8_t washminutes =0;
  static uint8_t recMotorTime1=0;       // Zeit1 vorwärts in Sekunden
  static uint8_t recMotorTime2=0;       // Zeit2 rückwärts in Sekunden
  static uint8_t recMotorTime3=0;       // Zeit3 vorwärts in Sekunden
  static uint8_t ui100HzSecCounter=0;   // Zähler für den Ablauf einer Sekunde

  if(startParam == STATE_INIT) {
    WashState = STATE_INIT;
  }

  if(breakSignal == true) {
    WashState = WST_BRUSHLIFTUP;
      brushArm.liftUp();
   }

  switch(WashState) {

  case STATE_INIT:
    washminutes = display.GetWashTime();
    recMotorTime1 = washminutes * 20;
    recMotorTime2 = washminutes * 20;
    recMotorTime3 = washminutes * 20;
    ui100HzSecCounter=0;
    WashState=WST_RUN;
    display.ScreenOut(SCR_WASHPROCESS);
    display.UpdateWashSecondsLeft((int) recMotorTime1 + (int) recMotorTime2 + (int) recMotorTime3);
    // Motor turn clockwise
    recMotor.SetDirection(STEPDIR_CLOCKWISE);
    recMotor.SetSpeed(1);
    recMotor.RunMotor();
    brushArm.servoStartUp();
    brushArm.liftDown();
    break;
  
  case WST_RUN:
        ui100HzSecCounter++;
        if(ui100HzSecCounter >=100) {
          display.UpdateWashSecondsLeft((int) recMotorTime1 + (int) recMotorTime2 + (int) recMotorTime3);
          ui100HzSecCounter = 0;
          
          if(recMotorTime1 > 0) {
            recMotorTime1--;
          } 
          else {
          // nächste Richtung setzen
          WashState=WST_WAITREVERS;
          recMotor.StopMotor();
          }
        }
    break;

  case WST_WAITREVERS:
    ui100HzSecCounter++;
    if(ui100HzSecCounter >=80) {
      ui100HzSecCounter=0;
      // turn motor counterclockwise
      recMotor.SetDirection(STEPDIR_COUNTERCLOCKWISE);
      recMotor.RunMotor();
      WashState=WST_RUNREVERS;
    }
    break;

  case WST_RUNREVERS:
    ui100HzSecCounter++;
    if(ui100HzSecCounter >=100) {
      display.UpdateWashSecondsLeft((int) recMotorTime1 + (int) recMotorTime2 + (int) recMotorTime3);
      ui100HzSecCounter = 0;
      if(recMotorTime2 > 0) {
        recMotorTime2--;
      } 
      else {
      // nächste Richtung setzen
      recMotor.StopMotor();
      WashState=WST_WAITFORWARD;
      }
    }      
    break;

  case WST_WAITFORWARD:
    ui100HzSecCounter++;
    if(ui100HzSecCounter >=80) {
      WashState=WST_FORWARD;
      recMotor.SetDirection(STEPDIR_CLOCKWISE);
      recMotor.RunMotor();      
    }
    break;

  case WST_FORWARD:
    ui100HzSecCounter++;
    if(ui100HzSecCounter >=100) {
      display.UpdateWashSecondsLeft((int) recMotorTime1 + (int) recMotorTime2 + (int) recMotorTime3);
      ui100HzSecCounter = 0;
      if(recMotorTime3 > 0) {
        recMotorTime3--;
      } 
      else {
      // fertig
      WashState = WST_BRUSHLIFTUP;
      brushArm.liftUp();
      
      }
    }
    break;

  case WST_BRUSHLIFTUP:
    if(brushArm.isLiftUp() == true) {
      doneStatus=1;
      recMotor.StopMotor();
      brushArm.servoTearDown();
    }
    break;
  
  }

  brushArm.servoLiftStep();

  return doneStatus;
}
