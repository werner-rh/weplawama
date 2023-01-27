/***
 * Project: Vinyl Record Cleaner (Plattenwaschmaschine)
 * File   : VRCDisplay.cpp
 * Author : Werner Riemann 
 * Created: 23.07.2022
 * Board: Arduino Nano
 * 
 * Description: Modul for Display outputs
 * 
 * Pins:
 * A4,A5  - I2C Display control
 * 
 */

#include "VRCDisplay.h"
#include "RecordCleanerDef.h"


LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7,3,POSITIVE);

VRCDisplay::VRCDisplay(/* args */)
{
}

void VRCDisplay::Setup() {
  lcd.begin(16, 2);
  lcd.clear();  
}


void VRCDisplay::SetBacklight(uint8_t mode) 
{
  lcd.setBacklight(mode);
}


uint8_t VRCDisplay::GetWashTime() {
  return washTime;
}

void VRCDisplay::SetWashTime(uint8_t wTime) {
  washTime = wTime;
}

void VRCDisplay::IncWashTime() {
  if(washTime < 5) { 
    washTime ++;
    UpdateWashTime();
  }
}

void VRCDisplay::DecWashTime() {
  if(washTime > 1) { 
    washTime --;
    UpdateWashTime();
  }
}

void VRCDisplay::UpdateWashTime() {
  char szBuf[3] = "";
  itoa(washTime, szBuf, 10);
  lcd.setCursor(11,0);
  lcd.print(szBuf);
}

void VRCDisplay::UpdateWashSecondsLeft(int washSecondsLeft) {
  char buf[6]="";
  char szTimeDisp[6]= "000";
  int l=0;
  itoa(washSecondsLeft, buf, 10);
  l=strlen(buf);
  strcpy(&szTimeDisp[3-l],buf);
  lcd.setCursor(13,0);
  lcd.print(szTimeDisp);
}

/***
 * ShowVacuProgress - Zeigt einen Fortschrisbalken an, der von links nach rechts
 *                    schrittweise die Zeichen mit Blanks ersetzt.
 * 
 * param barVal gibt an, wieviele Zeichen von links nach rechts mit Blanks ersetzt werden.
 */
void VRCDisplay::ShowVacuProgress(uint8_t barVal) {
  char szBar[6]= ">>>>";
  int l; 

  for(l=0; l < barVal; l++) {
    szBar[l]=' ';
  }
  lcd.setCursor(12,0);
  lcd.print(szBar);
}

void VRCDisplay::UpdateSetupValue(int setValue) {
  char buf[6]="";
  char szValDisp[7]= "[    ]";
  int l=0;
  int i;
  itoa(setValue, buf, 10);
  l=strlen(buf);
  for(i=0; i < l; i++)
    szValDisp[i+1] = buf[i];
  lcd.setCursor(0,1);
  lcd.print(szValDisp);
}

void VRCDisplay::ShowWashCounter() {
  char buf[6]="";
  int l=0;
  
  itoa(washCounter, buf, 10);
  l=strlen(buf);
  lcd.setCursor(15-l,0);
  lcd.print(buf);
}

void VRCDisplay::IncWashCounter() {
  washCounter++;
}

void VRCDisplay::ShowShortMsg(char* msg) {
  int l=strlen(msg);
  lcd.setCursor(0,1);
  lcd.print("      ");
  if(l <= 6) {
    lcd.setCursor(0,1);
    lcd.print(msg);
  }
}

void VRCDisplay::PrintHelpLine(uint8_t lineType) {
  char szHelpline1[17] = "Start        < >";
  char szHelpline2[17] = "Start  Back  < >";
  char szHelpline3[17] = "    Stop/Back   ";

  lcd.setCursor(0,1);
  switch (lineType)
  {
  case 0:
    lcd.print(szHelpline1);
    break;
  
  case 1:
    lcd.print(szHelpline2);
    break;

  case 2:
    lcd.print(szHelpline3);
    break;
  }
}

void VRCDisplay::ShowRangeSelect(uint8_t rangeNo) {
  char buf[6]="";
  
  itoa(rangeNo, buf, 10);
  lcd.setCursor(15,0);
  lcd.print(buf);

}

void VRCDisplay::ScreenOut(uint8_t uiScreenID) {

  lcd.setCursor(0,0); // start ist immer oben links

  switch(uiScreenID) 
  {
  case SCR_CLEAR:
    lcd.clear();
    break;

  case SCR_WELCOME:
    lcd.print("WePlaWama22 MKII");
    lcd.setCursor(0,1);
    lcd.print("FW:      READY >");
    lcd.setCursor(4,1);
    lcd.print(FWVERSION);
    break;

  case SCR_MAINAUTOCL:
    lcd.print("[  AutoClean   ]");
    PrintHelpLine(0);  
    break;

  case SCR_MAINWASH:
    lcd.print("[   Waschen    ]");
    ShowWashCounter();
    PrintHelpLine(0);
    break;

  case SCR_MAINVACUUM:
    lcd.print("[   Absaugen   ]");
    PrintHelpLine(0);
    break;

  case SCR_MAINSETUP:
    lcd.print("[ Einstellungen]");
    PrintHelpLine(0);
    break;

  case SCR_WASHMENU:
    lcd.print("Waschen:     Min");
    UpdateWashTime();
    PrintHelpLine(1);
    break;

  case SCR_WASHPROCESS:
    lcd.print("Waschvorgang    ");
    PrintHelpLine(2);
    break;

  case SCR_VACUPROCESS:
    lcd.print("Absaugung       ");
    PrintHelpLine(2);
    break;

  case SCR_SET_LIFTHI:
    lcd.print("Set:Armlifthoehe");
    PrintHelpLine(1);
    break;

  case SCR_SET_BORDER:
    lcd.print("Set:ArmPos  Rand");
    PrintHelpLine(1);
    break;

  case SCR_SET_INNERP:
    lcd.print("Set:ArmPos innen");
    PrintHelpLine(1);
    break;

  case SCR_SET_ARMDELAY:
    lcd.print("Set:Arm VacDelay");
    PrintHelpLine(1);
    break;

  case SCR_SAVESETUP:
    lcd.print("Setups speichern");
    PrintHelpLine(1);
    break;

  case SCR_ADJUST_RANGES:
    lcd.print("Set VacDelay: R ");
    PrintHelpLine(1);
    break;
  }

}