/***
 * Project: Vinyl Record Cleaner (Plattenwaschmaschine)
 * File   : VRCDisplay.h
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

#include <Arduino.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>


// Display Konstanten für ScreenOut
#define SCR_CLEAR       0       // Display löschen
#define SCR_WELCOME     1       // Welcome Message
#define SCR_MAINAUTOCL  2       // Auto Clean - Waschen und Absaugen automatisch
//TODO: defines um 1 erhöhen - erledigt
#define SCR_MAINWASH    3       // Main Menu screen Waschen
#define SCR_MAINVACUUM  4       // Main Menu Absaugen
#define SCR_MAINSETUP   5       // Main Menu Einstellungen
#define SCR_WASHMENU    6       // Menü Waschen Start und Zeiteinstellungen
#define SCR_WASHPROCESS 10      // Waschprozess läuft
#define SCR_VACUPROCESS 20      // Absaugung läuft

#define SCR_SETUPSELECT 30      // Untermenü Setup Auswahl
#define SCR_SET_LIFTHI  31      // Einstellung Lifthöhe
#define SCR_SET_BORDER  32      // Einstellung Position Aussenrand, Positionen werden in Steps vom Nullpunkt angegeben
#define SCR_SET_INNERP  33      // Einstellung innere Position
#define SCR_SET_ARMDELAY 34     // Einstellungen Verzögerungsbereiche für Armbewegung nach innen
#define SCR_SAVESETUP   35      // Einstellungen speichern

// Ranges einstellen -------------
#define SCR_ADJUST_RANGES   50  // Screen für Ranges einstellen

#define SCR_ADJUSTPART      40
#define SCR_ADJUSTLIFT      41      // Einstellung Lifthöhe
#define SCR_ADJUSTBORDER    42      // Plattenrand aussen
#define SCR_ADJUSTINNER     43      // Pos Platte innen
#define SCR_ADJUSTSAVE      44      // Einstellung speichern


class VRCDisplay
{
private:
    /* data */
    uint8_t washTime = 2;       // Waschzeit in Minuten, default 2
    uint8_t washCounter = 0;
    /* private methods */
    void PrintHelpLine(uint8_t lineType);

public:
    VRCDisplay(/* args */);
    void Setup();
    void SetBacklight(uint8_t mode);
    void ScreenOut(uint8_t uiScreenID);
    uint8_t GetWashTime();
    void SetWashTime(uint8_t wTime);
    void IncWashTime();
    void DecWashTime();
    void IncWashCounter();
    void UpdateWashTime();
    void UpdateWashSecondsLeft(int washSecondsLeft);
    void ShowVacuProgress(uint8_t barVal);
    void UpdateSetupValue(int setValue);
    void ShowWashCounter();
    void ShowShortMsg(char* msg);
    void ShowRangeSelect(uint8_t rangeNo);
};



