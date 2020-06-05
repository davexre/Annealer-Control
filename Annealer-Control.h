/*
 * Annealer-Control.h
 * 
 * Header file for annealer project. Breaks out defines, etc.
 * 
 */

#ifndef _ANNEALER_CONTROL_H
#define _ANNEALER_CONTROL_H

#include <Chrono.h>
#include <EEPROM.h>
#include <menu.h>
#include <Rencoder.h>
#include <SerLCD.h> // SerLCD from SparkFun - http://librarymanager/All#SparkFun_SerLCD
#include <Wire.h>

#include <ctype.h> 

/*
 * DEBUG - uncomment the #define to set us to DEBUG mode! Make sure you open a serial 
 * terminal at 115200 baud. Note that we don't expect to run with a Serial port regularly, 
 * so printing anything to Serial normally isn't going to be super useful.
 * 
 * If this isn't obvious, you need to recompile after commenting/uncommenting this statement!
 * 
 * You really should uncomment DEBUG when you do any of the others - otherwise, you may see
 * unexpected behavior.
 */

// #define DEBUG
// #define DEBUG_LOOPTIMING
// #define DEBUG_STATE
// #define DEBUG_LCD


// Select the pin layout needed based on which annealer shield is in play. If none, 
// set up the right pin layout for your installation.

#define _PROTO_BOARD
// #define _V3_BOARD

/*
 *  PIN CONFIGURATION
 */
#ifdef _PROTO_BOARD
  #define  THERM1_PIN      A0
  #define  CURRENT_PIN     A1
  #define  VOLTAGE_PIN     A2
  #define  OPTO_PIN        A5
  #define  INDUCTOR_PIN    4
  #define  SOLENOID_PIN    5
  #define  START_PIN       6   
  #define  STOP_PIN        7   
  #define  INDUCTOR_LED    8
  #define  SOLENOID_LED    9
  #define  ENCODER_A_PIN   10
  #define  ENCODER_B_PIN   11
  #define  ENCODER_BUTTON  12
#endif

#ifdef _V3_BOARD
  #define  VOLTAGE_PIN     A0
  #define  CURRENT_PIN     A1
  #define  THERM1_PIN      A2
  #define  OPTO1_PIN       A3
  #define  THERM2_PIN      A4
  #define  OPTO2_PIN       A5
  #define  AUX1_PIN        2
  #define  AUX2_PIN        3
  #define  INDUCTOR_PIN    4
  #define  SOLENOID_PIN    5
  #define  START_PIN       6   
  #define  STOP_PIN        7   
  #define  INDUCTOR_LED    8
  #define  SOLENOID_LED    9
  #define  ENCODER_A_PIN   10
  #define  ENCODER_B_PIN   11
  #define  ENCODER_BUTTON  12
#endif



/*
 * CONSTANTS
 */
// System Constants
#ifdef _AP3_VARIANT_H_
#define RESOLUTION_MAX  16384 // Artemis boards have a 14-bit ADC resolution (make sure to turn it on!!!)
#define VREF            2.0
#else
#define RESOLUTION_MAX  1024 // 10-bit ADC seems to be the lowest common denominator
#define VREF            5.0  // YMMV - basic Arduino board is 5v, but some are 3.3V
#endif

// Thermistor Values
#define THERM_NOMINAL       10000   //Resistance at nominal temperature
#define THERM_NOM_TEMP      25      //Nominal temperature in DegC
#define THERM_BETA          3950    //Beta coefficient for thermistor
#define THERM_RESISTOR      10000   //Value of resistor in series with thermistor
#define THERM_SMOOTH_RATIO  0.35    // What percentage of the running average is the latest reading - used to smooth analog input

#ifdef _AP3_VARIANT_H_
#define INT_TEMP_SMOOTH_RATIO 0.35
#endif

// Power sensor values
#define AMPS_SMOOTH_RATIO   0.50
#define VOLTS_SMOOTH_RATIO  0.50

#ifdef _AP3_VARIANT_H_
#define VOLTS_PER_RESOLUTION  0.0029296875 // 48v over 14-bit resolution - 48 divided by 16384
#else
#define VOLTS_PER_RESOLUTION  0.046875  // 48v over 10-bit resolution - 48 divided by 1024
#endif

// EEPROM addresses - int is 2 bytes, so make sure these are even numbers!
#define ANNEAL_ADDR   0
#define DELAY_ADDR    4
#define CASEDROP_ADDR 8
#define EE_FAILSAFE_ADDR  16
#define EE_FAILSAFE_VALUE 45  // bump in v0.5

// Control constants
#define CASE_DROP_DELAY_DEFAULT   50      // hundredths of seconds
#define ANNEAL_TIME_DEFAULT       10      // hundredths of seconds - for the timer formats
#define DELAY_DEFAULT             50      // hundredths of seconds - for the timer formats
#define LCD_STARTUP_INTERVAL      1000    // milliseconds - let the screen fire up and come online before we hit it
#define LCD_UPDATE_INTERVAL       500     // milliseconds
#define ANNEAL_LCD_TIMER_INTERVAL 100     // milliseconds - interval to update LCD timer during active anneal
#define ANNEAL_POWER_INTERVAL     250     // millseconds  - interval to check and update power sensors during active anneal
#define DEBOUNCE_MICROS           100000  // MICROseconds

// LCD contstants
#define LCD_SETPOINT_LABEL  0,0
#define LCD_SETPOINT        4,0
#define LCD_TIMER_LABEL     9,0
#define LCD_TIMER           15,0
#define LCD_CURRENT_LABEL   0,1
#define LCD_CURRENT         4,1
#define LCD_VOLTAGE_LABEL   9,1
#define LCD_VOLTAGE         15,1
#define LCD_THERM1_LABEL    0,2
#define LCD_THERM1          5,2
#define LCD_2NDTEMP_LABEL   9,2
#define LCD_2NDTEMP         16,2
#define LCD_STATE_LABEL     0,3
#define LCD_STATE           7,3


#define ANALOG_INTERVAL       1000
#define LCDSTARTUP_INTERVAL   1000


enum AnnealState
{
  WAIT_BUTTON,
  WAIT_CASE,
  START_ANNEAL,
  ANNEAL_TIMER,
  DROP_CASE,
  DROP_CASE_TIMER,
  DELAY
};

enum MenuState
{
  MAIN_MENU,
  ANNEALING,
  MAYAN
};

extern Encoder encoder;

extern AnnealState annealState;
extern MenuState menuState;

extern const char *annealStateDesc[];

extern float amps;
extern float volts;
extern float Therm1Avg;
extern float Therm1Temp;
extern float Therm1TempHigh;  // track highest temp we saw

#ifdef _AP3_VARIANT_H_
extern float internalTemp;
extern float internalTempHigh;  // track highest temp we saw
#endif

extern SerLCD lcd;
extern Chrono Timer;
extern Chrono AnalogSensors; 
extern Chrono AnnealPowerSensors;
extern Chrono AnnealLCDTimer;
extern Chrono LCDTimer;

extern float annealSetPoint;
extern float delaySetPoint;
extern float caseDropSetPoint;

extern boolean showedAnnealingScreen;

extern boolean startOnOpto; // not currently used

extern int encoderDiff;
extern int storedSetPoint; 
extern int storedDelaySetPoint;
extern int storedCaseDropSetPoint;

extern boolean encoderPressed;
extern boolean encoderMoved;
extern volatile boolean startPressed;
extern volatile boolean stopPressed;

extern Menu::navRoot nav;

// function protos

void annealStateMachine(void);
float calcSteinhart(float);
void checkPowerSensors(boolean);
void checkThermistors(boolean);
void updateLCD(boolean full);
void updateLCDState(void);
void updateLCDSetPoint(boolean sendIt);
void updateLCDPowerDisplay(boolean sendIt);
void updateLCDTemps(boolean sendIt);
void updateLCDTimer(boolean sendIt);
void eepromStartup(void); 
void eepromCheckAnnealSetPoint(void);
void eepromCheckDelaySetPoint(void);
void eepromCheckCaseDropSetPoint (void);

#endif // _ANNEALER_CONTROL_H
