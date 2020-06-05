/**************************************************************************************************
 * 
 * Annealer Control Program
 * Author: Dave Re
 * Inception: 04/23/2020
 * 
 * The basic annealer design is a GinaErick model, described in this thread on accurateshooter.com: 
 * http://forum.accurateshooter.com/threads/induction-brass-annealer-redux.3908353/
 * 
 * This code is heavily informed by posts from ottsm there - specifically post 1050
 * 
 * Requires:
 *  - SparkFun SerLCD library
 * 
 * 
 * Options:
 *  - SparkFun Apollo3 core 1.1.1 or above - includes an internal thermistor on the CPU, as well
 *    as several other features that seem inviting.     
 *    
 *    
 * So, why the whole "#ifdef DEBUG" thing, rather than a subroutine to print out debug messages?
 * I just don't want the DEBUG code to compile in the final code, if we don't need to debug things.
 * Yeah, this way is messy to read - I'm trading that for a smaller executable, and a tighter
 * execution path for the final code.
 * 
 * 
 * This code assumes a couple pieces of hardware:
 * - SparkFun AVR 20x4 LCD, connected through I2C using SerLCD (though it would be easily
 *   converted to using the LiquidCrystal library with any I2C based display)
 * - An encoder with a click button
 * 
 **************************************************************************************************/
 
#include <Chrono.h>
#include <Rencoder.h>
#include <SerLCD.h> // SerLCD from SparkFun - http://librarymanager/All#SparkFun_SerLCD
#include <Wire.h>

#include <ctype.h>  // presumably to get enumerations

#include "Annealer-Control.h" // globals and constants live here!
#include "AnnealMenu.h"


#define VERSION   0.6


/******************************************************
 * GLOBALS
 ******************************************************/

/*
 * STATE MACHINES - enum detailing the possible states for the annealing state machine. We may 
 * set one up for our future "Mayan" mode, if we get there, too.
 */

enum AnnealState annealState;

enum MenuState menuState;

const char *annealStateDesc[] = {
  "  Press Start",
  "Wait for Case",
  "    Annealing",
  "    Annealing",
  "    Drop Case",
  "    Drop Case",
  "        Pause",
  "        ERROR"
};


/*
 * DISPLAYS - initialize using SparkFun's SerLCD library
 */

SerLCD lcd; // Initialize the LCD with default I2C address 0x72

/*
 * ENCODER
 */


Encoder encoder(ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_BUTTON);


 /*
  * TIMERS - Chrono can set up a metronome (replaces old Metro library) to establish
  * a periodic time for accomplishing a task, based on milliseconds
  */
Chrono AnalogSensors; 
Chrono AnnealPowerSensors;
Chrono AnnealLCDTimer;
Chrono LCDTimer;
Chrono Timer; 

/*
 * ANALOG SENSOR VARIABLES
 */
float Therm1Avg = 0; 
float Therm1Temp = 0;
float Therm1TempHigh = 0;  // track highest temp we saw

#ifdef _AP3_VARIANT_H_
float internalTemp = 0;
float internalTempHigh = 0;  // track highest temp we saw
#endif

float amps = 0;
float volts = 0;

/*
 * Control variables
 */
boolean encoderPressed = false;
boolean encoderMoved = false;
volatile boolean startPressed = false;
volatile boolean stopPressed = false;

boolean startOnOpto = false; // we'll use this once we have an optical case sensor

float annealSetPoint = (float) ANNEAL_TIME_DEFAULT / 100;  // plan to store this value as hundredths of seconds, multiplied by 100
float delaySetPoint = (float) DELAY_DEFAULT / 100;         // same format - in this case, we start with a half second pause, just in case
float caseDropSetPoint = (float) CASE_DROP_DELAY_DEFAULT / 100;
int encoderDiff = 0;

volatile unsigned long startdebounceMicros = 0;
volatile unsigned long stopdebounceMicros = 0;

boolean showedAnnealingScreen = false;

#ifdef DEBUG_LOOPTIMING
unsigned long loopMillis = 0;
#endif

#ifdef DEBUG
int temp = 0;
#endif



/******************************************************
 * FUNCTIONS 
 ******************************************************/

/* 
*  startPressedHandler
*  
*  Interrupt handler for the start button being pressed.
*/

void startPressedHandler(void) {
  if ((long) (micros() - startdebounceMicros) >= DEBOUNCE_MICROS) {
    startdebounceMicros = micros();
    startPressed = true;
  }
}

 /* 
  *  stopPressedHandler
  *  
  *  Interrupt handler for the stop button being pressed.
  */

void stopPressedHandler(void) {
  if ((long) (micros() - stopdebounceMicros) >= DEBOUNCE_MICROS) {
    stopdebounceMicros = micros();
    stopPressed = true;
  }
}



/**************************************************************************************************
 * setup
 **************************************************************************************************/
void setup() {

  // Assign pin modes
  pinMode(INDUCTOR_PIN, OUTPUT);
  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(START_PIN, INPUT_PULLUP);
  pinMode(STOP_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);


  // make sure inductor board power is off, and the trap door is closed
  digitalWrite(INDUCTOR_PIN, LOW);
  digitalWrite(SOLENOID_PIN, LOW);

  attachInterrupt(digitalPinToInterrupt(START_PIN), startPressedHandler, FALLING);
  attachInterrupt(digitalPinToInterrupt(STOP_PIN), stopPressedHandler, FALLING);

  Serial.begin(115200); // something gets unhappy if we don't spin up Serial
  #ifdef DEBUG
    while (!Serial) ;
  #endif

  #ifdef _AP3_VARIANT_H_
    analogReadResolution(14); //Set ADC resolution to the highest value possible 
  #else
    analogReadResolution(10);
  #endif

  #ifdef DEBUG
    #ifdef _AP3_VARIANT_H_
      Serial.println("DEBUG: ADC read resolution set to 14 bits");
    #else
      Serial.println("DEBUG: ADC read resolution set to 10 bits");
    #endif
  #endif


  /*
   * XXXXXXX
   * 
   * Look into error handling for Wire, encoder, and lcd initialization
   * we need to error this out somehow, and make sure that we don't  
   * proceed if control or display is toast - and also make sure
   * that we can signal the user about it (LED flashes, if the screen
   * isn't there, etc.
   * 
   */

  #ifdef DEBUG
    Serial.println("DEBUG: starting I2C and LCD");
  #endif
  
  Wire.begin();
  lcd.begin(Wire);
  Wire.setClock(400000);

  // set up the menu system a bit ahead of initial call to nav.poll() below
  nav.idleTask=idle;
  nav.showTitle=false;
  nav.inputBurst=10; // helps responsiveness to the encoder knob

  // set the display for high temps to be read-only
  dataDisplayMenu[0].disable();
  #ifdef _AP3_VARIANT_H_
  dataDisplayMenu[1].disable();
  #endif
    
  // Initial analog sensor baselines
  checkThermistors(true);

  // initialize amps and volts for the display
  checkPowerSensors(true);

  // pull the intial settings from the EEPROM
  eepromStartup();
  

  // double check that we've been here for a second before we talk to the LCD
  // This is to work around what seems to be a startup timing issue, where 
  // the Apollo3 CPU gets through some of the init code faster than the 
  // LCD controller is actually ready to receive it. 
  
  if (! LCDTimer.hasPassed(LCD_STARTUP_INTERVAL) ) {
    delay(LCD_STARTUP_INTERVAL - LCDTimer.elapsed());
  } // clear to make first output to the LCD, now
 
  lcd.clear();

  // Show a banner, for now - might program this as a startup screen on the LCD later
  //
  // 01234567890123456789
  //   CASE BURNER 5000
  // PREPARE FOR GLORY!!!

  lcd.setCursor(2,1);
  lcd.print("CASE BURNER 5000");
  lcd.setCursor(0,2);
  lcd.print("PREPARE FOR GLORY!!!");
  delay(2000);
  
  lcd.clear();
  LCDTimer.restart();


  #ifdef DEBUG
  Serial.println("DEBUG: END OF SETUP!");
  #endif

}


/**************************************************************************************************
 * the loop
 **************************************************************************************************/
void loop() {

  #ifdef DEBUG_LOOPTIMING
  loopMillis = millis();
  #endif

  if (nav.sleepTask) {  // if we're not in the ArduinoMenu system

    // if this is our first cycle outside the menu, draw the whole screen and save any settings
    if (!showedAnnealingScreen) {
      showedAnnealingScreen = true;
      updateLCD(true);

      eepromCheckAnnealSetPoint();
      eepromCheckDelaySetPoint();
      eepromCheckCaseDropSetPoint();
      
    }

    // buzz through the state machine
    annealStateMachine();
    
  } // if (nav.sleepTask())
  else {
    
    // we're in the main menu!
    nav.poll();
    
  }

  #ifdef DEBUG_LOOPTIMING
  if ((millis() - loopMillis) > 50) {
    Serial.print("DEBUG: loop took ");
    Serial.println(millis() - loopMillis);
  }
  #endif
  
} // loop()
