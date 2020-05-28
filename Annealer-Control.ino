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
 * 
 * TO DO list:
 * - handle anything commented with "XXXXXX"
 * - set up IR optical detection on the case
 * - saved settings for different brass (possibly allowing name edits) and ability to choose
 * - support for a casefeeder (second opto, and logic)
 * - consider moving annealer "power on" LED to control here, or add a "status" LED. This can
 *   be used for communication with the user when the annealer has a problem, too.
 * - look at supporting both SerLCD amd LiquidCrystal
 * 
 * 
 * Version History:
 * v 0.5 - Implement ArduinoMenu, move environmental functions to separate file, convert 
 *         timer ints to floats
 * v 0.4 - Apollo3 1.1.1 core udpate, comment and DEBUG statement cleanup, moved LCD functions
 *         to separate files (AnnealLCD.cpp and .h), convert code to use a normal encoder
 *         library, cleanup architecture defines
 * v 0.3 - fix for internal thermistor
 * v 0.2 - refactor handling of LCD, ditched Bounce2, moved defines to header file
 * v 0.1 - initial stab at the code, just replicating the Sestos timer functionality, mostly
 * 
 **************************************************************************************************/
 
#include <Chrono.h>
#include <Rencoder.h>
#include <SerLCD.h> // SerLCD from SparkFun - http://librarymanager/All#SparkFun_SerLCD
#include <Wire.h>

#include <ctype.h>  // presumably to get enumerations

#include "Annealer-Control.h" // globals and constants live here!
#include "AnnealLCD.h" // LCD functions for when we're not in the menu system are here
#include "AnnealEEPROM.h"
#include "AnnealMenu.h"
#include "Environmentals.h"


#define VERSION   0.5


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

#ifdef DEBUG_STATE
  boolean stateChange = true;
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

/*
 * signalOurDeath()
 * 
 * If we hit a fatal error, call this routine to die usefully. First, just make sure that the SSRs
 * are definitely turned off. Then, fast blink the built in LED, and sit and spin.
 * 
 * If we're in DEBUG, also send something out to the console 
 * 
 */
void signalOurDeath() {

  digitalWrite(INDUCTOR_PIN, LOW);
  digitalWrite(SOLENOID_PIN, LOW);

  #ifdef DEBUG
  Serial.println("DEBUG: signalOurDeath called.");
  Serial.println("DEBUG: It's dead, Jim.");
  Serial.println("DEBUG: I'm sorry, Dave. I'm afraid I couldn't do that.");
  #endif

  while(1) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
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

    
    ////////////////////////////////////////////////////////////////
    // Housekeeping - https://www.youtube.com/watch?v=ERd5Faz8_-E  /
    ////////////////////////////////////////////////////////////////
    
    // gather button statuses
    
    if ( encoder.isClicked() ) {
      if (annealState == WAIT_BUTTON) { // exit annealing mode
        nav.idleOff();
        menuState = MAIN_MENU;
        showedAnnealingScreen = false;
        (void) encoder.clear(); // clear our flags
      }
      else { // if we're in a cycle, we'll use this click to stop the cycle safely
        encoderPressed = true;
        (void) encoder.isDoubleClicked(); // clear this flag in this context
        
        #ifdef DEBUG
        Serial.println("DEBUG: Encoder clicked");
        #endif
      }
    }
  
    if (startPressed && (annealState == WAIT_BUTTON)) {
      startPressed = true; // in case we didn't get here by interrupt
      
     #ifdef DEBUG
      Serial.println("DEBUG: start button pressed");
     #endif
     #ifdef DEBUG_STATE
      stateChange = true;
    #endif
  
    } 
    else if (startPressed) startPressed = false;
  

    // only take action on the Stop Button if we're actively in the anneal 
    // cycle. Treat the encoder button as a Stop Button if we're annealing, too
    if ((stopPressed || encoderPressed) && (annealState != WAIT_BUTTON)) {
      digitalWrite(INDUCTOR_PIN, LOW);
      digitalWrite(LED_BUILTIN, LOW);
      digitalWrite(SOLENOID_PIN, LOW);
      annealState = WAIT_BUTTON;
      encoderPressed = false;
      
      #ifdef DEBUG
      Serial.println("DEBUG: stop button pressed - anneal cycle aborted");
      #endif
      #ifdef DEBUG_STATE
      stateChange = true;
      #endif
  
    }
    else if (stopPressed) stopPressed = false;
  
  
    // check the encoder - note, only update this if we're not actively
    // annealing cases!
    encoderMoved = encoder.isMoved();
    if (encoderMoved && annealState == WAIT_BUTTON) {
      
      #ifdef DEBUG
        // heuristics to actually output the difference... sigh
        encoderDiff = encoder.getDiff(true);
        Serial.print("DEBUG: encoder moved - diff is ");
        Serial.println(encoderDiff);
        annealSetPoint += encoderDiff / 100.0;
        Serial.print("DEBUG: new annealSetPoint = ");
        Serial.println(annealSetPoint);
      #else
        annealSetPoint += encoder.getDiff(true) / 100.0;
      #endif
  
      /*
       * XXXXXX - Leave this section out, for now - not sure if it would interfere with
       * ArduinoMenu implementation to come if we zero'ed the count or not!
       
      int encoderCount = encoder.getCount();
      if ((encoderCount > 32000) || (encoderCount < -32000)) {
        
        #ifdef DEBUG
        Serial.print("DEBUG: Encoder count resetting - current count ");
        Serial.println(encoderCount);
        #endif
        
        encoder.setCount(0); // prevent over/underflow
      }
      */
      
      encoderMoved = false;
      
    }
    else if (encoderMoved) {  // and we're somewhere else in the state machine
      encoderDiff = encoder.getDiff(); // clear the difference
      encoderMoved = false;
    }
  
    
    // Check our normal analog sensors
    
    if (AnalogSensors.hasPassed(ANALOG_INTERVAL, true)) {   // Note - the boolean restarts the timer for us
  
      // we us this section to update current and voltage while we're not actively annealing
      if ( (annealState != START_ANNEAL) && (annealState != ANNEAL_TIMER ) ) checkPowerSensors(false);
      
      checkThermistors(false);
  
      #ifdef DEBUG_VERBOSE
      Serial.print("DEBUG: Inductor Board Thermistor =");
      Serial.println(Therm1Temp);
      Serial.print("DEBUG: Internal thermistor = ");
      Serial.println(internalTemp);
      #endif
      
    } // if (AnalogSensors...
  
  
  
  
    ////////////////////////////////////////////////////////
    // Basic state machine for the annealing cycle
    ////////////////////////////////////////////////////////
  
  
  
    switch(annealState) {
  
      ////////////////////////////////
      // WAIT_BUTTON
      //
      // Wait for the start button
      // Normal sensor handling, etc
      ////////////////////////////////
      case WAIT_BUTTON:
        #ifdef DEBUG_STATE
          if (stateChange) { Serial.println("DEBUG: STATE MACHINE: enter WAIT_BUTTON"); stateChange = false; }
        #endif
        
        if ( LCDTimer.hasPassed(LCD_UPDATE_INTERVAL) ) {
          updateLCD(false);
          LCDTimer.restart();
        }
        
        if (startPressed) {
          annealState = WAIT_CASE;
          startPressed = false;
          updateLCDState();
          
          #ifdef DEBUG_STATE
          stateChange = true;
          #endif
        }
        break;
  
  
      ////////////////////////////////
      // WAIT_CASE
      //
      // If we have an optical sensor
      // for cases, we'll wait here for
      // the sensor to detect a case.
      // Normal sensor handling while
      // we wait.
      ////////////////////////////////
      case WAIT_CASE:
        #ifdef DEBUG_STATE
        if (stateChange) { Serial.println("DEBUG: STATE MACHINE: enter WAIT_CASE"); stateChange = false; }
        #endif
  
        if ( LCDTimer.hasPassed(LCD_UPDATE_INTERVAL) ) {
          updateLCD(false);
          LCDTimer.restart();
        }

        eepromCheckAnnealSetPoint();
        
        if (startOnOpto) {
          // XXXXXX - this is where support for a proximity sensor or IR sensor goes
          updateLCDState();
          // do what we need to do
        }
        else { // if we're not messing w/ the opto sensor, just go to the next step
          annealState = START_ANNEAL;
          updateLCDState();
          
          #ifdef DEBUG_STATE
          stateChange = true;
          #endif
        }
        break;
  
  
      ////////////////////////////////
      // START_ANNEAL
      //
      // Initial state change to start
      // the annealing process and timer
      // 
      // This is a single cycle state,
      // so we don't need to update
      // any sensors or the display
      ////////////////////////////////
  
      case START_ANNEAL:
        #ifdef DEBUG_STATE
        if (stateChange) { Serial.println("DEBUG: STATE MACHINE: enter START_ANNEAL"); stateChange = false; }
        #endif
        
        annealState = ANNEAL_TIMER;
        digitalWrite(INDUCTOR_PIN, HIGH);
        digitalWrite(LED_BUILTIN, HIGH);
        Timer.restart(); 
        AnnealPowerSensors.restart();
        AnnealLCDTimer.restart();
  
        #ifdef DEBUG_STATE
        stateChange = true;
        #endif
        
        break;
  
  
      ////////////////////////////////
      // ANNEAL_TIMER
      //
      // Keep track of the time, and
      // stop annealing at the right
      // set point.
      // Update the display for timer,
      // amps, and volts - nothing else
      // needs to change. 
      // Reset timers on LCD display
      ////////////////////////////////
  
      case ANNEAL_TIMER:
        #ifdef DEBUG_STATE
        if (stateChange) { Serial.println("DEBUG: STATE MACHINE: enter ANNEAL_TIMER"); stateChange = false; }
        #endif
  
        if (Timer.hasPassed((int) annealSetPoint * 1000)) {  // if we're done...
          digitalWrite(INDUCTOR_PIN, LOW);
          digitalWrite(LED_BUILTIN, LOW);
          annealState = DROP_CASE;
          Timer.restart();
          updateLCDState();
          LCDTimer.restart();
          
          #ifdef DEBUG_STATE
          stateChange = true;
          #endif
        }    
        
        if (AnnealPowerSensors.hasPassed(ANNEAL_POWER_INTERVAL)) {
          checkPowerSensors(false);
          AnnealPowerSensors.restart();
        }
  
        // don't update the LCD if we're within 200 millseconds of ending the anneal cycle, so 
        // we don't overrun while out to lunch. annealSetPoint is a float in seconds, and we
        // need to convert it to milliseconds
        if ( (Timer.elapsed() < (int)((annealSetPoint * 1000) - 200)) && AnnealLCDTimer.hasPassed(ANNEAL_LCD_TIMER_INTERVAL)) {
           updateLCDTimer(true);
           updateLCDPowerDisplay(true);
           AnnealLCDTimer.restart();
        }
  
        break;
  
      ////////////////////////////////
      // DROP_CASE
      //
      // Trigger the solenoid and start
      // the solenoid timer. Update
      // the display once, so Timer goes 
      // back to 0.00 (by annealState in
      // updateLCDTimer)
      ////////////////////////////////
   
      case DROP_CASE: 
        #ifdef DEBUG_STATE
        if (stateChange) { Serial.println("DEBUG: STATE MACHINE: enter DROP_CASE"); stateChange = false; }
        #endif
        
        digitalWrite(SOLENOID_PIN, HIGH);
        annealState = DROP_CASE_TIMER;
        updateLCDTimer(true);
  
        #ifdef DEBUG_STATE
        stateChange = true;
        #endif
        
        break;
  
  
      ////////////////////////////////
      // DROP_CASE_TIMER
      //
      // Close the solenoid at the right time
      // Update the display on normal cycle
      ////////////////////////////////
  
      case DROP_CASE_TIMER:
        #ifdef DEBUG_STATE
        if (stateChange) { Serial.println("DEBUG: STATE MACHINE: enter DROP_CASE_TIMER"); stateChange = false; }
        #endif
  
        // this timing isn't critical, so we'll just update normally, now
        if ( LCDTimer.hasPassed(LCD_UPDATE_INTERVAL) ) {
          updateLCD(false);
          LCDTimer.restart();
        }
        
        if (Timer.hasPassed((int) caseDropSetPoint * 1000)) {
          digitalWrite(SOLENOID_PIN, LOW);
          annealState = DELAY;
          Timer.restart();
          updateLCDState();
  
          #ifdef DEBUG_STATE
          stateChange = true;
          #endif
          
          break;
        }
  
        break;
  
  
      ////////////////////////////////
      // DELAY
      //
      // Duty cycle to allow heat to 
      // dissipate a bit. 
      // Normal display handling and sensor
      // updates
      ////////////////////////////////
      
      case DELAY:
        #ifdef DEBUG_STATE
        if (stateChange) { Serial.println("DEBUG: STATE MACHINE: enter DELAY"); stateChange = false; }
        #endif
  
        if ( LCDTimer.hasPassed(LCD_UPDATE_INTERVAL) ) {
          updateLCD(false);
          LCDTimer.restart();
        }
        
        if (Timer.hasPassed((int) delaySetPoint * 1000)) {
          annealState = WAIT_CASE;
  
          #ifdef DEBUG_STATE
          stateChange = true;
          #endif
  
        }
        break;
        
   
    } // switch(StepNumber)
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
