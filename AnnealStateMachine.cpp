/**************************************************************************************************
 * 
 * AnnealStateMachine.cpp
 * Annealer Control Program
 * Author: Dave Re
 * Inception: 05/18/2020
 * 
 * This file contains the actual state machine that's used during the annealing cycle.
 * 
 * All of the externs below are in Annealer-Control.ino
 * 
 **************************************************************************************************/

#include "Annealer-Control.h"
#include <Chrono.h>
#include <Rencoder.h>

#include <ctype.h>

#ifdef DEBUG_STATE
  boolean stateChange = true;
#endif

void annealStateMachine() {

    ///////////////////////////////////////////////////////////////////////
    // First, Housekeeping - https://www.youtube.com/watch?v=ERd5Faz8_-E  /
    ///////////////////////////////////////////////////////////////////////
    
    // gather button statuses
    
    if ( encoder.isClicked() ) {
      if (annealState == WAIT_BUTTON) { // exit annealing mode
        nav.idleOff();
        menuState = MAIN_MENU;
        showedScreen = false;
        (void) encoder.clear(); // clear our flags
      }
      else { // if we're in a cycle, we'll use this click to stop the cycle safely
        encoderPressed = true;
        (void) encoder.isDoubleClicked(); // clear this flag in this context
        
        #ifdef DEBUG
        Serial.println(F("DEBUG: Encoder clicked"));
        #endif
      }
    }
  
    if (startPressed && (annealState == WAIT_BUTTON)) {
      
     #ifdef DEBUG
      Serial.println(F("DEBUG: start button pressed"));
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
      Serial.println(F("DEBUG: stop button pressed - anneal cycle aborted"));
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
        Serial.print(F("DEBUG: encoder moved - diff is "));
        Serial.println(encoderDiff);
        annealSetPoint += encoderDiff / 100.0;
        Serial.print(F("DEBUG: new annealSetPoint = "));
        Serial.println(annealSetPoint);
      #else
        annealSetPoint += encoder.getDiff(true) / 100.0;
      #endif
      
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
          if (stateChange) { Serial.println(F("DEBUG: STATE MACHINE: enter WAIT_BUTTON")); stateChange = false; }
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
        if (stateChange) { Serial.println(F("DEBUG: STATE MACHINE: enter WAIT_CASE")); stateChange = false; }
        #endif
  
        if ( LCDTimer.hasPassed(LCD_UPDATE_INTERVAL) ) {
          updateLCD(false);
          LCDTimer.restart();
        }

        // only save the annealer set point if it's changed and we go to use it
        eepromCheckAnnealSetPoint();

        if (startOnOpto) {
        
          // check the sensor
          int opto1State = 0;
          opto1State = digitalRead(OPTO1_PIN);

          #ifdef DEBUG
            Serial.print(F("DEBUG: OPTO1_PIN state: ")); Serial.println(opto1State);
          #endif

          if (opto1State == LOW) { // there's a case waiting if the pin is LOW
            annealState = START_ANNEAL;
            updateLCDState();
            
            #ifdef DEBUG_STATE
            stateChange = true;
            #endif
          }
          
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
        if (stateChange) { Serial.println(F("DEBUG: STATE MACHINE: enter START_ANNEAL")); stateChange = false; }
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
        if (stateChange) { Serial.println(F("DEBUG: STATE MACHINE: enter ANNEAL_TIMER")); stateChange = false; }
        #endif
  
        if (Timer.hasPassed(floor((annealSetPoint * 1000.0) + 0.5))) {  // if we're done...
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
          updateLCDPowerDisplay(true);
        }

  
        // don't update the LCD if we're within 200 millseconds of ending the anneal cycle, so 
        // we don't overrun while out to lunch. annealSetPoint is a float in seconds, and we
        // need to convert it to milliseconds
        if ( (Timer.elapsed() < (int)((annealSetPoint * 1000) - 200)) && AnnealLCDTimer.hasPassed(ANNEAL_LCD_TIMER_INTERVAL)) {
           updateLCDTimer(true);
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
        if (stateChange) { Serial.println(F("DEBUG: STATE MACHINE: enter DROP_CASE")); stateChange = false; }
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
        if (stateChange) { Serial.println(F("DEBUG: STATE MACHINE: enter DROP_CASE_TIMER")); stateChange = false; }
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
        if (stateChange) { Serial.println(F("DEBUG: STATE MACHINE: enter DELAY")); stateChange = false; }
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
        
   
    } // switch(annealState)

  
 }
