
/**************************************************************************************************
 * 
 * MayanStateMachine.cpp
 * Annealer Control Program
 * Author: Dave Re
 * Inception: 05/18/2020
 * 
 * This file contains the "Mayan" mode state machine.
 * 
 * All of the externs below are in Annealer-Control.ino
 * 
 **************************************************************************************************/

#include "Annealer-Control.h"
#include <avr/dtostrf.h>
#include <Chrono.h>
#include <CircularBuffer.h>
#include <Rencoder.h>

#include <ctype.h>
#include <vector>
#include <iterator>

#ifdef DEBUG_STATE
  boolean stateChange = true;
#endif

#define CIRCULAR_BUFFER_LENGTH 5
#define CYCLE_INTERVAL 50 // millis
#define mayanF 0.48
#define mayanK -0.016

using namespace std;

struct MayanDataPoint {
  unsigned int timestamp = 0;
  float dpAmps = 0.0;
  float dpVolts =0.0;
};

boolean mayanScreenUpdate = false;
boolean mayanUseSD = false;
int mayanStartMillis = 0;
int mayanCurrentMillis = 0;
int mayanLoopCount = 0;
int mayanCycleCount = 0;
float mayanAccRec = 0.0; // accumulated recommendation based on 1 or more runs
float mayanRecommendation = 0.0;
float lastMayanRecommendation = 0.0;

CircularBuffer<float, CIRCULAR_BUFFER_LENGTH> ampsBuffer;

vector<MayanDataPoint*> mayanDataPoints;
MayanDataPoint *newdp;

#ifdef DEBUG_MAYAN
void mayanPrintDataToSerial() {

  std::vector<MayanDataPoint*>::size_type i, sz;
  sz = mayanDataPoints.size();
  char c[6];
  
  Serial.println("MAYAN data dump");

  for (i = 0; i < sz; i++) {
    output = "";
    output.concat(mayanDataPoints[i]->timestamp);
    output.concat(",");
    dtostrf(mayanDataPoints[i]->dpAmps, 5, 2, c);
    output.concat(c);
    output.concat(",");
    dtostrf(mayanDataPoints[i]->dpVolts, 5, 2, c);
    output.concat(c);
    Serial.println(output);
  }

  Serial.println("");
  
}
#endif


void mayanStateMachine() {

    ///////////////////////////////////////////////////////////////////////
    // First, Housekeeping - https://www.youtube.com/watch?v=8wVRlL7QaQM  /
    ///////////////////////////////////////////////////////////////////////
    
    // gather button statuses
    
    if ( encoder.isClicked() ) {
      encoderPressed = true;
      (void) encoder.isDoubleClicked(); // clear this flag in this context
      
      #ifdef DEBUG
      Serial.println("DEBUG: Encoder clicked");
      #endif

    }

  
    if (startPressed) {

      switch(mayanState) {
        case WAIT_BUTTON_MAYAN:
        case PAUSE_WAIT:
        case WAIT_DROP_CASE:
        case ABORTED: 
                
          #ifdef DEBUG
          Serial.println("DEBUG: start button pressed");
          #endif
          
          #ifdef DEBUG_STATE
          stateChange = true;
          #endif
          
          break;

        default:
          startPressed = false;
          break;
          
      }
      
    } 
  

    // if the inductor's running, stop it and hit a state that allows us to abort or continue
    if (stopPressed || encoderPressed) {
      switch(mayanState) {
        case WAIT_BUTTON_MAYAN:
        case PAUSE_WAIT:
        case WAIT_DROP_CASE:
        case ABORTED:
          startPressed = false; // let stop override start
          stopPressed = true;
          encoderPressed = false;
          break;

        default:
          digitalWrite(INDUCTOR_PIN, LOW);
          digitalWrite(LED_BUILTIN, LOW);
          digitalWrite(SOLENOID_PIN, LOW);
          mayanState = ABORTED;
          mayanScreenUpdate = true;
          startPressed = false;
          stopPressed = false;
          encoderPressed = false;
          
          #ifdef DEBUG
          Serial.println("DEBUG: stop button pressed - Mayan cycle aborted");
          #endif
          #ifdef DEBUG_STATE
          stateChange = true;
          #endif
          
          break;
          
        } // switch(mayaState)
        
      }
    
    
    ////////////////////////////////////////////////////////
    // Basic state machine for the Mayan cycle
    ////////////////////////////////////////////////////////
  

  
    switch(mayanState) {
  
      ////////////////////////////////
      // WAIT_BUTTON_MAYAN
      //
      // Wait for the start button
      // 
      ////////////////////////////////
      case WAIT_BUTTON_MAYAN: {
        #ifdef DEBUG_STATE
          if (stateChange) { Serial.println("DEBUG: STATE MACHINE: enter WAIT_BUTTON_MAYAN"); stateChange = false; }
        #endif

        // need to update the LCD in some fashion, here, on entry ("press button to start")
        if (mayanScreenUpdate) {
          mayanLCDWaitButton(false);
          mayanScreenUpdate = false;
        }
        
        if (stopPressed) {
          #ifdef DEBUG
          Serial.println("DEBUG: MAYAN: Stop Pressed in WAIT_BUTTON_MAYAN");
          #endif
          
          lcd.setFastBacklight(255, 255, 255);
          nav.idleOff();
          menuState = MAIN_MENU;
          showedScreen = false;
          (void) encoder.clear(); // clear our flags
          stopPressed = false;
          startPressed = false;
        }
        else if (startPressed) {
          mayanState = START_MAYAN;
          startPressed = false;
          
          #ifdef DEBUG_STATE
          stateChange = true;
          #endif
        }
        
        break;
      }
  
  

  
      ////////////////////////////////
      // START_MAYAN
      //
      // Initial state change to start
      // the analysis.
      // 
      // This is a single cycle state
      ////////////////////////////////
  
      case START_MAYAN: {
        #ifdef DEBUG_STATE
        if (stateChange) { Serial.println("DEBUG: STATE MACHINE: enter START_MAYAN"); stateChange = false; }
        #endif

        mayanLCDStartMayan();
        
        mayanState = MAYAN_TIMER;

        ampsBuffer.clear();
        mayanDataPoints.clear();
        
        checkPowerSensors(true); // reset our amps/volts readings
        ampsBuffer.push(amps);

        newdp = new MayanDataPoint;
        newdp->timestamp = 0;
        newdp->dpAmps = amps;
        newdp->dpVolts = volts;
        mayanDataPoints.push_back(newdp);
             
        mayanLoopCount = 1;
        mayanCycleCount++;
        mayanStartMillis = millis();
        
        digitalWrite(INDUCTOR_PIN, HIGH);
        digitalWrite(LED_BUILTIN, HIGH);
 
        
        #ifdef DEBUG_STATE
        stateChange = true;
        #endif
        
        break;
      }
 
      ////////////////////////////////
      // MAYAN_TIMER
      //
      // Keep track of the environmentals
      // and stop the process at the appropriate
      // time
      ////////////////////////////////
  
      case MAYAN_TIMER: {
        #ifdef DEBUG_STATE
        if (stateChange) { Serial.println("DEBUG: STATE MACHINE: enter MAYAN_TIMER"); stateChange = false; }
        #endif

        // on the Artemis, analogRead appears to take .08 milliseconds - two reads take about .15 milliseconds
        // some of that timing has to do with logic around the test code, as well. These are very fast.

        mayanCurrentMillis = millis();

        if ( ((mayanCurrentMillis - mayanStartMillis) / CYCLE_INTERVAL) > mayanLoopCount) {
          mayanLoopCount++;

          #ifdef DEBUG_MAYAN
          Serial.print("MAYAN: Loop Count "); Serial.println(mayanLoopCount);
          #endif
          
          checkPowerSensors(false);
          ampsBuffer.push(amps); // push current amps to circular buffer to examine slope

          // save our data point
          newdp = new MayanDataPoint;
          newdp->timestamp = mayanCurrentMillis - mayanStartMillis;
          newdp->dpAmps = amps;
          newdp->dpVolts = volts;
          mayanDataPoints.push_back(newdp);

          // are we done? 
          // ideally, we'd be tracking the slope of the curve described by amps over time
          // For our purposes, we can shortcut that, and just compare current amps to 
          // a past value and see if we're lower. We're actually looking BUFLEN * CYCLE_INTERVAL
          // milliseconds back. amps is already smoothed a bit, too, so hopefully, we're not 
          // jumping the gun, here

          // this algorithm can likely be improved

          #ifdef DEBUG_MAYAN
          Serial.print("MAYAN: ampsBuffer.last = "); Serial.print(ampsBuffer.last()); Serial.print(" ampsBuffer.first = "); Serial.println(ampsBuffer.first());
          #endif
          
          if ((ampsBuffer.last() - ampsBuffer.first()) < 0.0) {
            digitalWrite(INDUCTOR_PIN, LOW);
            digitalWrite(LED_BUILTIN, LOW);
            
            mayanState = CALCULATE;

            #ifdef DEBUG_STATE
            stateChange = true;
            #endif
          }
        }
        break;
      }

      ////////////////////////////////
      // CALCULATE
      //
      // Take the results from the run,
      // and make a timing recommendation
      ////////////////////////////////

      case CALCULATE: {
        #ifdef DEBUG_STATE
        if (stateChange) { Serial.println("DEBUG: STATE MACHINE: enter CALCULATE"); stateChange = false; }
        #endif
        
        // after running calculations, we advance to the next state
        // need to update the LCD appropriately and ensure that
        // we show the results for long enough, until the user
        // chooses to proceed
        mayanLCDCalculate();

        float highestAmps = 0;

        std::vector<MayanDataPoint*>::size_type highestIndex = 0;
        std::vector<MayanDataPoint*>::size_type i;
        for (i = 0; i < mayanDataPoints.size(); i++) {
          if (mayanDataPoints[i]->dpAmps > highestAmps) {
            highestAmps = mayanDataPoints[i]->dpAmps;
            highestIndex = i; 
          }
        }

        // use LR88's algorithm here - 
        float timeTenthsSeconds = (float) mayanDataPoints[highestIndex]->timestamp / 100.0;
        mayanRecommendation = (timeTenthsSeconds * (mayanF + mayanK * (timeTenthsSeconds-90.0) * 0.1)) / 10.0;

        mayanAccRec = ( (mayanAccRec * (float) (mayanCycleCount - 1)) + mayanRecommendation) / mayanCycleCount;

        lastMayanRecommendation = mayanAccRec;
        
        // show recommendation on the LCD

        mayanState = SAVE_DATA;
        
        #ifdef DEBUG_STATE
        stateChange = true;
        #endif
        
        break;
      }

      ////////////////////////////////
      // SAVE_DATA
      //
      // Save all the data to SD
      ////////////////////////////////

      case SAVE_DATA: {
        #ifdef DEBUG_STATE
        if (stateChange) { Serial.println("DEBUG: STATE MACHINE: enter SAVE_DATA"); stateChange = false; }
        #endif
        
        mayanLCDSaving();
                
        // if we don't care about saving the data, move on
        if (mayanUseSD) {
          // if Cycle count is 1, open a new file
          // write to a CSV file on the SD card
        }

        #ifdef DEBUG_MAYAN
        mayanPrintDataToSerial();
        #endif
 
        mayanState = WAIT_DROP_CASE;

        #ifdef DEBUG_STATE
        stateChange = true;
        #endif
        
        break;
      }
      
      ////////////////////////////////
      // WAIT_DROP_CASE
      //
      // Wait for the user to press
      // a button to drop the
      // case, then proceed
      ////////////////////////////////
   
      case WAIT_DROP_CASE: {
        #ifdef DEBUG_STATE
        if (stateChange) { Serial.println("DEBUG: STATE MACHINE: enter WAIT_DROP_CASE"); stateChange = false; }
        #endif

        mayanLCDWait();
        
        if (stopPressed || startPressed) {
          mayanLCDDropCase();
          digitalWrite(SOLENOID_PIN, HIGH);
          mayanState = DROP_CASE_TIMER_MAYAN;
          Timer.restart();
          stopPressed = false;
          startPressed = false;
    
          #ifdef DEBUG_STATE
          stateChange = true;
          #endif
        }
        
        break;
      }
  
      ////////////////////////////////
      // DROP_CASE_TIMER_MAYAN
      //
      // Close the solenoid at the right time
      // Update the display on normal cycle
      ////////////////////////////////
  
      case DROP_CASE_TIMER_MAYAN: {
        #ifdef DEBUG_STATE
        if (stateChange) { Serial.println("DEBUG: STATE MACHINE: enter DROP_CASE_TIMER_MAYAN"); stateChange = false; }
        #endif
  
      
        if (Timer.hasPassed((int) caseDropSetPoint * 1000)) {
          digitalWrite(SOLENOID_PIN, LOW);
          mayanState = PAUSE_WAIT;
          mayanScreenUpdate = true;
  
          #ifdef DEBUG_STATE
          stateChange = true;
          #endif
          
        }
  
        break;
      }
  
      ////////////////////////////////
      // PAUSE_WAIT
      //
      // Wait for user to proceed while
      // results are displayed for them
      ////////////////////////////////


      case PAUSE_WAIT: {
        #ifdef DEBUG_STATE
        Serial.println("DEBUG: STATE MACHINE: enter PAUSE_WAIT"); stateChange = false;
        #endif
        
        if (mayanScreenUpdate) {
          mayanScreenUpdate = false;
          mayanLCDPauseWait();
        }

        if (stopPressed) { // we're ending this cycle 

          if (mayanUseSD) {
            // do what's needed to close the file
          }
          mayanStartMillis = 0;
          mayanCurrentMillis = 0;
          mayanLoopCount = 0;
          mayanCycleCount = 0;
          mayanAccRec = 0.0;
          mayanRecommendation = 0.0;

          mayanState = WAIT_BUTTON_MAYAN;
          #ifdef DEBUG_STATE
          stateChange = true;
          #endif
          
        }
        else if (startPressed) { // going to another case in this cycle
          
          mayanState = WAIT_BUTTON_MAYAN;
          #ifdef DEBUG_STATE
          stateChange = true;
          #endif
          
        }     
        break;
      }

    
      ////////////////////////////////
      // ABORTED
      //
      // Ask how to proceed
      ////////////////////////////////
      
      case ABORTED: {
        #ifdef DEBUG_STATE
        Serial.println("DEBUG: STATE MACHINE: enter ABORTED"); stateChange = false; 
        #endif

        // show something appropriate on the LCD, based on which state we're in

        if (mayanScreenUpdate) {
          mayanScreenUpdate = false;
          mayanLCDAbort();
        }
        
        if (stopPressed) { // we're ending this cycle 

          if (mayanUseSD) {
            // do what's needed to close the file
          }

          mayanLCDLeaveAbort();
          
          mayanStartMillis = 0;
          mayanCurrentMillis = 0;
          mayanLoopCount = 0;
          mayanCycleCount = 0;
          mayanAccRec = 0.0;
          mayanRecommendation = 0.0;

          mayanState = WAIT_BUTTON_MAYAN;
          #ifdef DEBUG_STATE
          stateChange = true;
          #endif
          
        }
        else if (startPressed) { // going to another case in this cycle

          mayanLCDLeaveAbort();
          
          mayanState = WAIT_BUTTON_MAYAN;
          #ifdef DEBUG_STATE
          stateChange = true;
          #endif
          
        }
        break;
      }
      
    } // switch(mayanState)

  
 }
