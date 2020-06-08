/**************************************************************************************************
 * 
 * AnnealLCD.cpp
 * Annealer Control Program
 * Author: Dave Re
 * Inception: 05/18/2020
 * 
 * This file contains the routines used to update the LCD when it's not under control of the
 * ArduinoMenu system - generally, when performance is paramount, and we want to show a 
 * single screen of the critical values and how they're changing, etc. 
 * 
 * All of the externs below are in Annealer-Control.ino
 * 
 **************************************************************************************************/

 
#include "Annealer-Control.h"
#include <avr/dtostrf.h>
#include <Chrono.h>
#include <SerLCD.h>

String output;
int LCDquotient = 0;
int LCDremainder = 0;
int timerCurrent = 0;


/*
 * updateLCD
 * 
 * Arguments
 * 
 * boolean full - write the whole screen?
 * 
 * Refresh the LCD - this code will get called in a couple places, so a subroutine makes sense.
 * This code is a little tedious - again, trying to leave printf out of the picture. Also, there
 * are preprocessor defines here (rather than above) - just keeping them local for ease of
 * reference and manipulation. 
 * 
 * Right now, this is just a basic display. Eventually, this should become a menu system of some
 * kind, rather than just a simple data display.
 * 
 * The LCD is 20x4
 * 
 * Try to do this as lightweight as possible. Here's the layout:
 * 
 * 01234567890123456789  <-- column numbers, not printed!!
 * Set 00.00 Time 00.00
 * Amp 00.00 Volt 00.00
 * Thrm 00.0 IntT  00.0  <-- IntT is only for Apollo3 architecture
 * State: xxxxxxxxxxxxx      TMax is shown, otherwise
 */
void updateLCD(boolean full) {
  
  #ifdef DEBUG_LCD
  Serial.println("DEBUG: updating the full LCD");
  #endif

  String outputFull;
  outputFull = "";

  if (full) {
    lcd.clear();
    lcd.setCursor(LCD_SETPOINT_LABEL);
    
    outputFull.concat("Set ");
    updateLCDSetPoint(false);
    outputFull.concat(output);
    outputFull.concat(" Time ");
    updateLCDTimer(false);
    outputFull.concat(output);
    lcd.print(outputFull);
    #ifdef DEBUG_LCD
    Serial.println(outputFull);
    #endif

    outputFull = "";

    // not sure why I need to do this, yet - the text "Amp "
    // doesn't print out, otherwise...
    lcd.setCursor(LCD_CURRENT_LABEL);
    lcd.print("Amp ");
    lcd.setCursor(LCD_CURRENT_LABEL);
    outputFull.concat("Amp ");
    updateLCDPowerDisplay("false");
    outputFull.concat(output);
    #ifdef DEBUG_LCD
    Serial.println(outputFull);
    #endif
    lcd.print(outputFull);


    outputFull = "";
    lcd.setCursor(LCD_THERM1_LABEL);
    outputFull.concat("Thrm ");
    updateLCDTemps(false);
    outputFull.concat(output);
    lcd.print(outputFull);
    #ifdef DEBUG_LCD
    Serial.println(outputFull);
    #endif
    
    lcd.setCursor(LCD_STATE_LABEL);
    lcd.print("State:");

    updateLCDState();
  }
  else {
    updateLCDSetPoint(true);
    updateLCDTimer(true);
    updateLCDPowerDisplay(true);
    updateLCDTemps(true);
    updateLCDState();
  }
  
  #ifdef DEBUG_LCD
  Serial.println("DEBUG: done updating LCD");
  #endif
}

// no boolean, here - we're likely always going to just do this
void updateLCDState() {
  #ifdef DEBUG_LCD
  Serial.println("DEBUG: LCD: print state");
  #endif
  lcd.setCursor(LCD_STATE);
  lcd.print(annealStateDesc[annealState]);


}


// set point is in hundredths of seconds! if it's less than 999, we need a space
void updateLCDSetPoint(boolean sendIt) {
  
  #ifdef DEBUG_LCD
  Serial.println("DEBUG: LCD: print set point");
  #endif
  char c[6];

  output = "";

  dtostrf(annealSetPoint, 5, 2, c);
  output.concat(c);

  if (sendIt) {
    lcd.setCursor(LCD_SETPOINT);
    lcd.print(output);
  }

}

// print most of the line at once! Save's us a cursor reposition and a print
void updateLCDPowerDisplay(boolean sendIt) {
  #ifdef DEBUG_LCD
  Serial.println("DEBUG: LCD: print amps and volts");
  #endif

  output = "";

  LCDremainder = (int) (amps * 100);
  LCDquotient = LCDremainder / 100;
  LCDremainder = LCDremainder % 100;
  if (LCDquotient < 10) output.concat(" ");
  output.concat(LCDquotient);
  output.concat(".");
  if (LCDremainder < 10) output.concat("0");
  output.concat(LCDremainder);

  output.concat(" Volt ");

  LCDremainder = (int) (volts * 100);
  LCDquotient = LCDremainder / 100;
  LCDremainder = LCDremainder % 100;
  if (LCDquotient < 10) output.concat(" ");
  output.concat(LCDquotient);
  output.concat(".");
  if (LCDremainder < 10) output.concat("0");
  output.concat(LCDremainder);

  #ifdef DEBUG_LCD
  Serial.print("DEBUG: updateLCDPowerDisplay output: "); Serial.println(output);
  #endif

  if (sendIt) {
    lcd.setCursor(LCD_CURRENT);
    lcd.print(output);
  }

  
}

void updateLCDTemps(boolean sendIt) {
  #ifdef DEBUG_LCD
  Serial.println("DEBUG: LCD: print temperatures");
  #endif

  output = "";

  LCDremainder = (int) (Therm1Temp * 10);
  LCDquotient = LCDremainder / 10;
  LCDremainder = LCDremainder % 10;
  if (LCDquotient >= 100) {
    output.concat(" ");
    output.concat(LCDquotient);
  }
  else {
    if (LCDquotient < 10) output.concat(" ");
    output.concat(LCDquotient);
    output.concat(".");
    output.concat(LCDremainder); // should be less than 10
  }

  #ifdef _AP3_VARIANT_H_
  output.concat(" IntT  "); 
  LCDremainder = (int) (internalTemp * 10);
  #else
  output.concat(" TMax  ");
  LCDremainder = (int) (Therm1TempHigh * 10);  
  #endif
  
  LCDquotient = LCDremainder / 10;
  LCDremainder = LCDremainder % 10;

  if (LCDquotient >= 100) {
    output.concat(" ");
    output.concat(LCDquotient);
  }
  else {
    if (LCDquotient < 10) output.concat(" ");
    output.concat(LCDquotient);
    output.concat(".");
    output.concat(LCDremainder); // should be less than 10
  }
  if (sendIt) {
    lcd.setCursor(LCD_THERM1);
    lcd.print(output);
  }
  
}


void updateLCDTimer(boolean sendIt) {
  #ifdef DEBUG_LCD
  Serial.println("DEBUG: LCD: print timer");
  #endif
  
  output = "";

  
  // if we're running a timer, do the math to print the right value, otherwise, a default
  if (annealState == START_ANNEAL || 
      annealState == ANNEAL_TIMER ) {
        
    timerCurrent = Timer.elapsed();
    LCDquotient = timerCurrent / 1000;
    LCDremainder = timerCurrent % 1000 / 10;
    if (LCDquotient < 10) output.concat(" ");
    output.concat(LCDquotient);
    output.concat(".");
    if (LCDremainder < 10) output.concat("0");
    output.concat(LCDremainder);
  }
  else {
    // this is for all the wait states *AND* DROP_CASE, so we can show 0.00 at 
    // timer start
    output.concat(" 0.00");
  }

  if (sendIt) {
    lcd.setCursor(LCD_TIMER);
    lcd.print(output);
  }
  
}
