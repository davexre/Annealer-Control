/**************************************************************************************************
 * 
 * MayanLCD.cpp
 * Annealer Control Program
 * Author: Dave Re
 * Inception: 05/18/2020
 * 
 * This file contains the routines used to update the LCD when it's under control of Mayan feature!
 * 
 * All of the externs below are in Annealer-Control.ino
 * 
 **************************************************************************************************/

 
#include "Annealer-Control.h"
#include <avr/dtostrf.h>
#include <Chrono.h>
#include <SerLCD.h>

#define BLANKLINE "                    "

/*
 * 01234567890123456789  <-- column numbers, not printed!!
 *        MAYAN!
 * START to begin
 * STOP  to exit Mayan
 * Cyc: XX  ARec: XX.XX  <-- after we've done a cycle!
 */

char c[6];

void mayanLCDWaitButton(boolean full) {

  if (full) {
    lcd.clear();
    lcd.setFastBacklight(WHITE);
    lcd.setCursor(7,0);
    lcd.print(F("MAYAN!"));
  }

  lcd.setCursor(0,1);
  lcd.print(F("START to begin      "));
  lcd.setCursor(0,2);
  lcd.print(F("STOP  to exit Mayan "));

  lcd.setCursor(0,3);
  if (mayanCycleCount > 0) {

    output = F("Cyc: ");
    if (mayanCycleCount < 10) {
      output.concat(" ");
    }
    output.concat(mayanCycleCount);
  
    output.concat(F("  ARec: "));

    if (mayanAccRec < 10.0) {
      output.concat(F(" "));
    }
    dtostrf(mayanAccRec, 5, 2, c);
    output.concat(c);
    lcd.print(output);
  }
  else {
    lcd.print(BLANKLINE);
  }
  
}

/*
 * 01234567890123456789  <-- column numbers, not printed!!
 *        MAYAN!
 *    RUNNING  CYCLE
 *    STOP to cancel
 * Cyc: XX  ARec: XX.XX
 */
void mayanLCDStartMayan() {
  lcd.setFastBacklight(RED);
  lcd.setCursor(0,1);
  lcd.print(F("   RUNNING  CYCLE  "));
  lcd.setCursor(0,2);
  lcd.print(F("   STOP to cancel   "));
  
  if (mayanCycleCount > 1) {
    lcd.setCursor(5,3);
    
    output = "";
    
    if (mayanCycleCount < 10) {
      output.concat(F(" "));
    }
    output.concat(mayanCycleCount);
    lcd.print(output);
  }
  else {
    lcd.setCursor(0,3);
    lcd.print(F("Cyc:  1  ARec: 00.00"));
  }
}

/*
 * 01234567890123456789  <-- column numbers, not printed!!
 *        MAYAN!
 *     CALCULATING     
 *    STOP to cancel
 * Cyc: XX  ARec: XX.XX
 */
void mayanLCDCalculate() {
  lcd.setFastBacklight(YELLOW);
  lcd.setCursor(0,1);
  lcd.print(F("    CALCULATING     "));
}

/*
 * 01234567890123456789  <-- column numbers, not printed!!
 *        MAYAN!
 *     SAVING DATA      
 *    STOP to cancel
 * Cyc: XX  ARec: XX.XX
 */
void mayanLCDSaving() {
  lcd.setCursor(0,1);
  lcd.print(F("    SAVING DATA     "));

}

/*
 * 01234567890123456789  <-- column numbers, not printed!!
 *        MAYAN!
 *   Recommend: XX.XX
 *   STOP to drop case  
 * Cyc: XX  ARec: XX.XX
 */
void mayanLCDWait() {
  lcd.setFastBacklight(GREEN);
  lcd.setCursor(0,1);

  output = "";
  output.concat(F("  Recommend: "));

  dtostrf(mayanRecommendation, 5, 2, c);
  output.concat(c);
  output.concat(F("  "));
  lcd.print(output);

  lcd.setCursor(0,2);
  lcd.print(F("  STOP to drop case "));

  lcd.setCursor(14,3); // new accumulated recommendation
  output="";
  if (mayanAccRec < 10.0) {
    output.concat(F(" "));
  }
  dtostrf(mayanAccRec, 5, 2, c);
  output.concat(c);
  lcd.print(output);
  
}


void mayanLCDDropCase() {
  // just blank the "STOP to drop case" line
  lcd.setCursor(0,2);
  lcd.print(BLANKLINE);
}

/*
 * 01234567890123456789  <-- column numbers, not printed!!
 *        MAYAN!
 * START for next case
 * STOP to end analysis  
 * Cyc: XX  ARec: XX.XX
 */
void mayanLCDPauseWait() {
  lcd.setFastBacklight(WHITE);

  lcd.setCursor(0,1);
  lcd.print(F("START for next case "));
  lcd.setCursor(0,2);
  lcd.print(F("STOP to end analysis"));
  
}

/*
 * 01234567890123456789  <-- column numbers, not printed!!
 *       ABORTED!
 * START for next case
 * STOP to end analysis  
 * Cyc: XX  ARec: XX.XX
 */
void mayanLCDAbort() {
  lcd.setFastBacklight(ORANGE);
  lcd.setCursor(6,0);
  lcd.print(F("ABORTED!"));
  lcd.setCursor(0,1);
  lcd.print(F("START for next case "));
  lcd.setCursor(0,2);
  lcd.print(F("STOP to end analysis"));
}

// reprint the MAYAN! header
void mayanLCDLeaveAbort() {
  lcd.setFastBacklight(WHITE);
  lcd.setCursor(6,0);
  lcd.print(F(" MAYAN! "));
}
