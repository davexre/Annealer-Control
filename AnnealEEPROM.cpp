/**************************************************************************************************
 * 
 * AnnealEEPROM.cpp
 * Annealer Control Program
 * Author: Dave Re
 * Inception: 05/25/2020
 * 
 * EEPROM functions
 * 
 **************************************************************************************************/

#include "Annealer-Control.h"
#include <EEPROM.h>


int storedSetPoint = 0;       // the annealSetPoint value hanging out in EEPROM - need this for comparison later
int storedDelaySetPoint = 0;
int storedCaseDropSetPoint = 0;

const char* caseNameDefault = "unused      ";

StoredCase storedCases[10];

boolean eepromGood = false;

void eepromStartup(void) {
  
  // double check that we can trust the EEPROM by looking for a previously
  // stored "failsafe" value at a given address. We're going to use 
  // storedSetPoint here so we don't have to initialize a different variable
 
  EEPROM.get(EE_FAILSAFE_ADDR, storedSetPoint); // borrow storedSetPoint for a moment
  
  if (storedSetPoint == EE_FAILSAFE_VALUE) {
 
    #ifdef DEBUG
      Serial.print("DEBUG: EEPROM Failsafe - found <"); Serial.print(storedSetPoint); Serial.println(">");
    #endif

    EEPROM.get(ANNEAL_ADDR, storedSetPoint);
    EEPROM.get(DELAY_ADDR, storedDelaySetPoint);
    EEPROM.get(CASEDROP_ADDR, storedCaseDropSetPoint);
    EEPROM.get(START_ON_OPTO_ADDR, startOnOpto);

    eepromGood = true;

  }
  else { // don't trust the EEPROM!
    
    #ifdef DEBUG
      Serial.print("DEBUG: EEPROM Failsafe failed - found <"); Serial.print(storedSetPoint); Serial.println(">");
    #endif
    storedSetPoint = EE_FAILSAFE_VALUE; // here we go, borrowing again
    EEPROM.put(EE_FAILSAFE_ADDR, storedSetPoint);
    storedSetPoint = ANNEAL_TIME_DEFAULT;
    EEPROM.put(ANNEAL_ADDR, storedSetPoint);
    storedDelaySetPoint = DELAY_DEFAULT;
    EEPROM.put(DELAY_ADDR, storedDelaySetPoint);
    storedCaseDropSetPoint = CASE_DROP_DELAY_DEFAULT;
    EEPROM.put(CASEDROP_ADDR, storedCaseDropSetPoint);
    EEPROM.put(START_ON_OPTO_ADDR, startOnOpto);

    eepromGood = false;
  }
    
  // and reset defaults if it looks like our defaults got wiped, but the
  // the EEPROM failsafe survived
  if (storedSetPoint == 0) {
    storedSetPoint = ANNEAL_TIME_DEFAULT;
    annealSetPoint = storedSetPoint / 100.0;
    EEPROM.put(ANNEAL_ADDR, storedSetPoint);
  }
  else {
    annealSetPoint = storedSetPoint / 100.0;
  }
  
  if (storedDelaySetPoint == 0) {
    storedDelaySetPoint = DELAY_DEFAULT;
    delaySetPoint = storedDelaySetPoint / 100.0;
    EEPROM.put(DELAY_ADDR, storedDelaySetPoint);
  }
  else {
    delaySetPoint = storedDelaySetPoint / 100.0;
  }
  
  if (storedCaseDropSetPoint == 0) {
    storedCaseDropSetPoint = CASE_DROP_DELAY_DEFAULT;
    caseDropSetPoint = storedCaseDropSetPoint / 100.0;
    EEPROM.put(CASEDROP_ADDR, storedCaseDropSetPoint);
  }
  else {
    caseDropSetPoint = storedCaseDropSetPoint / 100.0;
  }
  

  #ifdef DEBUG
    Serial.print("DEBUG: Starting Anneal set point: ");
    Serial.println(annealSetPoint, 2);
    Serial.print("DEBUG: EEPROM stored Delay set point: ");
    Serial.println(delaySetPoint, 2);
    Serial.print("DEBUG: EEPROM stored Case Drop set point: ");
    Serial.println(caseDropSetPoint, 2);
  #endif
  
  // Grab the stored case type names and anneal times
  for (int i=0; i < NUM_CASES; i++) { 
    if (eepromGood) {
      EEPROM.get((CASE_NAME_ARRAY_START_ADDR + (i*15)), storedCases[i].name);
      EEPROM.get((CASE_STORED_ARRAY_START_ADDR + (i * sizeof(float))), storedCases[i].time);
    }
    else {
      EEPROM.put((CASE_NAME_ARRAY_START_ADDR + (i*15)), storedCases[i].name);
      EEPROM.put((CASE_STORED_ARRAY_START_ADDR + (i * sizeof(float))), storedCases[i].time);
    }
  }
  
}

void eepromCheckAnnealSetPoint(void) {

  #ifdef DEBUG
    Serial.print("DEBUG: EEPROM checking annealSetPoint: storedSetPoint <");
    Serial.print(storedSetPoint);
    Serial.print("> annealSetPoint <");
    Serial.print(annealSetPoint, 2);
    Serial.println(">");
    Serial.print("DEBUG: EEPROM floor of annealSetPoint * 100 = ");
    Serial.println(floor((annealSetPoint * 100.00) + 0.5));
  #endif
  
  if (storedSetPoint != floor((annealSetPoint * 100.0) + 0.5)) {
   storedSetPoint = floor((annealSetPoint * 100.0) + 0.5);
    #ifdef DEBUG
      Serial.print("DEBUG: storedSetPoint != annealSetPoint. Setting to: "); Serial.println(storedSetPoint);
    #endif
    EEPROM.put(ANNEAL_ADDR, storedSetPoint);
  }
  
}

void eepromCheckDelaySetPoint(void) {
  #ifdef DEBUG
    Serial.println("DEBUG: EEPROM checking delaySetPoint");
  #endif
  if (storedDelaySetPoint != floor((delaySetPoint * 100.0) + 0.5)) {
    storedDelaySetPoint = floor((delaySetPoint * 100.0) + 0.5);
    #ifdef DEBUG
      Serial.print("DEBUG: storedDelaySetPoint != delaySetPoint. Setting to: "); Serial.println(storedDelaySetPoint);
    #endif
    EEPROM.put(DELAY_ADDR, storedDelaySetPoint);
  }
}

void eepromCheckCaseDropSetPoint (void) {
  #ifdef DEBUG
    Serial.println("DEBUG: EEPROM checking caseDropSetPoint");
  #endif
  if (storedCaseDropSetPoint != floor((caseDropSetPoint * 100.0) + 0.5)) {
    storedCaseDropSetPoint = floor((caseDropSetPoint * 100.0) + 0.5);
    #ifdef DEBUG
      Serial.print("DEBUG: storedCaseDropSetPoint != caseDropSetPoint. Setting to: "); Serial.println(storedCaseDropSetPoint);
    #endif
    EEPROM.put(CASEDROP_ADDR, storedCaseDropSetPoint);
  }
}

void eepromStoreCase(int index) {
  EEPROM.put((CASE_NAME_ARRAY_START_ADDR + (index*15)), storedCases[index].name);
  EEPROM.put((CASE_STORED_ARRAY_START_ADDR + (index * sizeof(float))), storedCases[index].time);
}

void eepromStoreStartOnOpto() {
  EEPROM.put(START_ON_OPTO_ADDR, startOnOpto);
}
