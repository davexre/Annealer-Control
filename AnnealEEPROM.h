/**************************************************************************************************
 * 
 * AnnealEEPROM.h
 * Annealer Control Program
 * Author: Dave Re
 * Inception: 05/25/2020
 * 
 * EEPROM functions header
 * 
 **************************************************************************************************/

extern int storedSetPoint; 
extern int storedDelaySetPoint;
extern int storedCaseDropSetPoint;

void eepromStartup(void); 
void eepromCheckAnnealSetPoint(void);
void eepromCheckDelaySetPoint(void);
void eepromCheckCaseDropSetPoint (void);
