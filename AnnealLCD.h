/**************************************************************************************************
 * 
 * AnnealLCD.h
 * Annealer Control Program
 * Author: Dave Re
 * Inception: 05/18/2020
 * 
 * Function prototypes for AnnealLCD.cpp
 * 
 **************************************************************************************************/

#ifndef _ANNEALLCD_H
#define _ANNEALLCD_H

void updateLCD(boolean full);
void updateLCDState(void);
void updateLCDSetPoint(boolean sendIt);
void updateLCDPowerDisplay(boolean sendIt);
void updateLCDTemps(boolean sendIt);
void updateLCDTimer(boolean sendIt);

#endif
