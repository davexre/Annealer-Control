/**************************************************************************************************
 * 
 * Environmentals.h
 * Annealer Control Program
 * Author: Dave Re
 * Inception: 05/25/2020
 * 
 * Environmental checking functions. All externs are defined in Annealer-Control.ino
 * 
 **************************************************************************************************/

#include "Annealer-Control.h"

#ifdef DEBUG
extern int temp;
#endif

/*
 * calcSteinhart
 * 
 * Arguments: 
 * input - generally a raw or smoothed value from analogRead.
 * 
 * Calculate a psuedo Steinhart-Hart based temperature value for our current
 * thermistor reading. Currently, we only have one thermistor in play, but in the
 * future, there may be more. If so, we should move most of this code to a function
 * that accepts inputting nominal temp, resistance, and beta values for each 
 * thermistor, and then have this one use whatever default we need it to use.
 * 
 * The Steinhart equation expects all temperature values to be in degrees Kelvin,
 * so the 273.15 constant below converts to/from Centigrade. The last line converts *that*
 * to degrees Farenheit (easy to remove if you want Centigrade...)
 */
float calcSteinhart(float input) {
  float output = 0.0;
  output = (THERM_RESISTOR / ((RESOLUTION_MAX / input) - 1)); // this gives us measured resistance in ohms!
  output = output / THERM_NOMINAL;
  output = log(output);
  output /= THERM_BETA;
  output += 1.0 / (THERM_NOM_TEMP + 273.15);
  output = 1.0 / output;
  output -= 273.15; 
  output = output * 1.8 + 32.0; 

  return output;
}


/*
 * checkPowerSensors
 * 
 * Aggregating the code for voltage and current monitoring to a subroutine, as it's used
 * in a couple different places
 */
void checkPowerSensors(boolean reset) {

  int ampsCalc = 0;

  #ifdef DEBUG_VERBOSE
  Serial.print("DEBUG: checkPowerSensors: reset? "); Serial.println(reset);
  #endif 

  if (reset) {
      amps = ( ( ( analogRead(CURRENT_PIN) / RESOLUTION_MAX * VREF ) - 1.0) / 100 );
      if ( amps < 0 ) amps = 0;
      volts = ( analogRead(VOLTAGE_PIN) * VOLTS_PER_RESOLUTION );
  }
  else {
    ampsCalc = ( ( ( analogRead(CURRENT_PIN) / RESOLUTION_MAX * VREF ) - 1.0) / 100 );
    if (ampsCalc < 0) ampsCalc = 0;
    amps = ( ( (1.0 - AMPS_SMOOTH_RATIO) * amps ) + (AMPS_SMOOTH_RATIO * ampsCalc) );
    volts = ( (1.0 - VOLTS_SMOOTH_RATIO) * volts ) + (VOLTS_SMOOTH_RATIO * ( analogRead(VOLTAGE_PIN) * VOLTS_PER_RESOLUTION ) );
  }

  #ifdef DEBUG_VERBOSE
  Serial.print("DEBUG: amps - ");
  Serial.println(amps);
  Serial.print("DEBUG: volts - ");
  Serial.println(volts);
  #endif
}

void checkThermistors(boolean reset) {

  if (reset) {
    for (int i=0; i<3; i++) {
      
      #ifdef DEBUG
      temp = analogRead(THERM1_PIN);
      Serial.print("DEBUG: THERM1_PIN read: "); Serial.println(temp);
      Therm1Avg = Therm1Avg + temp;

        #ifdef _AP3_VARIANT_H_
          temp = getInternalTemp();
          Serial.print("DEBUG: ADC_INTERNAL_TEMP read: "); Serial.println(temp);
          internalTemp = internalTemp + temp ;
        #endif
      
      #else
      Therm1Avg = Therm1Avg + analogRead(THERM1_PIN);
      
        #ifdef _AP3_VARIANT_H_
        internalTemp = internalTemp + getInternalTemp();
        #endif
      
      #endif

    }
  
    // Average over the three readings...
    Therm1Avg /= 3;
    Therm1Temp = calcSteinhart(Therm1Avg);    
    Therm1TempHigh = Therm1Temp;

    #ifdef _AP3_VARIANT_H_
    internalTemp /= 3;
    internalTemp = internalTemp * 1.8 + 32; // convert to F
    internalTempHigh = internalTemp;
    #endif
    
  }
  else {
    
    Therm1Avg = ((1.0 - THERM_SMOOTH_RATIO) * Therm1Avg) + (THERM_SMOOTH_RATIO * analogRead(THERM1_PIN));
    Therm1Temp = calcSteinhart(Therm1Avg); 
    if (Therm1Temp > Therm1TempHigh) {
      Therm1TempHigh = Therm1Temp;
    }


    #ifdef _AP3_VARIANT_H_
      internalTemp = (((1.0 - INT_TEMP_SMOOTH_RATIO) * internalTemp) + (INT_TEMP_SMOOTH_RATIO * (getInternalTemp() * 1.8 + 32)) );
      if (internalTemp > internalTempHigh) {
        internalTempHigh = internalTemp;
      }
    #endif
    
  }
}
