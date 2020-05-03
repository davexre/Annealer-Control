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
 * 
 * So, why the whole "#ifdef DEBUG" thing, rather than a subroutine to print out debug messages?
 * I just don't want the DEBUG code to compile in the final code, if we don't need to debug things.
 * Yeah, this way is messy to read - I'm trading that for a smaller executable, and a tighter
 * execution path for the final code. This is also why I didn't pull printf in to the project, too!
 * 
 * 
 * This code assumes a couple pieces of hardware:
 * - SparkFun RedBoard Artemis (though, it attempts to adjust for a normal Arduino where it can)
 * - SparkFun AVR 20x4 LCD, connected through I2C
 * - SparkFun Qwiic Twist encoder, connected through I2C
 * 
 * 
 * TO DO list:
 * - handle anything commented with "XXXXXX"
 * - change Bounce to use the faster detect method, so we don't have to hold the button
 *   That'd be bad for the stop button
 * - refactor how we write to the LCD for performance - only update what needs updating and when
 * - do something useful with thermistor temp, at some point
 * - convert to using just the Twist encoder, and not start/stop buttons
 * - set up IR optical detection on the case
 * - menu system of some kind
 * - saved settings for different brass (possibly allowing name edits) and ability to choose
 * - support for a casefeeder (second opto, and logic)
 * 
 * Version History:
 * v 0.1 - initial stab at the code, just replicating the Sestos timer functionality, mostly
 * 
 **************************************************************************************************/

#include <Bounce2.h>
#include <Chrono.h>
#include <EEPROM.h>
#include <elapsedMillis.h>
#include <SerLCD.h> // SerLCD from SparkFun - http://librarymanager/All#SparkFun_SerLCD
#include "SparkFun_Qwiic_Twist_Arduino_Library.h"
#include <Wire.h>

#include <ctype.h>  // presumably to get enumerations

#define VERSION   0.1

/*
 * DEBUG - uncomment the #define to set us to DEBUG mode! Make sure you open a serial 
 * terminal at 9600 baud. Note that we don't expect to run with a Serial port regularly, 
 * so printing anything to Serial normally isn't going to be super useful.
 * 
 * If this isn't obvious, you need to recompile after commenting/uncommenting this statement!
 */
#define DEBUG
// #define DEBUG_LOOPTIMING
// #define DEBUG_VERBOSE
// #define DEBUG_STATE
// #define DEBUG_LCD

/*
 *  PIN CONFIGURATION
 */
#define  THERM1_PIN      A0
#define  CURRENT_PIN     A1
#define  VOLTAGE_PIN     A2
#define  OPTO_PIN        A5
#define  INDUCTOR_PIN    6
#define  SOLENOID_PIN    7
#define  START_PIN       8   // Start button - may be deprecated eventually
#define  STOP_PIN        9   // Stop button  - may be deprecated eventually


/*
 * CONSTANTS
 */
// System Constants
#ifdef _AP3_VARIANT_H_
#define RESOLUTION_MAX  16384 // Artemis boards have a 14-bit ADC resolution (make sure to turn it on!!!)
#define VREF            2.0
#else
#define RESOLUTION_MAX  1024 // 10-bit ADC seems to be the lowest common denominator
#define VREF            5.0  // YMMV
#endif

// Thermistor Values
#define THERM_NOMINAL       10000   //Resistance at nominal temperature
#define THERM_NOM_TEMP      25      //Nominal temperature in DegC
#define THERM_BETA          3950    //Beta coefficient for thermistor
#define THERM_RESISTOR      10000   //Value of resistor in series with thermistor
#define THERM_SMOOTH_RATIO  0.35    // What percentage of the running average is the latest reading - used to smooth analog input

#define INT_TEMP_SMOOTH_RATIO 0.35

// Power sensor values
#define AMPS_SMOOTH_RATIO   0.50
#define VOLTS_SMOOTH_RATIO  0.50

#ifdef _AP3_VARIANT_H_
#define VOLTS_PER_RESOLUTION  0.0029296875 // 48v over 14-bit resolution - 48 divided by 16384
#else
#define VOLTS_PER_RESOLUTION  0.046875  // 48v over 10-bit resolution - 48 divided by 1024
#endif

#define ANNEAL_POWER_INTERVAL 250

// Twist color numbers - for use with the Twist.setColor() call. Won't take a hex RGB code, unfortunately.
#define RED       255,0,0
#define GREEN     0,255,0
#define BLUE      0,0,255

// EEPROM addresses - int is 2 bytes, so make sure these are even numbers!
#define ANNEAL_ADDR   0
#define DELAY_ADDR    2
#define EE_FAILSAFE_ADDR  100
#define EE_FAILSAFE_VALUE 42

// Control constatns
#define CASE_DROP_DELAY       500 // milliseconds
#define ANNEAL_TIME_DEFAULT   10
#define DELAY_DEFAULT         50 // hundredths of seconds - for the timer formats
#define LCD_STARTUP_INTERVAL  1000 // milliseconds - let the screen fire up and come online before we hit it
#define LCD_UPDATE_INTERVAL   50 // milliseconds


/*
 * STATE MACHINES - enum detailing the possible states for the annealing state machine. We may 
 * set one up for our future "Mayan" mode, if we get there, too.
 */

enum AnnealState
{
  WAIT_BUTTON,
  WAIT_CASE,
  START_ANNEAL,
  ANNEAL_TIMER,
  DROP_CASE,
  DROP_CASE_TIMER,
  DELAY
} annealState;

const char *annealStateDesc[] = {
  "  Press Start",
  "Wait for Case",
  " Start Anneal",
  "    Annealing",
  "    Drop Case",
  "    Drop Case",
  "        Pause",
  "        ERROR"
};


/*
 * DISPLAYS - initialize using SparkFun's SerLCD library
 */

SerLCD lcd; // Initialize the library with default I2C address 0x72

/*
 * ENCODER - initialize using SparkFun's Twist library
 */

TWIST twist;

 
 /*
  * TIMERS - Chrono can set up a metronome (replaces old Metro library) to establish
  * a periodic time for accomplishing a task, based on milliseconds
  */
#define ANALOG_INTERVAL       1000
#define LCDSTARTUP_INTERVAL   1000

Chrono AnalogSensors; 
Chrono AnnealPowerSensors;
Chrono LCDTimer;
elapsedMillis Timer;  // using this vs Chrono, because the model is a littler easier for this function


/*
 * Bounce2 instances - debounce our switches! Only needed if we use mechanical switches
 */

Bounce startBounce = Bounce();
Bounce stopBounce = Bounce();


/*
 * ANALOG SENSOR VARIABLES
 */
float Therm1Avg = 0;
float Therm1Temp = 0;
float Therm1TempHigh = 0;  // track highest temp we saw
String Therm1Pretty;

float internalTemp = 0;
float internalTempHigh = 0;  // track highest temp we saw

float amps = 0;
float volts = 0;

/*
 * Control variables
 */
boolean twistPressed = false;
boolean twistMoved = false;
boolean startPressed = false;

boolean startOnOpto = false; // we'll use this once we have an optical case sensor

int annealSetPoint = 0;  // plan to store this value as hundredths of seconds, multiplied by 100
int storedSetPoint = 0;  // the value hanging out in EEPROM - need this for comparison later
int delaySetPoint = 50;  // same format - in this case, we start with a half second pause, just in case
int twistDiff = 0;

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

// float DCCurrentValue = (medianValue / 1024.0 * Vref - Vref / 2.0) / mVperAmp;  //Sensitivity:100mV/A, 0A @ Vcc/2

  if (reset) {
      amps = ( ( ( analogRead(CURRENT_PIN) / RESOLUTION_MAX * 2.0 ) - 1.0) / 100 );
      if ( amps < 0 ) amps = 0;
      volts = ( analogRead(VOLTAGE_PIN) * VOLTS_PER_RESOLUTION );
  }
  else {
    ampsCalc = ( ( ( analogRead(CURRENT_PIN) / RESOLUTION_MAX * 2.0 ) - 1.0) / 100 );
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
 * Thrm 00.0 IntT  00.0
 * State: xxxxxxxxxxxxx
 */
void updateLCD(boolean full) {

int quotient = 0;
int remainder = 0;
int timerCurrent = 0;

#define LCD_SETPOINT_LABEL  0,0
#define LCD_SETPOINT        4,0
#define LCD_TIMER_LABEL     9,0
#define LCD_TIMER           15,0
#define LCD_CURRENT_LABEL   0,1
#define LCD_CURRENT         4,1
#define LCD_VOLTAGE_LABEL   9,1
#define LCD_VOLTAGE         15,1
#define LCD_THERM1_LABEL    0,2
#define LCD_THERM1          5,2
#define LCD_INTERNAL_LABEL  9,2
#define LCD_INTERNAL        16,2
#define LCD_STATE_LABEL     0,3
#define LCD_STATE           7,3

#ifdef DEBUG_LCD
  Serial.println("DEBUG: updating the LCD");
#endif

  if (full) {
#ifdef DEBUG_LCD
    Serial.println("DEBUG: full LCD update");
#endif
    lcd.clear();
    lcd.setCursor(LCD_SETPOINT_LABEL);
    lcd.print("Set ");
    lcd.setCursor(LCD_TIMER_LABEL);
    lcd.print(" Time ");
    lcd.setCursor(LCD_CURRENT_LABEL);
    lcd.print("Amp ");
    lcd.setCursor(LCD_VOLTAGE_LABEL);
    lcd.print(" Volt ");
    lcd.setCursor(LCD_THERM1_LABEL);
    lcd.print("Thrm ");
    lcd.setCursor(LCD_INTERNAL_LABEL);
    lcd.print(" IntT ");
    lcd.setCursor(LCD_STATE_LABEL);
    lcd.print("State:");
  }

  // set point is in hundredths of seconds! if it's less than 999, we need a space
#ifdef DEBUG_LCD
    Serial.println("DEBUG: LCD: print set point");
#endif
  lcd.setCursor(LCD_SETPOINT);
  quotient = annealSetPoint / 100;
  remainder = annealSetPoint % 100;
  if (quotient < 10) lcd.print(" ");
  lcd.print(quotient);
  lcd.print(".");
  if (remainder < 10) lcd.print("0");
  lcd.print(remainder);

#ifdef DEBUG_LCD
    Serial.println("DEBUG: LCD: print timer");
#endif
  lcd.setCursor(LCD_TIMER);
  // if we're running a timer, do the math to print the right value, otherwise, a default
  if (annealState == ( START_ANNEAL || ANNEAL_TIMER || DROP_CASE || DROP_CASE_TIMER || DELAY ) ) {
    timerCurrent = Timer;
    quotient = timerCurrent / 1000;
    remainder = timerCurrent % 1000 / 10;
    if (quotient < 10) lcd.print(" ");
    lcd.print(quotient);
    lcd.print(".");
    if (remainder < 10) lcd.print("0");
    lcd.print(remainder);
  }
  else {
    lcd.print(" 0.00");
  }

#ifdef DEBUG_LCD
    Serial.println("DEBUG: LCD: print amps");
#endif
  lcd.setCursor(LCD_CURRENT);
  remainder = (int) (amps * 100);
  quotient = remainder / 100;
  remainder = remainder % 100;
  if (quotient < 10) lcd.print(" ");
  lcd.print(quotient);
  lcd.print(".");
  if (remainder < 10) lcd.print("0");
  lcd.print(remainder);

#ifdef DEBUG_LCD
    Serial.println("DEBUG: LCD: print volts");
#endif
  lcd.setCursor(LCD_VOLTAGE);
  remainder = (int) (volts * 100);
  quotient = remainder / 100;
  remainder = remainder % 100;
  if (quotient < 10) lcd.print(" ");
  lcd.print(quotient);
  lcd.print(".");
  if (remainder < 10) lcd.print("0");
  lcd.print(remainder);

#ifdef DEBUG_LCD
    Serial.println("DEBUG: LCD: print therm1");
#endif
  lcd.setCursor(LCD_THERM1);
  remainder = (int) (Therm1Temp * 10);
  quotient = remainder / 10;
  remainder = remainder % 10;
  if (quotient >= 100) {
    lcd.print(" ");
    lcd.print(quotient);
  }
  else {
    if (quotient < 10) lcd.print(" ");
    lcd.print(quotient);
    lcd.print(".");
    lcd.print(remainder); // should be less than 10
  }

#ifdef DEBUG_LCD
    Serial.println("DEBUG: LCD: print internal");
#endif
  lcd.setCursor(LCD_INTERNAL);
  remainder = (int) (internalTemp * 10);
  quotient = remainder / 10;
  remainder = remainder % 10;

  if (quotient >= 100) {
    lcd.print(" ");
    lcd.print(quotient);
  }
  else {
    if (quotient < 10) lcd.print(" ");
    lcd.print(quotient);
    lcd.print(".");
    lcd.print(remainder); // should be less than 10
  }

#ifdef DEBUG_LCD
    Serial.println("DEBUG: LCD: print state");
#endif
  lcd.setCursor(LCD_STATE);
  lcd.print(annealStateDesc[annealState]);

#ifdef DEBUG_LCD
  Serial.println("DEBUG: done updating LCD");
#endif
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

#ifdef DEBUG
  //Serial.begin(9600);
  Serial.begin(115200);
  while (!Serial) ;
#endif

  analogReadResolution(14); //Set ADC resolution to the highest value possible 

#if defined(_AP3_VARIANT_H_) && defined(DEBUG) // Artemis based platforms have 14-bit ADC
  Serial.println("DEBUG: ADC read resolution set to 14 bits");
#else
  Serial.println("DEBUG: ADC read resolution set to 10 bits");
#endif


  /*
   * XXXXXXX
   * 
   * Look into error handling for Wire and lcd initialization - we need to error this out somehow! 
   * See below for Twist initialization.
   * 
   */
  Wire.begin(); // fire up I2C
  lcd.begin(Wire); // initialize our display over I2C
  Wire.setClock(400000); // kick the I2C SCL to High Speed Mode of 400kHz
  // Wire.setClock(1000000);   // 1 MHz.  (faster) - this also works?!?!?!

  // XXXXXX - related to internal temp sensor problem - keeping in the code for now
  //    note: the vcc var is removed above! Need a new float for that. 
  // determine the actual vcc for use in internal temp calculations
  // vcc = (float) analogRead(ADC_INTERNAL_VCC_DIV3) * 6 / RESOLUTION_MAX;

#ifdef DEBUG
  Serial.print("DEBUG: vcc == ");
  //Serial.println(vcc);
#endif
  
  // Initial analog sensor baselines



  for (int i=0; i<3; i++) {
#ifdef DEBUG
    temp = analogRead(THERM1_PIN);
    Serial.print("DEBUG: THERM1_PIN read: "); Serial.println(temp);
    Therm1Avg = Therm1Avg + temp;

    temp = analogRead(ADC_INTERNAL_TEMP);
    Serial.print("DEBUG: ADC_INTERNAL_TEMP read: "); Serial.println(temp);
    internalTemp = internalTemp + temp ;
#else
    Therm1Avg = Therm1Avg + analogRead(THERM1_PIN);
    internalTemp = internalTemp + analogRead(ADC_INTERNAL_TEMP);
#endif
  }

  // Average over the three readings...
  Therm1Avg /= 3;
  internalTemp /= 3;
  
#ifdef DEBUG
  Serial.print("DEBUG: Therm1Avg before Steinhart ");
  Serial.println(Therm1Avg);
  Serial.print("DEBUG: interalTemp before math: ");
  Serial.println(internalTemp);
#endif

  Therm1Temp = calcSteinhart(Therm1Avg);

  // The internal temp thermistor delivers 3.8 mV per degree C, apparently. We also seem to have an offset involved - 242 on my
  // system. Waiting on SparkFun to respond about the example code they supply, because it appears to be way off.
  internalTemp = ( ( internalTemp * VREF / RESOLUTION_MAX) / 0.0038 - 242) * 1.8 + 32; // 3.8 mV per degree, apparently. 
  
  Therm1TempHigh = Therm1Temp;
  internalTempHigh = internalTemp;

#ifdef DEBUG
  Serial.print("DEBUG: Therm1Temp initial == ");
  Serial.println(Therm1Temp);
  Serial.print("DEBUG: internalTemp initial == ");
  Serial.println(internalTemp);
#endif

  // initialize amps and volts for the display
  checkPowerSensors(true);

  // double check that we can trust the EEPROM by looking for a previously
  // stored "failsafe" value at a given address. We're going to use 
  // storedSetPoint here so we don't have to initialize a different variable
  EEPROM.get(EE_FAILSAFE_ADDR, storedSetPoint);
  if (storedSetPoint == EE_FAILSAFE_VALUE) {
    
#ifdef DEBUG
    Serial.print("DEBUG: EEPROM Failsafe - found <"); Serial.print(storedSetPoint); Serial.println(">");
#endif

    EEPROM.get(ANNEAL_ADDR, storedSetPoint);
    EEPROM.get(DELAY_ADDR, delaySetPoint);
  }
  else { // don't trust the EEPROM!
    
#ifdef DEBUG
    Serial.print("DEBUG: EEPROM Failsafe failed - found <"); Serial.print(storedSetPoint); Serial.println(">");
#endif

    EEPROM.put(EE_FAILSAFE_ADDR, EE_FAILSAFE_VALUE);
    EEPROM.put(ANNEAL_ADDR, ANNEAL_TIME_DEFAULT);
    EEPROM.put(DELAY_ADDR, DELAY_DEFAULT);
    annealSetPoint = ANNEAL_TIME_DEFAULT;
    delaySetPoint = DELAY_DEFAULT;
  }
    
  // and reset defaults if it looks like our defaults got wiped, but the
  // the EEPROM failsafe survived
  if (storedSetPoint == 0) {
    annealSetPoint = ANNEAL_TIME_DEFAULT;
  }
  else {
    annealSetPoint = storedSetPoint;
  }
  if (delaySetPoint == 0) delaySetPoint = DELAY_DEFAULT;

#ifdef DEBUG
  Serial.print("DEBUG: EEPROM stored Anneal set point was: ");
  Serial.print(storedSetPoint / 100);
  Serial.print(".");
  Serial.println(storedSetPoint % 100);
  Serial.print("DEBUG: Starting Anneal set point: ");
  Serial.print(annealSetPoint / 100);
  Serial.print(".");
  Serial.println(annealSetPoint % 100);
  Serial.print("DEBUG: EEPROM stored Delay set point: ");
  Serial.print(delaySetPoint / 100);
  Serial.print(".");
  Serial.println(delaySetPoint % 100);
#endif


  // double check that we've been here for a second before we talk to the LCD
  if (! LCDTimer.hasPassed(LCD_STARTUP_INTERVAL) ) {
    delay(LCD_STARTUP_INTERVAL - LCDTimer.elapsed());
  } // clear to make first output to the LCD, now

  lcd.clear(); // blank the screen to start
  
  // initialize the Twist encoder, and die if it's not there. Set initial knob color
  // to GREEN, and zero out the count
  if (twist.begin(Wire) == false)
  {
    
#ifdef DEBUG
    Serial.println("DEBUG: Can't find the SparkFun Qwiic Twist encoder. Can't move forward from here!");
#endif

    lcd.home();
    lcd.print("No Twist!       ");
    lcd.setCursor(0,1);
    lcd.print("We're toast!    ");
    signalOurDeath();
  }

  // cruft cleanup - not doing this can cause the annealer to immediately fire up if a Click event was
  // somehow left unhandled during our last endeavor
  twist.clearInterrupts(); 
  
  twist.setColor(GREEN);
  twist.setCount(0);

  // attach the Bounce2 objects
  startBounce.attach(START_PIN, INPUT_PULLUP);
  stopBounce.attach(STOP_PIN, INPUT_PULLUP);

  // show something on the screen - we know setup is done at this point, too!
  updateLCD(true);
  LCDTimer.restart();

#ifdef DEBUG
  Serial.println("DEBUG: END OF SETUP!");
#endif

#ifdef DEBUG_VERBOSE
  float tempamps = 0;
  while (1) {
    // temp = analogRead(ADC_INTERNAL_TEMP);
    // Serial.print("DEBUG: ADC_INTERNAL_TEMP Read: "); Serial.println(temp);
    tempamps = analogRead(CURRENT_PIN);
    Serial.print("DEBUG: CURRENT_PIN read "); Serial.print(tempamps);
    tempamps = ( ( ( tempamps / RESOLUTION_MAX * 2.0 ) - 1.0) / 100 );
    Serial.print(" calculated amps "); Serial.println(tempamps);
    // checkPowerSensors(false);
    delay(2000);
  }
#endif

}


/**************************************************************************************************
 * the loop
 **************************************************************************************************/
void loop() {

#ifdef DEBUG_LOOPTIMING
  loopMillis = millis();
#endif

  ////////////////////////////////////////////////////////////////
  // Housekeeping - https://www.youtube.com/watch?v=ERd5Faz8_-E  /
  ////////////////////////////////////////////////////////////////
  
  // gather button statuses
  
  // XXXXXX - how should debounce be handled for the Twist? Do we wait until the button is released before we count it? (isClicked)
  // or do we act upon press and then ignore Clicked? Or both, or...??? For now, both.
  //
  // Twist press/click will act as either start or stop button, for now, depending on context. This will change later to 
  // a state completely based on menu context
  
  if ( twist.isClicked() ) {
  // if ( twist.isClicked() || twist.isPressed() ) {
    twistPressed = true;
#ifdef DEBUG
    Serial.println("DEBUG: Twist clicked");
#endif
  }


  startBounce.update(); 
  stopBounce.update();
  
  if ((startBounce.fell() || twistPressed) && (annealState == WAIT_BUTTON)) {
    startPressed = true;
    twistPressed = false;
 #ifdef DEBUG
    Serial.println("DEBUG: start button pressed");
 #endif
 #ifdef DEBUG_STATE
    stateChange = true;
#endif
  } 
  
  if ((stopBounce.fell() || twistPressed) && (annealState != WAIT_BUTTON)) {
    digitalWrite(INDUCTOR_PIN, LOW);
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(SOLENOID_PIN, LOW);
    annealState = WAIT_BUTTON;
    twist.setColor(GREEN);
    twistPressed = false;
#ifdef DEBUG
    Serial.println("DEBUG: stop button pressed - anneal cycle aborted");
#endif
#ifdef DEBUG_STATE
    stateChange = true;
#endif
  }

  // check the encoder

  twistMoved = twist.isMoved();
  if (twistMoved && annealState == WAIT_BUTTON) {
    
#ifdef DEBUG
    twistDiff = twist.getDiff();
    Serial.print("DEBUG: twist moved - diff is ");
    Serial.println(twistDiff);
    annealSetPoint += twistDiff;
    Serial.print("DEBUG: new annealSetPoint = ");
    Serial.println(annealSetPoint);
#else
    annealSetPoint += twist.getDiff();
#endif

    int twistCount = twist.getCount();
    if ((twistCount > 32000) || (twistCount < -32000)) {
#ifdef DEBUG
      Serial.print("DEBUG: Twist count resetting - current count ");
      Serial.println(twistCount);
#endif
      twist.setCount(0); // prevent over/underflow
    }
    
    twistMoved = false;
  }
  else if (twistMoved) {  // and we're somewhere else in the state machine
    twistDiff = twist.getDiff(); // clear the difference
    twistMoved = false;
  }

  
  // Check our normal analog sensors
  
  if (AnalogSensors.hasPassed(ANALOG_INTERVAL, true)) {   // Note - the boolean restarts the timer for us

    // we us this section to update current and voltage while we're not actively annealing
    if (annealState != (START_ANNEAL || ANNEAL_TIMER )) checkPowerSensors(false);
    
    Therm1Avg = ((1.0 - THERM_SMOOTH_RATIO) * Therm1Avg) + (THERM_SMOOTH_RATIO * analogRead(THERM1_PIN));
    Therm1Temp = calcSteinhart(Therm1Avg); 
    if (Therm1Temp > Therm1TempHigh) {
      Therm1TempHigh = Therm1Temp;
    }

    // this algorithm is incorrect, but it's what SparkFun provided. Calculate something else...
    // internalTemp = ((1.0 - INT_TEMP_SMOOTH_RATIO) * internalTemp) + (INT_TEMP_SMOOTH_RATIO * ( (analogRead(ADC_INTERNAL_TEMP) * vcc / RESOLUTION_MAX) / 0.0038 * 1.8 + 32.0));

    internalTemp = (((1.0 - INT_TEMP_SMOOTH_RATIO) * internalTemp) + (INT_TEMP_SMOOTH_RATIO * ( ( (analogRead(ADC_INTERNAL_TEMP) * VREF / RESOLUTION_MAX) / 0.0038 - 242 ) * 1.8 + 32 ) ) );
    if (internalTemp > internalTempHigh) {
      internalTempHigh = internalTemp;
    }

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
    // wait for the go button
    case WAIT_BUTTON:
#ifdef DEBUG_STATE
      if (stateChange) { Serial.println("DEBUG: STATE MACHINE: enter WAIT_BUTTON"); stateChange = false; }
#endif
      if (startPressed) {
        annealState = WAIT_CASE;
        twist.setColor(RED);
        startPressed = false;
#ifdef DEBUG_STATE
        stateChange = true;
#endif
      }
      break;
      
    // we'll wait for a case here, once we have an opto
    case WAIT_CASE:
      // if we updated the set point, update it in the EEPROM, too.
      // this will need to change after we implement various menu routines, etc
#ifdef DEBUG_STATE
      if (stateChange) { Serial.println("DEBUG: STATE MACHINE: enter WAIT_CASE"); stateChange = false; }
#endif
      if (storedSetPoint != annealSetPoint) {
        storedSetPoint = annealSetPoint;
        EEPROM.put(ANNEAL_ADDR, storedSetPoint);
      }
      if (startOnOpto) {
        // nuttin' honey
      }
      else { // if we're not messing w/ the opto sensor, just go to the next step
        annealState = START_ANNEAL;
        
#ifdef DEBUG_STATE
        stateChange = true;
#endif
      }
      break;

    // fire up the induction board and start the clock
    case START_ANNEAL:
#ifdef DEBUG_STATE
      if (stateChange) { Serial.println("DEBUG: STATE MACHINE: enter START_ANNEAL"); stateChange = false; }
#endif
      annealState = ANNEAL_TIMER;
      digitalWrite(INDUCTOR_PIN, HIGH);
      digitalWrite(LED_BUILTIN, HIGH);
      Timer = 0;
      AnnealPowerSensors.restart();

#ifdef DEBUG_STATE
      stateChange = true;
#endif
      break;

    // wait for the annealer set point to expire
    case ANNEAL_TIMER:
#ifdef DEBUG_STATE
      if (stateChange) { Serial.println("DEBUG: STATE MACHINE: enter ANNEAL_TIMER"); stateChange = false; }
#endif
      if (Timer >= (annealSetPoint * 10)) {  // if we're done...
        digitalWrite(INDUCTOR_PIN, LOW);
        digitalWrite(LED_BUILTIN, LOW);
        annealState = DROP_CASE;
        twist.setColor(BLUE);
        Timer = 0;
        
#ifdef DEBUG_STATE
        stateChange = true;
#endif
      }    
      if (AnnealPowerSensors.hasPassed(ANNEAL_POWER_INTERVAL)) checkPowerSensors(false);
      AnnealPowerSensors.restart();
      break;
      
    case DROP_CASE: 
#ifdef DEBUG_STATE
      if (stateChange) { Serial.println("DEBUG: STATE MACHINE: enter DROP_CASE"); stateChange = false; }
#endif
      digitalWrite(SOLENOID_PIN, HIGH);
      annealState = DROP_CASE_TIMER;

#ifdef DEBUG_STATE
      stateChange = true;
#endif
      break;
      
    case DROP_CASE_TIMER:
#ifdef DEBUG_STATE
      if (stateChange) { Serial.println("DEBUG: STATE MACHINE: enter DROP_CASE_TIMER"); stateChange = false; }
#endif
      if (Timer >= CASE_DROP_DELAY) {
        digitalWrite(SOLENOID_PIN, LOW);
        annealState = DELAY;
        Timer = 0;

#ifdef DEBUG_STATE
        stateChange = true;
#endif
        break;
      }
      break;
      
    case DELAY:
#ifdef DEBUG_STATE
      if (stateChange) { Serial.println("DEBUG: STATE MACHINE: enter DELAY"); stateChange = false; }
#endif
      if (Timer >= (delaySetPoint * 10)) {
        twist.setColor(RED);
        annealState = WAIT_CASE;

#ifdef DEBUG_STATE
        stateChange = true;
#endif

      }
      break;
      
 
  } // switch(StepNumber)


  ////////////////////////////////////////////////////////
  // Boob Tube
  ////////////////////////////////////////////////////////

  // try to avoid updating the LCD in the last 150ms of an anneal cycle to avoid overshooting
  // annealSetPoint is in hundredths of seconds, so we need to multiply by 10 to get millis
  if ( LCDTimer.hasPassed(LCD_UPDATE_INTERVAL) && ( (annealState != START_ANNEAL || ANNEAL_TIMER) || ( ( (annealSetPoint * 10) - Timer ) > 150) ) ) {
    updateLCD(false);
    LCDTimer.restart();
  }

#ifdef DEBUG_LOOPTIMING
  Serial.print("DEBUG: loop took ");
  Serial.println(millis() - loopMillis);
#endif
  
} // loop()
