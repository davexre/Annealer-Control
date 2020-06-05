/**************************************************************************************************
 * 
 * AnnealMenu.h
 * Annealer Control Program
 * Author: Dave Re
 * Inception: 05/25/2020
 * 
 * ArduinoMenu related functions and declarations. This is the main menu for the annealer.
 * 
 **************************************************************************************************/
#include <Wire.h>
#include <SerLCD.h>
#include <Rencoder.h>

#include "Annealer-Control.h"

#include <menu.h> //menu macros and objects
#include "SerLCDOut.h" //SparkFun SerLCD menu output
#include <menuIO/serialIn.h> //Serial input
#include "RencoderIn.h"
#include <menuIO/chainStream.h> // concatenate multiple input streams

using namespace Menu;


#define MAX_DEPTH 2

extern SerLCD lcd;

#ifdef DEBUG
  serialIn serial(Serial);
#endif

extern Encoder encoder;

RencoderStream encoderStream(&encoder);

result enterAnneal(); // proto!

#ifdef DEBUG
  menuIn* inputsList[]={&encoderStream,&serial};
  chainStream<2> in(inputsList);
#else
  menuIn* inputsList[]={&encoderStream};
  chainStream<1> in(inputsList);
#endif

result idle(menuOut &o, idleEvent e) {
  return(proceed);
}


MENU(annealerSettingsMenu, "Annealer Settings", doNothing, anyEvent, noStyle,
  FIELD(annealSetPoint, "Anneal Time", "sec", 0.0, 20.0, .10, 0.01, doNothing, noEvent, noStyle),
  FIELD(delaySetPoint, "Delay Time ", "sec", 0.0, 20.0, .10, 0.01, doNothing, noEvent, noStyle),
  FIELD(caseDropSetPoint, "Trapdoor   ", "sec", 0.5, 2.0, .10, 0.01, doNothing, noEvent, noStyle),
  EXIT("<< Back")
);

MENU(dataDisplayMenu, "Data Display", doNothing, anyEvent, noStyle,
  FIELD(Therm1TempHigh, "T1 High", " F", 0.0, 200.0, 0.1, 0.001, doNothing, noEvent, noStyle),
  #ifdef _AP3_VARIANT_H_
  FIELD(internalTempHigh, "Int High", " F", 0.0, 200.0, 0.1, 0.001, doNothing, noEvent, noStyle),
  #endif
  EXIT("<< Back")
);
   
MENU(mainMenu,"Case Burner 5000",doNothing,noEvent,wrapStyle,
  OP("Anneal", enterAnneal, enterEvent),
  SUBMENU(annealerSettingsMenu),
  SUBMENU(dataDisplayMenu)
);

MENU_OUTPUTS(out,MAX_DEPTH
  ,LCD_OUT(lcd,{0,0,20,4})
  ,NONE
);

NAVROOT(nav,mainMenu,MAX_DEPTH,in,out);

// calling this function should cause us to exit/pause the menu system, and 
// fire up the annealer menu
result enterAnneal() { 
  menuState = ANNEALING;
  nav.idleOn();
  
  return(quit);
}
