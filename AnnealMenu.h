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
   
MENU(mainMenu,"Main Menu",doNothing,noEvent,wrapStyle,
  OP("Anneal", enterAnneal, enterEvent),
  SUBMENU(annealerSettingsMenu)
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
