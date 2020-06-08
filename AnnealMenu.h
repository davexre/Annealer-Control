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
#include <plugin/userMenu.h>

using namespace Menu;


#define MAX_DEPTH 4

#ifdef DEBUG
  serialIn serial(Serial);
#endif

result targetEvent(eventMask, navNode&);

RencoderStream encoderStream(&encoder);

//characters allowed on name field
const char* constMEM alphaNum MEMMODE= "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789. ";
const char* constMEM alphaNumMask[] MEMMODE={alphaNum};

StoredCase target;

result enterAnneal(); // proto!

#ifdef DEBUG
  menuIn* inputsList[]={&encoderStream,&serial};
  chainStream<2> in(inputsList);
#else
  menuIn* inputsList[]={&encoderStream};
  chainStream<1> in(inputsList);
#endif

result saveTarget(eventMask e, navNode& nav) {
  navNode& nn=nav.root->path[nav.root->level-1];
  idx_t n=nn.sel;//get selection of previous level
  storedCases[n]=target;
  eepromStoreCase(n);
  return(quit);
}

result useTarget(eventMask e, navNode& nav) {
  idx_t n=nav.root->path[nav.root->level-1].sel;
  storedCases[n]=target;
  eepromStoreCase(n);
  annealSetPoint = target.time;
  return(quit);
}

result saveCurrentTimeTarget(eventMask e, navNode& nav) {
  idx_t n=nav.root->path[nav.root->level-1].sel;
  storedCases[n].time = annealSetPoint;
  eepromStoreCase(n);
  return(quit);
}

struct TargetMenu:UserMenu {
  using UserMenu::UserMenu;

  Used printItem(menuOut& out, int idx,int len) override {
    return len?out.printText(storedCases[idx].name,len):0;
  }
};

result idle(menuOut &o, idleEvent e) {
  return(proceed);
}

MENU(targetEdit, "Case Edit", doNothing, noEvent, wrapStyle,
  EDIT("Name", target.name, alphaNumMask, doNothing, noEvent, noStyle),
  FIELD(target.time, "Time", "", 0.0, 200.0, 0.1, 0.01, doNothing, noEvent, noStyle),
  OP("Use", useTarget, enterEvent),
  OP("Save", saveTarget, enterEvent),
  OP("Store Current", saveCurrentTimeTarget, enterEvent),
  EXIT("<< Back")
);

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

constMEM char targetsMenuTitle[] MEMMODE="Stored Cases";
constMEM menuNodeShadowRaw targetsMenuInfoRaw MEMMODE={
  (callback)targetEvent,
  (systemStyles)(_menuData|_canNav),
  targetsMenuTitle,
  enterEvent,
  wrapStyle,
  10, // number of cases
  NULL
};
constMEM menuNodeShadow& targetsMenuInfo=*(menuNodeShadow*)&targetsMenuInfoRaw;
TargetMenu targetsMenu(targetsMenuInfo,targetEdit,"<Back");

  
MENU(mainMenu,"Case Burner 5000",doNothing,noEvent,wrapStyle,
  OP("Anneal", enterAnneal, enterEvent),
  SUBMENU(annealerSettingsMenu),
  SUBMENU(dataDisplayMenu),
  OBJ(targetsMenu),
  EXIT("<< Back")
);

MENU_OUTPUTS(out,MAX_DEPTH
  ,LCD_OUT(lcd,{0,0,20,4})
  ,NONE
);

NAVROOT(nav,mainMenu,MAX_DEPTH,in,out);

result targetEvent(eventMask e, navNode& nav) {
  if(nav.target==&targetsMenu)//only if we are on targetsMenu
    target=storedCases[nav.sel];
  return(proceed);
}

// calling this function should cause us to exit/pause the menu system, and 
// fire up the annealer menu
result enterAnneal() { 
  menuState = ANNEALING;
  nav.idleOn();
  
  return(quit);
}
