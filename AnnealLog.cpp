/**************************************************************************************************
 * 
 * AnnealLog.cpp
 * Annealer Control Program
 * Author: Dave Re
 * Inception: 06/21/2020
 * 
 * This file contains functions that allow use of a SparkFun Qwiic OpenLog device to log data  
 * from various operations. Currently, this is only for Mayan runs, so we can examine the 
 * data later via graphs, spreadsheets, and all that fun.
 * 
 * Requires SparkFun's Qwiic OpenLog library, and the device.
 * 
 * Logs to a file named XXXXXXXX.CSV, where the Xs are an integer number. The Arduino has no 
 * sense of date/time (and it isn't needed for this project, anyway). So, we'll use an incrementing
 * integer to pick a filename to log to. Each analysis run may consist of one or more cases - we'll
 * log all of that data into the same file. When a run is finished, the file is closed and we
 * reset. When we start, we figure out the highest number CSV file on the disk, add one to the number
 * and use that as our log file name.
 *  * 
 **************************************************************************************************/

#include "Annealer-Control.h" // includes necessary libraries!

OpenLog annealLog;

void annealLogStartNewFile(void) {
  // set mayanUseSD to false if we fail in here
  byte status = annealLog.getStatus();
  int highestFileNum = 0;

  if (status == 0xFF) {
    // we're toast
    #ifdef DEBUG
    Serial.println(F("DEBUG: LOG: OpenLog device not available; mayanUseSD set to false"));
    #endif
    
    mayanUseSD = false;
    return;
  }
  // check to make sure we're cool and we're talking to an SD card ok

  if (! status & 1<<STATUS_SD_INIT_GOOD) {
    // we're still toast - OpenLog is working, but seemingly no SD card
    #ifdef DEBUG
    Serial.println(F("DEBUG: LOG: SD card appears to be uninitialized; mayanUseSD set to false"));
    #endif
    
    mayanUseSD = false;
    return;
  }

  // list all .CSV files and find the highest numbered one
  annealLog.searchDirectory("*.CSV");

  String fileName = annealLog.getNextDirectoryItem();
  while (fileName != "") {
    // strip the extension
    
    #ifdef DEBUG
    Serial.print(F("DEBUG: LOG: file in dir "));
    Serial.println(fileName);
    #endif
    
    fileName.remove(fileName.length() - 4); // ".CSV"

    #ifdef DEBUG
    Serial.print(F("DEBUG: LOG: file name minus extension "));
    Serial.println(fileName);
    #endif

    int filenum = fileName.toInt();

    if (filenum > highestFileNum) { 
      highestFileNum = filenum; 
    }
    
    fileName = annealLog.getNextDirectoryItem();
  }

  // increment that number by one and start our new file
  highestFileNum++;
  String newFileName = String(highestFileNum);
  newFileName.concat(F(".CSV"));

  #ifdef DEBUG
  Serial.print(F("DEBUG: LOG: opening file "));
  Serial.println(newFileName);
  #endif

  if (! annealLog.append(newFileName)) {
    mayanUseSD = false;
    #ifdef DEBUG
    Serial.println(F("DEBUG: LOG: append of new file name returned false; mayanUseSD set to false"));
    #endif
    
  }
  
}

void annealLogCloseFile(void) {
  annealLog.syncFile();
  // not sure there's anything else to do - we're dependent on the calling end
  // to decide when to make the new file, and OpenLog will continue to use the
  // same file until told to do otherwise
}

void annealLogWrite(String s) {
  annealLog.println(s);
  delay(15); // prevents us from overrunning the OpenLog's buffer
}
