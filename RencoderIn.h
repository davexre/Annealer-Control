/*******************************************************************************
 *
 * RencoderIn.h
 * 
 * Based directly on ClickEncoderStream.h and SparkFun's Qwiic Twist library
 * 
 * May 2020
 * Created from ClickEncoderStream.h by Dave Re. Original header comments
 * follow. This creates a quick psuedo-Serial keyboard driver to navigate
 * ArduinoMenu menus using the Rencoder library.
 * 
 * Jun. 2016
 * Modified by Christophe Persoz and Rui Azevedo.
 * Based on keyStream.h developed by Rui Azevado.
 * and ClickEncoder library by Peter Dannegger.
 * https://github.com/christophepersoz/encoder
 * 
 * Sept. 2014 Rui Azevedo - ruihfazevedo(@rrob@)gmail.com
 * 
 * quick and dirty keyboard driver
 * metaprog keyboard driver where N is the number of keys
 * all keys are expected to be a pin (buttons)
 * we can have reverse logic (pull-ups) by entering negative pin numbers
 * ex: -A0 means: pin A0 normally high, low when button pushed (reverse logic)
 * 
 *******************************************************************************/


#ifndef __RencoderStream_h__
  #define __RencoderStream_h__

  #include <Arduino.h>
  #include <Rencoder.h>

  #ifndef ARDUINO_SAM_DUE
    // Arduino specific libraries
    // #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega328P__)
      // #include <stdint.h>
      //#include <avr/io.h>
      //#include <avr/interrupt.h>
    // #endif

    #include "menuDefs.h"

    namespace Menu {

      //emulate a stream based on builtin Encoder movement returning, +/- for every step
      //buffer not needer because we have an accumulator
    class RencoderStream:public menuIn {
      public:
        Encoder *encoder; 
        // int8_t sensivity;
        boolean encoderClicked = false;
        boolean encoderDoubleClicked = false;
        boolean clearEncoder = false;
        int pos;
        int oldPos;

        inline void update() {
          pos += encoder->getDiff();

          if (encoder->isClicked()) {
            encoderClicked = true;
            clearEncoder = true;
          }

          if (encoder->isDoubleClicked()) {
            encoderDoubleClicked = true;
            clearEncoder = true;
          }

          if (clearEncoder) encoder->clear();

        }

        RencoderStream(Encoder *e) {
          encoder = e;
          pos = encoder->getCount();
          oldPos = pos;
        }

        int available(void) {
            return peek() != -1;
        }

        int peek(void) {
          update();

          if (encoderDoubleClicked) {
            return options->navCodes[escCmd].ch;//menu::escCode;
          }
          
          if (encoderClicked) {
            return options->navCodes[enterCmd].ch;//menu::enterCode;
          }

          int d = pos - oldPos;
          if (d < 0)
              return options->navCodes[downCmd].ch;//menu::downCode;
          if (d > 0)
              return options->navCodes[upCmd].ch;//menu::upCode;
          return -1;
        }

        int read()
        {
            int ch = peek();
            if (ch == options->navCodes[upCmd].ch)          //menu::upCode
                oldPos += 1;
            else if (ch == options->navCodes[downCmd].ch)   //menu::downCode
                oldPos -= 1;
            else if (ch == options->navCodes[escCmd].ch) {  //menu::escCode;
                encoderDoubleClicked = false;
                encoderClicked = false;
            }
            else if (ch == options->navCodes[enterCmd].ch)  //menu::enterCode;
                encoderClicked = false;
                
            return ch;
        }

        void flush() {
            update();
            encoderClicked = false;
            encoderDoubleClicked = false;
            oldPos = pos;
        }

        size_t write(uint8_t v) {
            oldPos = v;
            return 1;
        }
      };
    }//namespace Menu

  #endif

#endif /* RencoderStream */
