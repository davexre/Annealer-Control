/**************************************************************************************************

   Annealer Control Program - AnnealLCD.cpp
   Author: Dave Re
   Inception: 05/20/2020

   I wasn't totally happy with the available encoder libraries, so I implemented my own here,
   based on the implementation described here: https://bildr.org/2012/08/rotary-encoder-arduino/

   There are a bunch of encoder libraries out there, so I didn't do an exhaustive search. I needed
   this to work in a way that it would be easily usable by both the main code and by the ArduinoMenu
   system.

   Note... only works with a single encoder at the moment

*/


#include "Annealer-Control.h"
#include "./Encoder.h"

Encoder *Encoder::thisencoder[1] = { NULL };

Encoder::Encoder(uint8_t a, uint8_t b) {
  Encoder::encoderSetup(a, b, -1);
}

Encoder::Encoder(uint8_t a, uint8_t b, uint8_t btn) {
  Encoder::encoderSetup(a, b, btn);
}

void Encoder::encoderSetup(uint8_t a, uint8_t b, uint8_t btn) {
  pinA = a;
  pinB = b;
  pinButton = btn;

  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  attachInterrupt(pinA, encoderISR, CHANGE);
  attachInterrupt(pinB, encoderISR, CHANGE);

  if (pinButton >= 0) {
    buttonEnabled = true;
    pinMode(pinButton, INPUT_PULLUP);
    attachInterrupt(pinButton, buttonISR, CHANGE);
  }

  thisencoder[0] = this;

}


int16_t Encoder::getCount() {
  return (count);
}
void Encoder::setCount(int16_t amount) {
  count = amount;
}

int16_t Encoder::getDiff(boolean clearValue) {
  int16_t diffReturn = diff;
  if (clearValue) diff = 0;
  return (diffReturn);
}

boolean Encoder::isMoved() {
  boolean moved = statusRegister & (1 << statusEncoderMovedBit);
  statusRegister &= ~(1 << statusEncoderMovedBit); // clear the bit after we've read it
  return (moved);
}

boolean Encoder::isPressed() {
  boolean pressed = statusRegister & (1 << statusButtonPressedBit);
  statusRegister &= ~(1 << statusButtonPressedBit); // clear the bit after we've read it
  return (pressed);
}

boolean Encoder::isClicked() {
  boolean clicked = statusRegister & (1 << statusButtonClickedBit);
  statusRegister &= ~(1 << statusButtonClickedBit); // clear the bit after we've read it
  return (clicked);
}

boolean Encoder::isDoubleClicked() {
  boolean doubleclicked = statusRegister & (1 << statusButtonDoubleClickedBit);
  statusRegister &= ~(1 << statusButtonDoubleClickedBit); // clear the bit after we've read it
  return (doubleclicked);
}

void Encoder::clear() {
  statusRegister = 0;
}

void Encoder::encoderInterrupt() {
  byte MSB = digitalRead(pinA);
  byte LSB = digitalRead(pinB);

  byte encoded = (MSB << 1) | LSB;
  lastEncoded = (lastEncoded << 2) | encoded;

  // many ways to do this. In this case, stealing from the Qwiic Twist and bildr code
  // to only look for two unique values. Trying to do this to discard invalid values,
  // and only act on complete encoder readings
  //
  // if this doesn't work, we'll try it with a look up table, like ClickEncoder uses

  if (lastEncoded == 0b01001011) //One indent clockwise
  {
    count++;
    diff++;
    statusRegister |= (1 << statusEncoderMovedBit);
  }
  else if (lastEncoded == 0b10000111) //One indent counter clockwise
  {
    count--;
    diff--;
    statusRegister |= (1 << statusEncoderMovedBit);
  }
}


void Encoder::buttonInterrupt() {
  // we're here because the button pin has changed
  // remember, we're using a pullup resistor, here, so LOW = pressed
  int buttonState = HIGH;
  unsigned long currentMicros;
  
  if ((long) (micros() - debounceMicros) >= ENCODER_DEBOUNCE_MICROS) {
    
    debounceMicros = micros();
    buttonState = digitalRead(pinButton);  
    
    // need to check our last button state (HIGH or LOW)
  
    if (buttonLastState == LOW && buttonState == HIGH) {
      // button is up, and was previously down
      // check for double click, too
      buttonLastState == HIGH;
      statusRegister |= (1 << statusButtonClickedBit);
      
      if (((unsigned long) micros() - buttonLastCheck) <= ENCODER_DOUBLECLICKTIME) {
        // looks like we have a double click
        statusRegister |= (1 << statusButtonDoubleClickedBit);
        buttonLastCheck = micros();
      }
      
    }
    else if (buttonLastState == HIGH && buttonState == LOW ) { 
      // button is down, and wasn't before
      
      buttonLastState == LOW;
      statusRegister |= (1 << statusButtonPressedBit);

    }
    
  } // debounce
  
}
