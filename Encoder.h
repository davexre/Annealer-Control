/**************************************************************************************************
 * 
 * Annealer Control Program - AnnealLCD.cpp
 * Author: Dave Re
 * Inception: 05/20/2020
 * 
 * I wasn't totally happy with the available encoder libraries, so I implemented my own here,
 * based on the implementation described here: https://bildr.org/2012/08/rotary-encoder-arduino/
 * 
 * Includes a button, and ISRs, so we're not timer driven - and therefore, we don't pay to service
 * the encoder when it's not actually being moved.
 * 
 */
#ifndef _ENCODER_H
#define _ENCODER_H

#include "Arduino.h"

#define ENCODER_DEBOUNCE_MICROS 50000    // in micros, 50ms debounce time on encoder button
#define ENCODER_DOUBLECLICKTIME 500000   // in micros, second button press in 500ms == double click

const byte statusButtonDoubleClickedBit = 4;
const byte statusButtonClickedBit = 2;
const byte statusButtonPressedBit = 1;
const byte statusEncoderMovedBit = 0;

static volatile byte  statusRegister = 0;
static volatile int16_t   count = 0;
static volatile int16_t   diff = 0;

static uint8_t pinA = 0;
static uint8_t pinB = 0;
static int pinButton = -1;

static boolean buttonEnabled = false;

static volatile byte lastEncoded = 0;

static volatile unsigned long buttonLastCheck = 0;
static volatile unsigned long debounceMicros = 0;
static volatile int buttonLastState = LOW;


class Encoder 
{
  public:
    Encoder(uint8_t a, uint8_t b);
    Encoder(uint8_t a, uint8_t b, uint8_t btn);

    void encoderSetup(uint8_t a, uint8_t b, uint8_t btn);
    int16_t getCount();                                        // Returns the number of indents the user has turned the knob
    void setCount(int16_t amount);                             // Set the number of indents to a given amount
    int16_t getDiff(boolean clearValue = true);                // Returns the number of ticks since last check
    boolean isMoved();                                         // Returns true if knob has been twisted
    boolean isPressed();                                       // Return true if button is currently pressed.
    boolean isClicked();                                       // Return true if the button has been pressed and released
    boolean isDoubleClicked();                                 // Return true if the button has been double clicked
    void clear();                                              // Clear the status register

    void encoderInterrupt();
    void buttonInterrupt();

    static Encoder *thisencoder[1];

    static void encoderISR() { thisencoder[0]->encoderInterrupt(); }
    static void buttonISR() { thisencoder[0]->buttonInterrupt(); }

};



#endif
