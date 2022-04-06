#include <Arduino.h>
#include <avr/sleep.h>
#include <RotaryEncoder.h>

#include "hardware.h"
#include "timer.h"
#include "indicators.h"

RotaryEncoder rotaryEncoder;

void setup()
{
  rotaryEncoder.begin(PIN_ENC_A, PIN_ENC_B, PIN_ENC_SW, ROTARY_ENCODER_DECODE_MODE_4X, ROTARY_ENCODER_MODE_LINEAR, 0, 360);
  rotaryEncoder.registerOnClickCallback(timerOnEncClick);
  rotaryEncoder.registerOnLongPressCallback(timerOnEncLongPress);
  rotaryEncoder.registerOnRotationCallback(timerOnEncRotation);

  indicatorsSetup();

  timerSetup();

  powerUpSequence();

  displayBatteryLevel();

  showMode();
}

void loop()
{  
  rotaryEncoder.loop();

  timerLoop();
}
