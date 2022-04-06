/********************************************************************************************
 *
 *   Copyright (C) 2022 Nicola Cimmino
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see http://www.gnu.org/licenses/.
 *
 *******************************************************************************************/

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
