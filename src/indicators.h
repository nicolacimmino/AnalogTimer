
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

#ifndef __INDICATORS_H__
#define __INDICATORS_H__

#include <Arduino.h>

extern bool soundsOn;
extern uint8_t mode_leds[3];

void beep(uint16_t durationMillis);
void timeUp();

void indicatorsSetup();
void powerUpSequence();
void showMode();
void flash(uint8_t times = 8);
void displayValue(uint8_t pin, int16_t value, uint16_t maxValue);
void displayBatteryLevel();
void enableSounds();

#endif
