
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

#ifndef __TIMER_H__
#define __TIMER_H__

#include <Arduino.h>
#include <EEPROM.h>
#include "hardware.h"
#include "indicators.h"

#define MODES_COUNT 3
#define MODE_TEETH 0
#define MODE_EGG 1
#define MODE_COOK 2

#define EEPROM_MODE 0
#define EEPROM_RESET_TABLE_BASE 1

#define MAX_TIME_EGG 60 * 20
#define MAX_TIME_COOK 60 * 20
#define INCREMENT_TIME_EGG 10
#define INCREMENT_TIME_COOK 10
#define MAX_TIME_TEETH 60
#define INCREMENT_TIME_TEETH 1
#define FLASHES_ON_END 4

extern uint8_t mode;
extern int16_t timeSeconds;
extern bool running;
extern bool paused;

void timerSetup();
void timerLoop();
uint16_t getMaxTime();
uint16_t getIncrement();
uint16_t getResetTime();
bool shouldResetAtEnd();
bool interruptableDelay(uint32_t delayMs);
void timerOnEncClick();
void timerOnEncLongPress();
void timerOnEncRotation(bool cw, int position);

#endif
