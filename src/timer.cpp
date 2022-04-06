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

#include "timer.h"

uint8_t mode;
int16_t timeSeconds;
bool running;
bool paused;

void timerSetup()
{
  mode = EEPROM.read(EEPROM_MODE) % MODES_COUNT;
  timeSeconds = getResetTime();
  running = false;
}

void timerLoop()
{
  static unsigned long lastTick = millis();

  if ((running && !paused) && millis() - lastTick > 1000)
  {
    if (timeSeconds > 0)
    {
      timeSeconds--;
    }
    else
    {
      timeUp();
      timeSeconds = getResetTime();

      if (!shouldResetAtEnd())
      {
        running = false;
        showMode();
      }
    }

    lastTick = millis();
  }

  displayValue(PIN_ANALOG_INDICATOR, timeSeconds, getMaxTime());

  if (running)
  {
    if (paused)
    {
      digitalWrite(mode_leds[mode], HIGH);
    }
    else
    {
      // Keep breathing! See Sean Voisen great post from which I grabbed the formula.
      // https://sean.voisen.org/blog/2011/10/breathing-led-with-arduino/
      float val = (exp(sin(millis() / 2000.0 * PI)) - 0.36787944) * 108.0;
      analogWrite(mode_leds[mode], val);
    }
  }
}

uint16_t getMaxTime()
{
  if (mode == MODE_EGG)
  {
    return MAX_TIME_EGG;
  }

  if (mode == MODE_COOK)
  {
    return MAX_TIME_COOK;
  }

  return MAX_TIME_TEETH;
}

uint16_t getIncrement()
{
  if (mode == MODE_EGG)
  {
    return INCREMENT_TIME_EGG;
  }

  if (mode == MODE_COOK)
  {
    return INCREMENT_TIME_COOK;
  }

  return INCREMENT_TIME_TEETH;
}

uint16_t getResetTime()
{
  uint16_t resetTimeSeconds;
  EEPROM.get(EEPROM_RESET_TABLE_BASE + (2 * mode), resetTimeSeconds);

  return min(resetTimeSeconds, getMaxTime());
}

bool shouldResetAtEnd()
{
  return mode == MODE_TEETH || mode == MODE_COOK;
}

bool interruptableDelay(uint32_t delayMs)
{
  unsigned long startTime = millis();
  while (millis() - startTime < delayMs)
  {
    if (digitalRead(PIN_ENC_SW) == LOW)
    {
      return false;
    }
  }

  return true;
}

void timerOnEncClick()
{
  if (!running)
  {
    running = true;
    EEPROM.put(EEPROM_RESET_TABLE_BASE + (2 * mode), timeSeconds);
    return;
  }

  paused = !paused;
}

void timerOnEncLongPress()
{
  if (!running)
  {
    mode = (mode + 1) % MODES_COUNT;
    EEPROM.write(EEPROM_MODE, mode);
  }
  else
  {
    running = false;
    paused = false;
  }

  timeSeconds = getResetTime();
  showMode();
}

void timerOnEncRotation(bool cw, int position)
{
  if (running)
  {
    return;
  }

  timeSeconds += ((cw ? 1 : -1) * (int16_t)getIncrement());

  timeSeconds = min(max(timeSeconds, 0), getMaxTime());
}
