
#include "hardware.h"
#include "indicators.h"
#include "timer.h"

bool soundsOn = false;

uint8_t mode_leds[3] = {PIN_LED_G, PIN_LED_Y, PIN_LED_R};

void indicatorsSetup()
{
    for (uint8_t ix = 0; ix < MODES_COUNT; ix++)
  {
    pinMode(mode_leds[ix], OUTPUT);
    digitalWrite(mode_leds[ix], LOW);
  }

  pinMode(PIN_ANALOG_INDICATOR, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  if (digitalRead(PIN_ENC_SW) == LOW)
  {
    enableSounds();
  }

}

void beep(uint16_t durationMillis)
{
  unsigned long beepUntil = millis() + durationMillis;

  while (millis() < beepUntil)
  {
    if (soundsOn)
    {
      digitalWrite(PIN_BUZZER, HIGH);
      delayMicroseconds(500);
      digitalWrite(PIN_BUZZER, LOW);
      delayMicroseconds(500);
    }
  }
}

void timeUp()
{
  if (shouldResetAtEnd())
  {
    beep(100);
    delay(500);
    beep(100);
    delay(500);
    beep(100);

    for (uint8_t b = 0; b < 4; b++)
    {
      digitalWrite(mode_leds[mode], HIGH);
      delay(100);
      digitalWrite(mode_leds[mode], LOW);
      delay(200);
    }
    return;
  }

  for (uint8_t b = 0; b < 4; b++)
  {
    beep(b < 3 ? 200 : 1000);

    if (b < 3)
    {
      digitalWrite(mode_leds[mode], HIGH);
      delay(100);
      digitalWrite(mode_leds[mode], LOW);
      delay(300);
      digitalWrite(mode_leds[mode], HIGH);
      delay(100);
      digitalWrite(mode_leds[mode], LOW);
      delay(100);
    }
  }
}

void powerUpSequence()
{
  for (uint8_t ix = 0; ix < MODES_COUNT; ix++)
  {
    digitalWrite(mode_leds[ix], HIGH);
    delay(500);
  }

  delay(500);

  for (uint8_t ix = 0; ix < MODES_COUNT; ix++)
  {
    digitalWrite(mode_leds[ix], LOW);
  }
}

void showMode()
{
  for (uint8_t ix = 0; ix < MODES_COUNT; ix++)
  {
    digitalWrite(mode_leds[ix], ix == mode);
  }
}

void flash(uint8_t times = 8)
{
  while (times-- > 0)
  {
    digitalWrite(mode_leds[mode], HIGH);
    delay(100);
    digitalWrite(mode_leds[mode], LOW);
    delay(200);
  }
}

void displayValue(uint8_t pin, int16_t value, uint16_t maxValue)
{
  static int16_t lastLevel = 0;
  uint8_t pwmLevel = (250.0 * value) / maxValue;

  for (uint8_t level = lastLevel; level != pwmLevel; level += (pwmLevel < lastLevel) ? -1 : 1)
  {
    analogWrite(pin, level);
    delay(5);
  }

  lastLevel = pwmLevel;
}

void displayBatteryLevel()
{
  // See this article for an in-depth explanation.
  // https://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
  // tl;dr: we switch the ADC to measure the internal 1.1v reference using Vcc as reference, the rest is simple math.

  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);

  delay(2);
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC))
    ;

  long measuredVcc = 1125300L / (ADCL | (ADCH << 8));
  analogReference(DEFAULT);

  uint8_t batteryPercentage = min(max((measuredVcc - 2700) / 7, 0), 100);

  digitalWrite(PIN_LED_G, batteryPercentage > 70);
  digitalWrite(PIN_LED_Y, batteryPercentage > 40);
  digitalWrite(PIN_LED_R, HIGH);

  displayValue(PIN_ANALOG_INDICATOR, batteryPercentage, 100);

  interruptableDelay(2000);

  for (uint8_t ix = 0; ix < MODES_COUNT; ix++)
  {
    digitalWrite(mode_leds[ix], LOW);
  }
}

void enableSounds()
{
  soundsOn = true;
  beep(200);
  while (digitalRead(PIN_ENC_SW) == LOW)
  {
    delay(1);
  }
}