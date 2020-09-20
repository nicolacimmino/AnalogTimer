#include <Arduino.h>
#include <avr/sleep.h>
#include <RotaryEncoder.h>
#include <EEPROM.h>

#define PIN_R 3
#define PIN_Y 9
#define PIN_G 10
#define PIN_PWM 6
#define PIN_ENC_A 9
#define PIN_ENC_B 10
#define PIN_ENC_SW 8

uint8_t timeSeconds = 60;
uint8_t resetTimeSeconds = 60;

RotaryEncoder rotaryEncoder;

bool running = false;
bool autoRestart = true;

void flash(uint8_t times = 4)
{
  for (uint8_t ix = 0; ix < times; ix++)
  {
    digitalWrite(PIN_R, HIGH);
    delay(200);
    digitalWrite(PIN_R, LOW);
    delay(300);
  }
}

void displayValue(uint8_t pin, uint8_t value, uint8_t maxValue)
{
  static uint8_t lastLevel = 0;
  uint8_t pwmLevel = (250 * value) / maxValue;

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

  displayValue(PIN_PWM, measuredVcc / 100, 34);
}

void click()
{
  if (!running)
  {
    running = true;
    resetTimeSeconds = timeSeconds;
    EEPROM.write(1, resetTimeSeconds);
  }
}

void longPress()
{
  if (running)
  {
    running = false;
    if (autoRestart)
    {
      timeSeconds = resetTimeSeconds;
    }
    return;
  }

  autoRestart = !autoRestart;

  EEPROM.write(0, autoRestart ? 1 : 0);

  flash(autoRestart ? 1 : 2);
}

void rotation(bool cw, int position)
{
  if (running)
  {
    return;
  }

  if (cw && timeSeconds < 60)
  {
    timeSeconds++;
  }

  if (!cw && timeSeconds > 0)
  {
    timeSeconds--;
  }
}

void setup()
{
  Serial.begin(9600);
  pinMode(PIN_R, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);

  rotaryEncoder.begin(PIN_ENC_A, PIN_ENC_B, PIN_ENC_SW, ROTARY_ENCODER_DECODE_MODE_4X, ROTARY_ENCODER_MODE_LINEAR, 0, 360);
  rotaryEncoder.registerOnClickCallback(click);
  rotaryEncoder.registerOnLongPressCallback(longPress);
  rotaryEncoder.registerOnRotationCallback(rotation);

  autoRestart = (EEPROM.read(0) > 0);
  resetTimeSeconds = min(EEPROM.read(1), 60);
  timeSeconds = resetTimeSeconds;

  flash(autoRestart ? 1 : 2);

  delay(1000);
  digitalWrite(PIN_R, HIGH);

  displayBatteryLevel();
  delay(2000);
  digitalWrite(PIN_R, LOW);
}

void loop()
{
  static unsigned long lastTick = millis();

  rotaryEncoder.loop();

  if (running && millis() - lastTick > 1000)
  {
    if (timeSeconds > 0)
    {
      timeSeconds--;
    }
    else
    {
      flash();

      if (!autoRestart)
      {
        running = false;
      }
      else
      {
        timeSeconds = resetTimeSeconds;
      }
    }

    lastTick = millis();
    
  }

Serial.println(timeSeconds);
  displayValue(PIN_PWM, timeSeconds, 60);
  
}
