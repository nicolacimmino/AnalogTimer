#include <Arduino.h>
#include <avr/sleep.h>
#include <RotaryEncoder.h>
#include <EEPROM.h>

#define PIN_R 3
#define PIN_Y 9
#define PIN_G 10
#define PIN_BUZZER A0
#define PIN_PWM 6
#define PIN_ENC_A 9
#define PIN_ENC_B 10
#define PIN_ENC_SW 8
#define MODE_TEETH 0
#define MODE_EGG 1
#define EEPROM_MODE 0
#define EEPROM_RESET_TABLE_BASE 1
#define MODES_COUNT 2
#define MAX_TIME_EGG 60 * 20
#define INCREMENT_TIME_EGG 10
#define MAX_TIME_TEETH 60
#define INCREMENT_TIME_TEETH 1
#define FLASHES_ON_END 4

uint8_t mode;
int16_t timeSeconds;
bool running;
bool paused = false;
bool soundsOn = false;

RotaryEncoder rotaryEncoder;

void flash(uint8_t times = 8)
{
  while (times-- > 0)
  {
    digitalWrite(PIN_R, HIGH);
    delay(100);
    digitalWrite(PIN_R, LOW);
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

  displayValue(PIN_PWM, batteryPercentage, 100);
}

uint16_t getMaxTime()
{
  if (mode == MODE_EGG)
  {
    return MAX_TIME_EGG;
  }

  return MAX_TIME_TEETH;
}

uint16_t getIncrement()
{
  if (mode == MODE_EGG)
  {
    return INCREMENT_TIME_EGG;
  }

  return INCREMENT_TIME_TEETH;
}

uint16_t getResetTime()
{
  uint16_t resetTimeSeconds;
  EEPROM.get(EEPROM_RESET_TABLE_BASE + (2 * mode), resetTimeSeconds);

  return min(resetTimeSeconds, getMaxTime());
}

void click()
{
  if (!running)
  {
    running = true;
    EEPROM.put(EEPROM_RESET_TABLE_BASE + (2 * mode), timeSeconds);
    return;
  }

  paused = !paused;
}

void longPress()
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
  flash(mode + 1);
}

void rotation(bool cw, int position)
{
  if (running)
  {
    return;
  }

  timeSeconds += ((cw ? 1 : -1) * (int16_t)getIncrement());

  timeSeconds = min(max(timeSeconds, 0), getMaxTime());
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
    beep(200);
    for (uint8_t b = 0; b < 4; b++)
    {
      digitalWrite(PIN_R, HIGH);
      delay(100);
      digitalWrite(PIN_R, LOW);
      delay(200);
    }
    return;
  }

  for (uint8_t b = 0; b < 4; b++)
  {
    beep(b < 3 ? 200 : 1000);

    if (b < 3)
    {
      digitalWrite(PIN_R, HIGH);
      delay(100);
      digitalWrite(PIN_R, LOW);
      delay(300);
      digitalWrite(PIN_R, HIGH);
      delay(100);
      digitalWrite(PIN_R, LOW);
      delay(100);
    }
  }
}

void setup()
{
  pinMode(PIN_R, OUTPUT);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  rotaryEncoder.begin(PIN_ENC_A, PIN_ENC_B, PIN_ENC_SW, ROTARY_ENCODER_DECODE_MODE_4X, ROTARY_ENCODER_MODE_LINEAR, 0, 360);
  rotaryEncoder.registerOnClickCallback(click);
  rotaryEncoder.registerOnLongPressCallback(longPress);
  rotaryEncoder.registerOnRotationCallback(rotation);

  if (digitalRead(PIN_ENC_SW) == LOW)
  {
    soundsOn = true;
    beep(200);
  }

  mode = EEPROM.read(EEPROM_MODE) % MODES_COUNT;
  timeSeconds = getResetTime();
  running = false;

  flash(mode + 1);

  delay(1000);
  digitalWrite(PIN_R, HIGH);
  displayBatteryLevel();
  delay(2000);
  digitalWrite(PIN_R, LOW);
}

bool shouldResetAtEnd()
{
  return mode == MODE_TEETH;
}

void loop()
{
  static unsigned long lastTick = millis();

  rotaryEncoder.loop();

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
        analogWrite(PIN_R, 0);
      }
    }

    lastTick = millis();
  }

  displayValue(PIN_PWM, timeSeconds, getMaxTime());

  if (running)
  {
    if (paused)
    {
      digitalWrite(PIN_R, HIGH);
    }
    else
    {
      // Keep breathing! See Sean Voisen great post from which I grabbed the formula.
      // https://sean.voisen.org/blog/2011/10/breathing-led-with-arduino/
      float val = (exp(sin(millis() / 2000.0 * PI)) - 0.36787944) * 108.0;
      analogWrite(PIN_R, val);
    }
  }
  else
  {
    digitalWrite(PIN_R, LOW);
  }
}
