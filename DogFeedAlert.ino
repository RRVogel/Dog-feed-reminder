// SPDX-FileCopyrightText: YYYY Rick Vogel for Adafruit Industries
//
// SPDX-License-Identifier: MIT

#include <avr/sleep.h>
#include <RTClib.h>
#include <FastLED.h>
#include <Wire.h>
#define LED_PIN 5
#define NUM_LEDS 1
#define alarmPin 3
#define tiltPin 2
#define VBATPIN A6

float VBat;
int Tilt;
bool fedFlag = true;
volatile bool tiltFlag;
uint8_t piezoPin = 11;
static byte prevADCSRA;
RTC_DS3231 rtc;
DateTime nowTime = rtc.now;
CRGB leds[NUM_LEDS];

void setup() {
  //Uncomment to set time
  //if(rtc.lostPower()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  rtc.begin();
  rtc.disableAlarm(1);
  rtc.disableAlarm(2);
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
  rtc.writeSqwPinMode(DS3231_OFF); // Place SQW pin into alarm interrupt mode
  rtc.disable32K();
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  pinMode(alarmPin, INPUT_PULLUP);
  pinMode(tiltPin, INPUT_PULLUP);

  //Blink Violet to show working
  leds[0] = CRGB(255, 0, 255);
  FastLED.show();
  delay(1000);
  leds[0] = CRGB(0, 0, 0);
  FastLED.show();
  delay(500);
  leds[0] = CRGB(255, 0, 255);
  FastLED.show();
  delay(1000);
  leds[0] = CRGB(0, 0, 0);
  FastLED.show();
  delay(500);
  leds[0] = CRGB(255, 0, 255);
  FastLED.show();
  delay(1000);
  leds[0] = CRGB(0, 0, 0);
  FastLED.show();
  delay(500);
}

void loop() {

  if (fedFlag) {
    BlinkGreen();
    rtc.disableAlarm(1);
    nowTime = rtc.now();

    if (nowTime.hour() >= 16) {
      rtc.setAlarm2(nowTime + TimeSpan(0, 11, 0, 0), DS3231_A2_Hour);
    }
    else {
     rtc.setAlarm2(nowTime + TimeSpan(0, 7, 0, 0), DS3231_A2_Hour); 
    }

    GoToSleep();
    detachInterrupt (digitalPinToInterrupt(alarmPin));
    fedFlag = false;
    //LowBat();
  }
  while (!fedFlag) {
    BlinkRed();

    nowTime = rtc.now();
    rtc.setAlarm1(nowTime + TimeSpan(0, 0, 0, 15), DS3231_A1_Second);

    attachInterrupt (digitalPinToInterrupt(tiltPin), tilt_ISR, LOW);
    GoToSleep();
    detachInterrupt (digitalPinToInterrupt(alarmPin));

    if (tiltFlag) {
      delay(50);
      for (int i = 0; i < 50; i++) {
        Tilt = digitalRead(tiltPin) + Tilt;
        delay(2);

      }
      if (Tilt > 40) {

        BlinkGreen();
        CheckVBat();
        if (VBat < 3.7) LowBat();

        nowTime = rtc.now();
        rtc.setAlarm1(nowTime + TimeSpan(0, 0, 0, 30), DS3231_A1_Second);
        GoToSleep();
        detachInterrupt (digitalPinToInterrupt(alarmPin));
        CheckVBat();
        if (VBat < 3.7) LowBat();
        BlinkGreen();
        tiltFlag = false;
        fedFlag = true;


      }
      else {
        tiltFlag = false;
        attachInterrupt (digitalPinToInterrupt(tiltPin), tilt_ISR, LOW);
      }

    }
  }

}
//==============
void alarm_ISR() {
  sleep_disable();
}
//=============
void tilt_ISR() {
  sleep_disable();
  detachInterrupt (digitalPinToInterrupt(tiltPin));
  tiltFlag = true;
}
//=============
void GoToSleep() {
  // Disable the ADC (Analog to digital converter, pins A0 [14] to A5 [19])
  prevADCSRA = ADCSRA;
  ADCSRA = 0;
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();

  // Turn of Brown Out Detection (low voltage)
  MCUCR = bit (BODS) | bit(BODSE);
  MCUCR = bit(BODS);

  attachInterrupt (digitalPinToInterrupt(alarmPin), alarm_ISR, FALLING);
  sleep_cpu();

  ADCSRA = prevADCSRA;
  detachInterrupt (digitalPinToInterrupt(alarmPin));
  rtc.disableAlarm(1);
  rtc.disableAlarm(2);
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
}
//=============
void LowBat() {
  leds[0] = CRGB(255, 255, 0);
  FastLED.show();
  tone(piezoPin, 4000, 200);
  delay(200);
  leds[0] = CRGB(0, 0, 0);
  FastLED.show();
  delay(300);
  leds[0] = CRGB(255, 255, 0);
  FastLED.show();
  tone(piezoPin, 4000, 200);
  delay(200);
  leds[0] = CRGB(0, 0, 0);
  FastLED.show();
  delay(300);
  leds[0] = CRGB(255, 255, 0);
  FastLED.show();
  tone(piezoPin, 4000, 200);
  delay(200);
  leds[0] = CRGB(0, 0, 0);
  FastLED.show();
}
//=============
void BlinkGreen() {
  leds[0] = CRGB(0, 255, 0);
  FastLED.show();
  delay(200);
  leds[0] = CRGB(0, 0, 0);
  FastLED.show();
  delay(300);
  leds[0] = CRGB(0, 255, 0);
  FastLED.show();
  delay(200);
  leds[0] = CRGB(0, 0, 0);
  FastLED.show();
  delay(300);
  leds[0] = CRGB(0, 255, 0);
  FastLED.show();
  delay(200);
  leds[0] = CRGB(0, 0, 0);
  FastLED.show();
}
//=============
void BlinkRed() {
  leds[0] = CRGB(255, 0, 0);
  FastLED.show();
  delay(100);
  leds[0] = CRGB(0, 0, 0);
  FastLED.show();
  delay(200);
  leds[0] = CRGB(255, 0, 0);
  FastLED.show();
  delay(100);
  leds[0] = CRGB(0, 0, 0);
  FastLED.show();
  delay(200);
  leds[0] = CRGB(255, 0, 0);
  FastLED.show();
  delay(100);
  leds[0] = CRGB(0, 0, 0);
  FastLED.show();
}
//Measure battery voltage
//=============
void CheckVBat() {
  VBat = analogRead(VBATPIN);
  VBat *= 6.6;
  VBat /= 1024;
}
