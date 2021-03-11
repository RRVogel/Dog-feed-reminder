
#include <avr/sleep.h>
#include <Wire.h>
#include <RTClib.h>
#include <FastLED.h>
#define VBATPIN A6
#define LED_PIN 5
#define NUM_LEDS 1
#define alarmPin 3
#define tiltPin 2

CRGB leds[NUM_LEDS];
uint8_t piezoPin = 7;
float VBat;
long timer = 0;
long debounce = 100;
uint8_t Tilt;
bool tiltCheck = false;
bool fedFlag = false;
volatile bool tiltFlag = false;
static byte prevADCSRA;
RTC_DS3231 rtc;


void setup() {
  //Serial.begin(9600);
  Wire.begin();
  rtc.begin();
  //Uncomment to set time
  //if(rtc.lostPower()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  
  rtc.disable32K();
  rtc.writeSqwPinMode(DS3231_OFF);
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
  rtc.disableAlarm(2);
  //Set ISR pins
  pinMode(alarmPin, INPUT_PULLUP);
  pinMode(tiltPin, INPUT_PULLUP);

  FastLED.addLeds<WS2812, LED_PIN, RGB>(leds, NUM_LEDS);
  //Blink green to show working
  leds[0] = CRGB(0, 255, 0);
  FastLED.show();
  delay(500);
  leds[0] = CRGB(0, 0, 0);
  FastLED.show();
  delay(200);
  leds[0] = CRGB(0, 255, 0);
  FastLED.show();
  delay(500);
  leds[0] = CRGB(0, 0, 0);
  FastLED.show();
  DateTime now = rtc.now();
  
}
//=============
void GoToSleep() {
  // Disable the ADC (Analog to digital converter, pins A0 [14] to A5 [19])
  prevADCSRA = ADCSRA;
  ADCSRA = 0;
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();

  //pinMode(0, INPUT);
  //pinMode(1, OUTPUT);
  //for (int i = 4; i < 18; i++) {
  //pinMode(i, OUTPUT);
  //}
  // Turn of Brown Out Detection (low voltage)
  MCUCR = bit (BODS) | bit(BODSE);
  MCUCR = bit(BODS);
  attachInterrupt(digitalPinToInterrupt(alarmPin), alarmISR, FALLING);
  sleep_cpu();
}
//==============
void alarmISR() {
  //Serial.println("Alarm occured!");
  detachInterrupt(digitalPinToInterrupt(alarmPin));
  rtc.clearAlarm(1);
}
//=============
void tiltISR() {
  detachInterrupt(digitalPinToInterrupt(tiltPin));
  tiltFlag = true;
}

//=============
void MeasureVBat() {
  VBat = analogRead(VBATPIN);
  VBat *= 6.6;
  VBat /= 1024;
}
//==============
void loop() {
//Set debounce timer and check if tilt switch triggered
  timer = millis();
  if (tiltFlag) tiltCheck = true;
  else { //reset fedFlag, blink red to feed
    fedFlag = false;
    leds[0] = CRGB(255, 0, 0);
    FastLED.show();
    delay(1000);
    leds[0] = CRGB(0, 0, 0);
    FastLED.show();
    delay(500);
    leds[0] = CRGB(255, 0, 0);
    FastLED.show();
    delay(1000);
    leds[0] = CRGB(0, 0, 0);
    FastLED.show();
    delay(500);
  }
//Tilt debounce and display green for done
  while (tiltCheck) {
    Tilt = digitalRead(tiltPin);
    if (Tilt) timer = millis();
    if ((millis() - timer) > debounce) {
      leds[0] = CRGB(0, 255, 0);
      FastLED.show();
      delay(1000);
      leds[0] = CRGB(0, 0, 0);
      FastLED.show();
      delay(500);
      leds[0] = CRGB(0, 255, 0);
      FastLED.show();
      delay(1000);
//Measure battery voltage and beep & blink yellow if needs charging
      /*MeasureVBat();
      if (VBat < 3.3) {
        leds[0] = CRGB(255, 255, 0);
        FastLED.show();
        tone(piezoPin, 4000, 200);
        delay(500);
        leds[0] = CRGB(0, 0, 0);
        FastLED.show();
        tone(piezoPin, 4000, 200);
        delay(500);
        leds[0] = CRGB(255, 255, 0);
        FastLED.show();
        tone(piezoPin, 4000, 200);
        delay(1000);
        leds[0] = CRGB(0, 0, 0);
        FastLED.show();
      }
      */
      rtc.setAlarm1(rtc.now() + TimeSpan(30), DS3231_A1_Second);
      GoToSleep();
      ADCSRA = prevADCSRA;
      fedFlag = true;
      tiltCheck = false;
      tiltFlag = false;
    }


  }
  if (fedFlag) rtc.setAlarm1(rtc.now() + TimeSpan(1), DS3231_A1_Minute);
  else {
    attachInterrupt(digitalPinToInterrupt(tiltPin), tiltISR, LOW);
    rtc.setAlarm1(rtc.now() + TimeSpan(10), DS3231_A1_Second);
  }
  GoToSleep();
  ADCSRA = prevADCSRA;
}
