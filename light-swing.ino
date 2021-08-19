#include <FastLED.h>
#include "SPI.h"

const uint8_t Num_LEDs = 5;
const uint8_t LED_Data_Pin = 5;

CRGB leds[Num_LEDs];

void setup() {
  Serial.begin(9600);

  // FastLED set-up
  
  FastLED.setMaxPowerInVoltsAndMilliamps(5,500);
  FastLED.addLeds<WS2812, LED_Data_Pin>(leds, Num_LEDs);  
  for ( uint8_t i = 0; i < Num_LEDs; i++ )
    leds[i] = CRGB::Black;
  FastLED.show();

}

unsigned long loop_length_ms = 1e3;

void loop() {

  // LEDs loop through cyan to magenta

  unsigned long ms = millis();
  for ( uint8_t i = 0; i < Num_LEDs; i++ ) {
    uint8_t hue = map( ms % loop_length_ms, 0, loop_length_ms-1, 116, 224 ); // 116 -> 124 : cyan -> magenta
    leds[i] = CHSV(hue,255,255);
    ms += loop_length_ms/10;
  }
  FastLED.show();

}
