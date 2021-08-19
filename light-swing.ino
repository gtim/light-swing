#include "SPI.h" // for FastLED
#include <Wire.h> // for MPU6050
#include <FastLED.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

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

  // MPU set-up

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);


  delay(100);
}

unsigned long loop_length_ms = 1e3;

void loop() {

  // MPU

  // Get new readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  // Print acceleration
  Serial.print("Acc ");
  Serial.print(a.acceleration.x);
  Serial.print(", ");
  Serial.print(a.acceleration.y);
  Serial.print(", ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");
  

  // LEDs loop through cyan to magenta

  unsigned long ms = millis();
  for ( uint8_t i = 0; i < Num_LEDs; i++ ) {
    uint8_t hue = map( ms % loop_length_ms, 0, loop_length_ms-1, 116, 224 ); // 116 -> 124 : cyan -> magenta
    leds[i] = CHSV(hue,255,255);
    ms += loop_length_ms/10;
  }
  FastLED.show();


  delay(30);
}
