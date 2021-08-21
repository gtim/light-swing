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

void loop() {

  // Read accelerometer
  
  sensors_event_t a, g, temp; // acceleration, gyro, temperature
  mpu.getEvent(&a, &g, &temp);
  
  // Set LED color from accelerometer.
  // Map z-direction acceleration 0..10 m/s^2 to cyan..magenta
  
  uint8_t hue = map( (long)(a.acceleration.z*100), 0, 1000, 116, 224 ); // 116 -> 224 : cyan -> magenta
  hue = constrain( hue, 116, 224 );
  for ( uint8_t i = 0; i < Num_LEDs; i++ ) {
    leds[i] = CHSV(hue,255,255);
  }
  FastLED.show();


  delay(30);
}
