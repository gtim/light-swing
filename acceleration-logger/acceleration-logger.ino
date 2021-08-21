#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

/*
 * Log acceleration data from MPU-9150 to serial.
 * 
 * Components:
 *   - Arduino Uno
 *   - MPU9150
 *   - 220 uF cap
 * Circuit:
 *   - Arduino GND  <-> MPU-9150 GND
 *   - Arduino 3.3V <-> MPU-9150 VCC
 *   - Arduino SDA  <-> MPU-9150 SDA
 *   - Arduino SCL  <-> MPU-9150 SCL
 *   - Arduino 3.3V <-> 220 uF cap <-> Arduino GND
 */

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(9600);

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

  // Output in CSV format "time,acc_x,acc_y,acc_z"

  Serial.print(millis()); // ms
  Serial.print(",");
  Serial.print(a.acceleration.x); // m/s^2
  Serial.print(",");
  Serial.print(a.acceleration.y);
  Serial.print(",");
  Serial.print(a.acceleration.z);
  Serial.println("");
  
  delay(30);
  
}
