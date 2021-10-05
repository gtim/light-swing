/*
 * Find MPU9150/MPU6050 gyroscope offsets for calibration
 */

#include <Wire.h> // also for MPU
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

constexpr uint16_t N = 100; // values to record
float gyro_output_sum[3];

void setup() {

    Serial.begin(115200);
    delay(100);

    if (!mpu.begin()) {
      Serial.println("Failed to find MPU6050 chip");
      while (1) {
        delay(10);
      }
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // Adjust to your application
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);      // Adjust to your application
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    delay(100);
}

void loop() {
  Serial.println("measuring..");
  gyro_output_sum[0] = 0;
  gyro_output_sum[1] = 0;
  gyro_output_sum[2] = 0;
  for ( uint16_t i = 0; i < N; i++ ) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyro_output_sum[0] += g.gyro.x;
    gyro_output_sum[1] += g.gyro.y;
    gyro_output_sum[2] += g.gyro.z;
    delay(100);
  }
  Serial.print("x offset: ");
  Serial.println( - gyro_output_sum[0] / N, 4 );
  Serial.print("y offset: ");
  Serial.println( - gyro_output_sum[1] / N, 4 );
  Serial.print("z offset: ");
  Serial.println( - gyro_output_sum[2] / N, 4 );
}
