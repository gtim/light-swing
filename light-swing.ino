// Embedded_EKF_Library
#include <Wire.h> // also for MPU
#include <elapsedMillis.h>
#include "src/ekf/swing_ekf.h"

// MPU
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Bluetooth 
#include <SoftwareSerial.h>

// FastLED
#include "SPI.h"
#include <FastLED.h>

// Pre-recorded sensor data for testing
#include "mock_sensor_data.h"
#define USE_MOCK_SENSOR_DATA



// MPU
Adafruit_MPU6050 mpu;
float measured_accel_abs, measured_gyro_abs;

// Bluetooth
int bluetoothTx = 0;
int bluetoothRx = 1;
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

// FastLED
const uint8_t Num_LEDs = 5;
const uint8_t LED_Data_Pin = 7;
CRGB leds[Num_LEDs];

// Lightshow state variables
bool angvel_positive = true; // track to find for zero-crossing = swing turns
uint32_t last_swingturn_ms = 0; // millis() of last swing turn = angvel zero-crossing
uint32_t last_swingturn_posneg_ms = 0; // angvel pos->neg (arbitrarily left/right)
uint32_t last_swingturn_negpos_ms = 0;

// EKF
SwingEKF swingEKF;
elapsedMillis timerEKF;
char bufferTxSer[100];


/*
 * Setup
 */

void setup() {
  
    // Init serial and Bluetooth
    
    Serial.begin(115200);
    delay(100);
    bluetooth.begin(115200);  // BlueSMiRF defaults to 115200bps

    // MPU/sensor

    if (!mpu.begin()) {
      Serial.println("Failed to find MPU6050 chip");
      while (1) {
        delay(10);
      }
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // FastLED init
    
    FastLED.setMaxPowerInVoltsAndMilliamps(5,500);
    FastLED.addLeds<WS2812, LED_Data_Pin>(leds, Num_LEDs);  
    for ( uint8_t i = 0; i < Num_LEDs; i++ )
      leds[i] = CRGB::Black;
    FastLED.show();

    // EKF init

    swingEKF.reset();

    delay(100);

}

/*
 * Main loop
 */

void loop() {
    if (timerEKF >= swingEKF.Step_size_s*1000) {
        timerEKF = 0;
        
        // Read sensor data

        #ifndef USE_MOCK_SENSOR_DATA

          // Use live sensor data
          
          sensors_event_t a, g, temp; // acceleration, gyro, temperature
          mpu.getEvent(&a, &g, &temp);
  
          // acceleration magnitude 
          measured_accel_abs = sqrt( a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z ); 
          // gyro x/y magnitude (not including z = twist)
          measured_gyro_abs  = sqrt( g.gyro.x * g.gyro.x + g.gyro.y * g.gyro.y );

        #else

          // Use pre-recorded mock data

          measured_accel_abs = mock_sensor_data[mock_sensor_data_row_i][0];
          measured_gyro_abs  = mock_sensor_data[mock_sensor_data_row_i][1];
          mock_sensor_data_row_i = ( mock_sensor_data_row_i + 1 ) % MOCK_SENSOR_DATA_NUM_ROWS;
          
        #endif

        
        // Log measurements to bluetooth to use as pre-recorded test case data

        //snprintf(bufferTxSer, sizeof(bufferTxSer)-1, "%.5f %.5f", measured_accel_abs, measured_gyro_abs );
        //bluetooth.print(bufferTxSer);
        //bluetooth.print('\n');
        
        
        // Update Kalman filter
        
        if (!swingEKF.kalmanUpdateStep(measured_accel_abs,measured_gyro_abs)) {
            swingEKF.reset();
            Serial.println("Whoop ");
        }

        // Update lightshow state

        // Angular velocity crosses zero = swing turns at end of arc
        if (          angvel_positive && swingEKF.getEstAngularVelocity() < 0 ) {
          last_swingturn_ms = millis();
          last_swingturn_posneg_ms = last_swingturn_ms;
          angvel_positive = false;
        } else if ( ! angvel_positive && swingEKF.getEstAngularVelocity() > 0 ) {
          last_swingturn_ms = millis();
          last_swingturn_negpos_ms = last_swingturn_ms;
          angvel_positive = true;
        }

        // Update LEDs

        uint32_t cur_millis = millis();
        if ( cur_millis - last_swingturn_ms < 1000 ) {
          //uint8_t hsv_hue   = map( cur_millis - last_swingturn_ms, 0, 1000, 224, 128 );
          //hsv_hue   = constrain( hsv_hue, 128, 224 );
          uint8_t hsv_hue = ( last_swingturn_negpos_ms > last_swingturn_posneg_ms ? 224 : 128 );
          uint8_t hsv_value = map( cur_millis - last_swingturn_ms, 0, 1000, 255, 0 );
          for ( uint8_t i = 0; i < Num_LEDs; i++ ) {
            leds[i] = CHSV(hsv_hue,255,hsv_value);
          }
        } else {
          for ( uint8_t i = 0; i < Num_LEDs; i++ ) {
            leds[i] = CRGB::Black;
          }
        }
        FastLED.show();
        
        // Print state to bluetooth/serial for live plotting
        
        snprintf(bufferTxSer, sizeof(bufferTxSer)-1, "%.3f %.3f %.3f %.3f %.3f",
                                                     swingEKF.getEstAngle() *180/3.1415, // x1 = estimated angle (deg)
                                                     swingEKF.getEstAngularVelocity() *180/3.1415, // x2 = estimated angular velocity (deg/s)
                                                     measured_accel_abs,   // y1 = measured acceleration magnitude
                                                     measured_gyro_abs*50, // y2 = measured gyro x/y magnitude (times 50)
                                                     (double)(millis()-last_swingturn_ms)/100 // time since last swing turn (ms/100)
        );
        Serial.print(bufferTxSer);
        Serial.print('\n');
        
    }
}
