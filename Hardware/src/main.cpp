#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// I2C PIN CONFIGURATION
// I2C0: SDA=2, SCL=1
// I2C1: SDA=41, SCL=42

// MPU CONFIGURATION
Adafruit_MPU6050 mpu11; // I2C0, 0x68
Adafruit_MPU6050 mpu12; // I2C0, 0x69  
Adafruit_MPU6050 mpu21; // I2C1, 0x68
Adafruit_MPU6050 mpu22; // I2C1, 0x69

struct MPUData {
  float ax, ay, az;
  float gx, gy, gz;
};

MPUData mpu11_data = {0,0,0,0,0,0};
MPUData mpu12_data = {0,0,0,0,0,0};
MPUData mpu21_data = {0,0,0,0,0,0};
MPUData mpu22_data = {0,0,0,0,0,0};

bool mpu11_ok = false;
bool mpu12_ok = false;
bool mpu21_ok = false;
bool mpu22_ok = false;

// FLEX SENSOR CONFIGURATION
const int NUM_FLEX_SENSORS = 10;
const int FLEX_PINS[NUM_FLEX_SENSORS] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

// Flex sensor calibration storage
int flex_raw_min[NUM_FLEX_SENSORS] = {4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095}; // Straight finger (high resistance)
int flex_raw_max[NUM_FLEX_SENSORS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Bent finger (low resistance)
int flex_calibrated[NUM_FLEX_SENSORS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Calibration mode control
bool calibration_mode = false;
unsigned long calibration_start_time = 0;
const unsigned long CALIBRATION_DURATION = 10000; // 10 seconds calibration

void setupMPU(Adafruit_MPU6050* mpu) {
  mpu->setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu->setGyroRange(MPU6050_RANGE_500_DEG);
  mpu->setFilterBandwidth(MPU6050_BAND_94_HZ);
}

void readMPU(Adafruit_MPU6050* mpu, MPUData* data, bool* ok) {
  if (!*ok) {
    data->ax = data->ay = data->az = 0;
    data->gx = data->gy = data->gz = 0;
    return;
  }

  sensors_event_t a, g, temp;
  if (mpu->getEvent(&a, &g, &temp)) {
    data->ax = a.acceleration.x;
    data->ay = a.acceleration.y;
    data->az = a.acceleration.z;
    data->gx = g.gyro.x;
    data->gy = g.gyro.y;
    data->gz = g.gyro.z;
  } else {
    *ok = false; // Mark as failed if reading fails
  }
}

void readAllMPUs() {
  readMPU(&mpu11, &mpu11_data, &mpu11_ok);
  readMPU(&mpu12, &mpu12_data, &mpu12_ok);
  readMPU(&mpu21, &mpu21_data, &mpu21_ok);
  readMPU(&mpu22, &mpu22_data, &mpu22_ok);
}

void readFlexSensors() {
  for (int i = 0; i < NUM_FLEX_SENSORS; i++) {
    int raw_value = analogRead(FLEX_PINS[i]);
    
    if (calibration_mode) {
      // Update min/max during calibration
      if (raw_value < flex_raw_min[i]) flex_raw_min[i] = raw_value;
      if (raw_value > flex_raw_max[i]) flex_raw_max[i] = raw_value;
      flex_calibrated[i] = map(raw_value, flex_raw_min[i], flex_raw_max[i], 1000, 0);
    } else {
      // Normal operation with calibrated values
      raw_value = constrain(raw_value, flex_raw_min[i], flex_raw_max[i]);
      flex_calibrated[i] = map(raw_value, flex_raw_min[i], flex_raw_max[i], 1000, 0);
    }
  }
}

void printData() {
  // Print MPU data in order: MPU11, MPU12, MPU21, MPU22
  // MPU11: AX11,AY11,AZ11,GX11,GY11,GZ11
  Serial.print(mpu11_data.ax, 3); Serial.print(",");
  Serial.print(mpu11_data.ay, 3); Serial.print(",");
  Serial.print(mpu11_data.az, 3); Serial.print(",");
  Serial.print(mpu11_data.gx, 3); Serial.print(",");
  Serial.print(mpu11_data.gy, 3); Serial.print(",");
  Serial.print(mpu11_data.gz, 3); Serial.print(",");

  // MPU12: AX12,AY12,AZ12,GX12,GY12,GZ12
  Serial.print(mpu12_data.ax, 3); Serial.print(",");
  Serial.print(mpu12_data.ay, 3); Serial.print(",");
  Serial.print(mpu12_data.az, 3); Serial.print(",");
  Serial.print(mpu12_data.gx, 3); Serial.print(",");
  Serial.print(mpu12_data.gy, 3); Serial.print(",");
  Serial.print(mpu12_data.gz, 3); Serial.print(",");

  // MPU21: AX21,AY21,AZ21,GX21,GY21,GZ21
  Serial.print(mpu21_data.ax, 3); Serial.print(",");
  Serial.print(mpu21_data.ay, 3); Serial.print(",");
  Serial.print(mpu21_data.az, 3); Serial.print(",");
  Serial.print(mpu21_data.gx, 3); Serial.print(",");
  Serial.print(mpu21_data.gy, 3); Serial.print(",");
  Serial.print(mpu21_data.gz, 3); Serial.print(",");

  // MPU22: AX22,AY22,AZ22,GX22,GY22,GZ22
  Serial.print(mpu22_data.ax, 3); Serial.print(",");
  Serial.print(mpu22_data.ay, 3); Serial.print(",");
  Serial.print(mpu22_data.az, 3); Serial.print(",");
  Serial.print(mpu22_data.gx, 3); Serial.print(",");
  Serial.print(mpu22_data.gy, 3); Serial.print(",");
  Serial.print(mpu22_data.gz, 3); Serial.print(",");

  // Print flex sensors: F11,F12,F13,F14,F15,F21,F22,F23,F24,F25
  for (int i = 0; i < NUM_FLEX_SENSORS; i++) {
    Serial.print(flex_calibrated[i]);
    if (i < NUM_FLEX_SENSORS - 1) Serial.print(",");
  }

  Serial.println();
}

void startCalibration() {
  calibration_mode = true;
  calibration_start_time = millis();
  
  // Reset calibration values
  for (int i = 0; i < NUM_FLEX_SENSORS; i++) {
    flex_raw_min[i] = 4095;
    flex_raw_max[i] = 0;
  }
  
  Serial.println("\n=== CALIBRATION MODE ===");
  Serial.println("Move all fingers through full range for 10 seconds");
  Serial.println("Straight -> Fully bent -> Straight");
  Serial.println("Calibrating...");
}

void endCalibration() {
  calibration_mode = false;
  Serial.println("=== CALIBRATION COMPLETE ===");
  Serial.println("Min/Max values saved:");
  for (int i = 0; i < NUM_FLEX_SENSORS; i++) {
    Serial.print("F"); 
    if (i < 5) Serial.print("1"); else Serial.print("2");
    Serial.print((i % 5) + 1);
    Serial.print(": min="); Serial.print(flex_raw_min[i]);
    Serial.print(", max="); Serial.print(flex_raw_max[i]);
    Serial.println();
  }
  Serial.println("Returning to normal mode...");
}

void checkSerialCommands() {
  if (Serial.available()) {
    char command = Serial.read();
    
    if (command == 'c' || command == 'C') {
      startCalibration();
    } else if (command == 'n' || command == 'N') {
      if (calibration_mode) {
        endCalibration();
      }
    }
  }
}

void setup() {
  Serial.begin(921600);
  delay(1000);
  
  Serial.println("\n=== 4x MPU6050 + 10x Flex Sensor System ===");
  Serial.println("Initializing...");

  // Initialize I2C buses with your working pins
  Wire.begin(39, 40);    // I2C0: SDA=39, SCL=40
  Wire1.begin(41, 42); // I2C1: SDA=41, SCL=42

  // Initialize MPU6050 sensors
  Serial.println("\nMPU Initialization");
  
  // MPU11: I2C0, 0x68
  Serial.print("MPU11 (I2C0, 0x68)...");
  if (mpu11.begin(0x68, &Wire)) {
    setupMPU(&mpu11);
    mpu11_ok = true;
    Serial.println("OK");
  } else {
    Serial.println("FAILED");
  }

  // MPU12: I2C0, 0x69
  Serial.print("MPU12 (I2C0, 0x69)...");
  if (mpu12.begin(0x69, &Wire)) {
    setupMPU(&mpu12);
    mpu12_ok = true;
    Serial.println("OK");
  } else {
    Serial.println("FAILED");
  }

  // MPU21: I2C1, 0x68
  Serial.print("MPU21 (I2C1, 0x68)...");
  if (mpu21.begin(0x68, &Wire1)) {
    setupMPU(&mpu21);
    mpu21_ok = true;
    Serial.println("OK");
  } else {
    Serial.println("FAILED");
  }

  // MPU22: I2C1, 0x69
  Serial.print("MPU22 (I2C1, 0x69)...");
  if (mpu22.begin(0x69, &Wire1)) {
    setupMPU(&mpu22);
    mpu22_ok = true;
    Serial.println("OK");
  } else {
    Serial.println("FAILED");
  }

  // Initialize flex sensor pins
  Serial.println("\nFlex Sensor Setup");
  for (int i = 0; i < NUM_FLEX_SENSORS; i++) {
    pinMode(FLEX_PINS[i], INPUT);
    Serial.print("Flex F"); 
    if (i < 5) Serial.print("1"); else Serial.print("2");
    Serial.print((i % 5) + 1); 
    Serial.print(" (Pin "); 
    Serial.print(FLEX_PINS[i]); 
    Serial.println("): OK");
  }

  Serial.println("\nSYSTEM READY");
  Serial.println("Commands: 'c' - Start calibration, 'n' - Normal mode");
  Serial.println("Data Format: AX11,AY11,AZ11,GX11,GY11,GZ11,AX12,...,GZ22,F11,F12,...,F25");
  Serial.println("==========================================");
}

void loop() {
  checkSerialCommands();
  
  // Handle calibration mode timeout
  if (calibration_mode && (millis() - calibration_start_time >= CALIBRATION_DURATION)) {
    endCalibration();
  }

  // Read all sensors
  readAllMPUs();
  readFlexSensors();
  
  // Print data
  printData();
  
  // 100 Hz sampling rate
  delay(10);
}