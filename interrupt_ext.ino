
#include "Wire.h"
#include <Arduino.h>

const uint8_t button_PIN = 23;


#define MPU_ADDR 0x68
#define FILTER_ORDER 3
uint32_t intervalMPU = 5000;
uint32_t last;
uint32_t last_button_time = 0;
uint32_t button_time;

int16_t AccX, AccY, AccZ;
int16_t GyroX, GyroY, GyroZ;
float NormAccX, NormAccY, NormAccZ;
float NormGyroX, NormGyroY, NormGyroZ;
float AccOffsetX, AccOffsetY, AccOffsetZ, GyroOffsetX, GyroOffsetY, GyroOffsetZ;

float refG = 9.8037f;
float rangeAcc = .0001220703125f;
float rangeGyro = .0152671755725191f;
int c = 0;
int flag_isr = 0;

void IRAM_ATTR isr() {
  if (millis() - last_button_time > 250) {
    flag_isr = 1;
    last_button_time = millis();
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(button_PIN, INPUT_PULLUP);
  attachInterrupt(button_PIN, isr, FALLING);

  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU_ADDR);  // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0);                     // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission

  Wire.beginTransmission(MPU_ADDR);  // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x1B);                  // Talk to register GYRO_CONFIG
  Wire.write(0x08);                  // Set gyroscope range +- 500 deg/s
  Wire.endTransmission(true);        //end the transmission

  Wire.beginTransmission(MPU_ADDR);  // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x1C);                  // Talk to register ACC_CONFIG
  Wire.write(0x08);                  // Set accelerometer range +- 4g
  Wire.endTransmission(true);        //end the transmission

  delay(20);
}

void loop() {
  if (flag_isr == 1) {
    calculate_IMU_error(&AccOffsetX, &AccOffsetY, &AccOffsetZ, &GyroOffsetX, &GyroOffsetY, &GyroOffsetZ);
    flag_isr = 0;
  }
  if (millis() - last >= intervalMPU) {
    // === Read acceleromter data === //
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);  // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);  // Read 6 registers total, each axis value is stored in 2 registers
    //For a range of +-4g, we need to divide the raw values by 8192, according to the datasheet
    AccX = ((Wire.read() << 8) | (Wire.read()));  // X-axis value
    AccY = ((Wire.read() << 8) | (Wire.read()));  // Y-axis value
    AccZ = ((Wire.read() << 8) | (Wire.read()));  // Z-axis value
    //Normalisasi Raw Data tersebut  Ref: 9.8036
    NormAccX = (AccX * rangeAcc) * 9.80665f - AccOffsetX;
    NormAccY = (AccY * rangeAcc) * 9.80665f - AccOffsetY;
    NormAccZ = (AccZ * rangeAcc) * 9.80665f - AccOffsetZ;

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);  // Gyro data first register address 0x43
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);  // Read 6 registers total, each axis value is stored in 2 registers
    // For a 500deg/s range we have to divide first the raw value by 65.5, according to the datasheet
    GyroX = ((Wire.read() << 8) | (Wire.read()));  // X-axis value
    GyroY = ((Wire.read() << 8) | (Wire.read()));  // Y-axis value
    GyroZ = ((Wire.read() << 8) | (Wire.read()));  // Z-axis value
    // Correct the outputs with the calculated error values
    NormGyroX = (GyroX * rangeGyro) - GyroOffsetX;  // GyroErrorX ~(-0.56)
    NormGyroY = (GyroY * rangeGyro) - GyroOffsetY;  // GyroErrorY ~(2)
    NormGyroZ = (GyroZ * rangeGyro) - GyroOffsetX;  // GyroErrorZ ~ (-0.8)
                                                    // ------PRINT TO serial---------
    Serial.print(NormAccX, 4);
    Serial.print(';');
    Serial.print(NormAccY, 4);
    Serial.print(';');
    Serial.print(NormAccZ, 4);
    Serial.print(';');
    Serial.print(NormGyroX, 4);
    Serial.print(';');
    Serial.print(NormGyroY, 4);
    Serial.print(';');
    Serial.print(NormGyroZ, 4);
    Serial.println(';');
    last += intervalMPU;
  }
}

void calculate_IMU_error(float* AccOffsetX, float* AccOffsetY, float* AccOffsetZ, float* GyroOffsetX, float* GyroOffsetY, float* GyroOffsetZ) {
  float AccErrorX = 0.0f;
  float AccErrorY = 0.0f;
  float AccErrorZ = 0.0f;
  float GyroErrorX = 0.0f;
  float GyroErrorY = 0.0f;
  float GyroErrorZ = 0.0f;
  int c = 0;
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 100) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
    AccX = ((Wire.read() << 8) | (Wire.read()));  // X-axis value
    AccY = ((Wire.read() << 8) | (Wire.read()));  // Y-axis value
    AccZ = ((Wire.read() << 8) | (Wire.read()));  // Z-axis value
    //Normalisasi Raw Data tersebut
    NormAccX = (AccX * rangeAcc) * 9.80665f;
    NormAccY = (AccY * rangeAcc) * 9.80665f;
    NormAccZ = (AccZ * rangeAcc) * 9.80665f;
    // Sum all readings
    AccErrorX = AccErrorX + NormAccX;
    AccErrorY = AccErrorY + NormAccY;
    AccErrorZ = AccErrorZ + NormAccZ;
    c++;
  }
  //Divide the sum by 100 to get the error value
  AccErrorX = AccErrorX / 100;
  AccErrorY = AccErrorY / 100;
  AccErrorZ = AccErrorZ / 100;
  // Reference G: 9.8037
  if (NormAccX > 9.0f) {
    *AccOffsetX = (AccErrorX - refG);
  } else if (NormAccY > 9.0f) {
    *AccOffsetY = (AccErrorY - refG);
  } else if (NormAccZ > 9.0f) {
    *AccOffsetZ = (AccErrorZ - refG);
  }
  c = 0;
  // Read gyro values 200 times
  while (c < 100) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    GyroX = ((Wire.read() << 8) | (Wire.read()));
    GyroY = ((Wire.read() << 8) | (Wire.read()));
    GyroZ = ((Wire.read() << 8) | (Wire.read()));
    // Sum all readings
    NormGyroX = GyroX * rangeGyro;
    NormGyroY = GyroY * rangeGyro;
    NormGyroZ = GyroZ * rangeGyro;
    GyroErrorX = GyroErrorX + NormGyroX;
    GyroErrorY = GyroErrorY + NormGyroY;
    GyroErrorZ = GyroErrorZ + NormGyroZ;
    c++;
  }
  //Divide the sum by 200 to get the error value
  if (NormAccX > 9.0f) {
    *GyroOffsetX = GyroErrorX / 100;
    *GyroOffsetY = GyroErrorY / 100;
    *GyroOffsetZ = GyroErrorZ / 100;
  }
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(*AccOffsetX, 4);
  Serial.print("AccErrorY: ");
  Serial.println(*AccOffsetY, 4);
  Serial.print("AccErrorZ: ");
  Serial.println(*AccOffsetZ, 4);
  Serial.print("GyroErrorX: ");
  Serial.println(*GyroOffsetX, 4);
  Serial.print("GyroErrorY: ");
  Serial.println(*GyroOffsetY, 4);
  Serial.print("GyroErrorZ: ");
  Serial.println(*GyroOffsetZ, 4);
}