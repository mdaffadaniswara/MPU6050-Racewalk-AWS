#include "Wire.h"
#include <Arduino.h>
#include <PeakDetection.h>
PeakDetection peakDetection;                     // create PeakDetection object

#define MPU_ADDR 0x68
#define FILTER_ORDER 3

uint32_t intervalMPU = 10;
uint32_t last;
int16_t AccX, AccY, AccZ;
int16_t GyroX, GyroY, GyroZ;
float NormAccX, NormAccY, NormAccZ;
float NormGyroX, NormGyroY, NormGyroZ;
float AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;
float rangePerDigit = .00006103515625f;
int c = 0;
float FILTER_COEFF_A[FILTER_ORDER + 1] = { 1.0000, -0.5772, 0.4218, -0.0563 };
float FILTER_COEFF_B[FILTER_ORDER + 1] = { 0.0985, 0.2956, 0.2956, 0.0985 };
float BUFFER_A_acc_x[FILTER_ORDER + 1] = { 0.0 };
float BUFFER_B_acc_x[FILTER_ORDER + 1] = { 0.0 };
float BUFFER_A_acc_y[FILTER_ORDER + 1] = { 0.0 };
float BUFFER_B_acc_y[FILTER_ORDER + 1] = { 0.0 };
float BUFFER_A_acc_z[FILTER_ORDER + 1] = { 0.0 };
float BUFFER_B_acc_z[FILTER_ORDER + 1] = { 0.0 };
float BUFFER_A_gyro_x[FILTER_ORDER + 1] = { 0.0 };
float BUFFER_B_gyro_x[FILTER_ORDER + 1] = { 0.0 };
float BUFFER_A_gyro_y[FILTER_ORDER + 1] = { 0.0 };
float BUFFER_B_gyro_y[FILTER_ORDER + 1] = { 0.0 };
float BUFFER_A_gyro_z[FILTER_ORDER + 1] = { 0.0 };
float BUFFER_B_gyro_z[FILTER_ORDER + 1] = { 0.0 };
int counter = 0;

float FILTER(float input, float buffer_b[], float buffer_a[], int counter) {
  float FILTERED_DATA;

  buffer_b[counter] = input;
  FILTERED_DATA = 0;
  for (int i = 0; i <= FILTER_ORDER; i++) {
    FILTERED_DATA = FILTERED_DATA + buffer_b[(FILTER_ORDER + counter - i + 1) % (FILTER_ORDER + 1)] * FILTER_COEFF_B[i];
  }
  for (int i = 1; i <= FILTER_ORDER; i++) {
    FILTERED_DATA = FILTERED_DATA - buffer_a[(FILTER_ORDER + counter - i + 1) % (FILTER_ORDER + 1)] * FILTER_COEFF_A[i];
  }
  buffer_a[counter] = FILTERED_DATA;

  return FILTERED_DATA;
}

void setup() {
  Serial.begin(230400);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU_ADDR);  // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0);                     // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
                                     // Call this function if you need to get the IMU error values for your module
                                     // calculate_IMU_error();
                                     // SerialBT.begin("ESP32test");  //Bluetooth device name
                                     // for (int i = 0; i <= FILTER_ORDER; i++) {
                                     //   BUFFER_A[i] = 0;
                                     //   BUFFER_B[i] = 0;
                                     // }
  peakDetection.begin(36, 5, 0.6);   // sets the lag, threshold and influence
  //change the threshold
  delay(20);
  // calculate_IMU_error();
}

void loop() {
  if (millis() - last >= intervalMPU) {  // === Read acceleromter data === //

    // === Read acceleromter data === //
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);  // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);  // Read 6 registers total, each axis value is stored in 2 registers
    //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
    AccX = ((Wire.read() << 8) | (Wire.read()));  // X-axis value
    AccY = ((Wire.read() << 8) | (Wire.read()));  // Y-axis value
    AccZ = ((Wire.read() << 8) | (Wire.read()));  // Z-axis value
    //Normalisasi Raw Data tersebut
    NormAccX = AccX * rangePerDigit * 9.80665f - 0.04;
    NormAccY = AccY * rangePerDigit * 9.80665f + 0.10;
    NormAccZ = AccZ * rangePerDigit * 9.80665f - 0.367;

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);  // Gyro data first register address 0x43
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);           // Read 4 registers total, each axis value is stored in 2 registers
    GyroX = ((Wire.read() << 8) | (Wire.read()));  // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
    GyroY = ((Wire.read() << 8) | (Wire.read()));
    GyroZ = ((Wire.read() << 8) | (Wire.read()));
    // Correct the outputs with the calculated error values
    NormGyroX = (GyroX  / 131.0) + 0.15;  // GyroErrorX ~(-0.56)
    NormGyroY = (GyroY  / 131.0) + 4.26;  // GyroErrorY ~(2)
    NormGyroZ = (GyroZ / 131.0) - 0.66;  // GyroErrorZ ~ (-0.8)

    float FilteredAccX, FilteredAccY, FilteredAccZ, FilteredGyroX, FilteredGyroY, FilteredGyroZ;
    FilteredAccX = FILTER(NormAccX, BUFFER_B_acc_x, BUFFER_A_acc_x, counter);
    FilteredAccY = FILTER(NormAccY, BUFFER_B_acc_y, BUFFER_A_acc_y, counter);
    FilteredAccZ = FILTER(NormAccZ, BUFFER_B_acc_z, BUFFER_A_acc_z, counter);
    FilteredGyroX = FILTER(NormGyroX, BUFFER_B_gyro_x, BUFFER_A_gyro_x, counter);
    FilteredGyroY = FILTER(NormGyroY, BUFFER_B_gyro_y, BUFFER_A_gyro_y, counter);
    FilteredGyroZ = FILTER(NormGyroZ, BUFFER_B_gyro_z, BUFFER_A_gyro_z, counter);
    
    peakDetection.add(FilteredGyroZ);
    int peak = peakDetection.getPeak(); 
    Serial.print(peak);
    Serial.print(";");
    // // ------PRINT TO serial---------
    // // Serial.print(NormAccX);
    // // Serial.print(';');
    // Serial.print(FilteredAccX);
    // Serial.print(';');
    // // Serial.print(NormAccY);
    // // Serial.print(';');
    // Serial.print(FilteredAccY);
    // Serial.print(';');
    // // Serial.print(NormAccZ);
    // // Serial.print(';');
    // Serial.print(FilteredAccZ);
    // Serial.print(';');
    // // Serial.print(NormGyroX);
    // // Serial.print(';');
    // Serial.print(FilteredGyroX);
    // Serial.print(';');
    // // Serial.print(NormGyroY);
    // // Serial.print(';');
    // Serial.print(FilteredGyroY);
    // Serial.print(';');
    Serial.print(NormGyroZ);
    Serial.print(';');
    Serial.print(FilteredGyroZ);
    Serial.println(';');
    counter = counter + 1;
    if (counter > FILTER_ORDER) {
      counter = 0;
    }
    // counter = (counter + 1) % (FILTER_ORDER + 1);
    //   delay(10);
    last += intervalMPU;
  }
}

void calculate_IMU_error() {
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
    NormAccX = AccX * rangePerDigit * 9.80665f;
    NormAccY = AccY * rangePerDigit * 9.80665f;
    NormAccZ = AccZ * rangePerDigit * 9.80665f;
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
    NormGyroX = GyroX  / 131.0;
    NormGyroY = GyroY  / 131.0;
    NormGyroZ = GyroZ  / 131.0;
    GyroErrorX = GyroErrorX + NormGyroX;
    GyroErrorY = GyroErrorY + NormGyroY;
    GyroErrorZ = GyroErrorZ + NormGyroZ;
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 100;
  GyroErrorY = GyroErrorY / 100;
  GyroErrorZ = GyroErrorZ / 100;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("AccErrorZ: ");
  Serial.println(AccErrorZ);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}