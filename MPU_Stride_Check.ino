#include "Wire.h"
#include <Arduino.h>
#include <PeakDetection.h>
PeakDetection peakDetection;  // create PeakDetection object

#define MPU_ADDR 0x68
#define FILTER_ORDER 3

uint32_t intervalMPU = 10;
uint32_t last;
int16_t AccX, AccY, AccZ;
int16_t GyroX, GyroY, GyroZ;
float NormAccX, NormAccY, NormAccZ;
float NormGyroX, NormGyroY, NormGyroZ;
float AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;

float avg_AccX, avg_AccY, avg_AccZ, avg_GyroX, avg_GyroY, avg_GyroZ;
float max_AccX, max_AccY, max_AccZ, max_GyroX, max_GyroY, max_GyroZ;
float min_AccX, min_AccY, min_AccZ, min_GyroX, min_GyroY, min_GyroZ;
float stdev_AccX, stdev_AccY, stdev_AccZ, stdev_GyroX, stdev_GyroY, stdev_GyroZ;
float Array_AccX[51], Array_AccY[51], Array_AccZ[51], Array_GyroX[51], Array_GyroY[51], Array_GyroZ[51];

float StrideCheck1 = 500.0;
float StrideCheck2 = 500.0;
int DataCount, DataProcess, DataStart = 0;
int Start = 0;
int StrideChange = 2;

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
float FilteredAccX, FilteredAccY, FilteredAccZ, FilteredGyroX, FilteredGyroY, FilteredGyroZ;
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

float get_stdev(float data[51], float mean) {
  float hasil = 0;
  float stdev;
  for (int i = 0; i < 50; i++) {
    hasil = hasil + (sq(data[i] - mean));
  }
  hasil = hasil / 50;
  stdev = sqrt(hasil);
  return stdev;
}

void DEBUG_PRINT_MPU() {  // // ------PRINT TO serial---------
  // // Serial.print(NormAccX);
  // // Serial.print(';');
  Serial.print(FilteredAccX);
  Serial.print(';');
  // // Serial.print(NormAccY);
  // // Serial.print(';');
  Serial.print(FilteredAccY);
  Serial.print(';');
  // // Serial.print(NormAccZ);
  // // Serial.print(';');
  Serial.print(FilteredAccZ);
  Serial.print(';');
  // // Serial.print(NormGyroX);
  // // Serial.print(';');
  Serial.print(FilteredGyroX);
  Serial.print(';');
  // // Serial.print(NormGyroY);
  // // Serial.print(';');
  Serial.print(FilteredGyroY);
  Serial.print(';');
  // Serial.print(NormGyroZ);
  // Serial.print(';');
  Serial.print(FilteredGyroZ);
  Serial.println(';');
}
void DEBUG_PRINT() {
  Serial.print(avg_AccX);
  Serial.print(';');
  Serial.print(avg_AccY);
  Serial.print(';');
  Serial.print(avg_AccZ);
  Serial.print(';');
  Serial.print(avg_GyroX);
  Serial.print(';');
  Serial.print(avg_GyroY);
  Serial.print(';');
  Serial.print(avg_GyroZ);
  Serial.print(';');
  Serial.print(max_AccX);
  Serial.print(';');
  Serial.print(max_AccY);
  Serial.print(';');
  Serial.print(max_AccZ);
  Serial.print(';');
  Serial.print(max_GyroX);
  Serial.print(';');
  Serial.print(max_GyroY);
  Serial.print(';');
  Serial.print(max_GyroZ);
  Serial.print(';');
  Serial.print(min_AccX);
  Serial.print(';');
  Serial.print(min_AccY);
  Serial.print(';');
  Serial.print(min_AccZ);
  Serial.print(';');
  Serial.print(min_GyroX);
  Serial.print(';');
  Serial.print(min_GyroY);
  Serial.print(';');
  Serial.print(min_GyroZ);
  Serial.print(';');
  Serial.print(stdev_AccX);
  Serial.print(';');
  Serial.print(stdev_AccY);
  Serial.print(';');
  Serial.print(stdev_AccZ);
  Serial.print(';');
  Serial.print(stdev_GyroX);
  Serial.print(';');
  Serial.print(stdev_GyroY);
  Serial.print(';');
  Serial.print(stdev_GyroZ);
  Serial.println(';');
}
void RESET_DATA() {
  avg_AccX = 0;
  avg_AccY = 0;
  avg_AccZ = 0;
  avg_GyroX = 0;
  avg_GyroY = 0;
  avg_GyroZ = 0;
  max_GyroZ = 0;
  max_AccX = (-500.0);
  max_AccY = (-500.0);
  max_AccZ = (-500.0);
  max_GyroX = (-500.0);
  max_GyroY = (-500.0);
  max_GyroZ = (-500.0);
  min_AccX = 500.0;
  min_AccY = 500.0;
  min_AccZ = 500.0;
  min_GyroX = 500.0;
  min_GyroY = 500.0;
  min_GyroZ = 500.0;
  stdev_AccX = 0;
  stdev_AccY = 0;
  stdev_AccZ = 0;
  stdev_GyroX = 0;
  stdev_GyroY = 0;
  stdev_GyroZ = 0;
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
    //Normalisasi Raw Data tersebut  Ref: 9.73
    NormAccX = AccX * rangePerDigit * 9.80665f - 0.53;
    NormAccY = AccY * rangePerDigit * 9.80665f + 0.15;
    NormAccZ = AccZ * rangePerDigit * 9.80665f - 1.04;

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);  // Gyro data first register address 0x43
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);           // Read 4 registers total, each axis value is stored in 2 registers
    GyroX = ((Wire.read() << 8) | (Wire.read()));  // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
    GyroY = ((Wire.read() << 8) | (Wire.read()));
    GyroZ = ((Wire.read() << 8) | (Wire.read()));
    // Correct the outputs with the calculated error values
    NormGyroX = (GyroX / 131.0) + 0.97;  // GyroErrorX ~(-0.56)
    NormGyroY = (GyroY / 131.0) + 2.82;  // GyroErrorY ~(2)
    NormGyroZ = (GyroZ / 131.0) - 1.07;  // GyroErrorZ ~ (-0.8)

    FilteredAccX = FILTER(NormAccX, BUFFER_B_acc_x, BUFFER_A_acc_x, counter);
    FilteredAccY = FILTER(NormAccY, BUFFER_B_acc_y, BUFFER_A_acc_y, counter);
    FilteredAccZ = FILTER(NormAccZ, BUFFER_B_acc_z, BUFFER_A_acc_z, counter);
    FilteredGyroX = FILTER(NormGyroX, BUFFER_B_gyro_x, BUFFER_A_gyro_x, counter);
    FilteredGyroY = FILTER(NormGyroY, BUFFER_B_gyro_y, BUFFER_A_gyro_y, counter);
    FilteredGyroZ = FILTER(NormGyroZ, BUFFER_B_gyro_z, BUFFER_A_gyro_z, counter);

    // Stride check by detecting local minima
    if (Start == 0) {
      StrideCheck1 = min(StrideCheck1, FilteredGyroZ);
    }
    if ((StrideCheck1 != FilteredGyroZ) && (DataStart != 1)) {  // Menandakan grafik gyro-Z sudah naik kembali -- menandakan pembacaan sebelumnya merupakan local minima
      StrideChange = 1;                                         // StrideChange = 1 -> kaki kanan, StrideChange = 0 -> kaki kiri
      DataCount = 0;
      Start = 1;
    }

    if (StrideChange == 1) {
      DataStart = 1;
      //proses data
      avg_AccX = avg_AccX + FilteredAccX;
      avg_AccY = avg_AccY + FilteredAccY;
      avg_AccZ = avg_AccZ + FilteredAccZ;
      avg_GyroX = avg_GyroX + FilteredGyroX;
      avg_GyroY = avg_GyroY + FilteredGyroY;
      avg_GyroZ = avg_GyroZ + FilteredGyroZ;
      max_AccX = max(max_AccX, FilteredAccX);
      max_AccY = max(max_AccY, FilteredAccY);
      max_AccZ = max(max_AccZ, FilteredAccZ);
      max_GyroX = max(max_GyroX, FilteredGyroX);
      max_GyroY = max(max_GyroY, FilteredGyroY);
      max_GyroZ = max(max_GyroZ, FilteredGyroZ);
      min_AccX = min(min_AccX, FilteredAccX);
      min_AccY = min(min_AccY, FilteredAccY);
      min_AccZ = min(min_AccZ, FilteredAccZ);
      min_GyroX = min(min_GyroX, FilteredGyroX);
      min_GyroY = min(min_GyroY, FilteredGyroY);
      min_GyroZ = min(min_GyroZ, FilteredGyroZ);
      Array_AccX[DataCount] = FilteredAccX;
      Array_AccY[DataCount] = FilteredAccY;
      Array_AccZ[DataCount] = FilteredAccZ;
      Array_GyroX[DataCount] = FilteredGyroX;
      Array_GyroY[DataCount] = FilteredGyroY;
      Array_GyroZ[DataCount] = FilteredGyroZ;
      DataCount = DataCount + 1;
      if (DataCount > 30) {  // Atur hingga dirasa data sudah selalu turun terus
        StrideCheck2 = min(StrideCheck2, FilteredGyroZ);
        if ((StrideCheck2 != FilteredGyroZ)) {
          StrideCheck1 = 500.0;
          StrideChange = 0;
          DataProcess = 1;
        }
      }
    } else if (StrideChange == 0) {
      // pengambilan data interval sebelumnya selesai
      //proses data
      avg_AccX = avg_AccX + FilteredAccX;
      avg_AccY = avg_AccY + FilteredAccY;
      avg_AccZ = avg_AccZ + FilteredAccZ;
      avg_GyroX = avg_GyroX + FilteredGyroX;
      avg_GyroY = avg_GyroY + FilteredGyroY;
      avg_GyroZ = avg_GyroZ + FilteredGyroZ;
      max_AccX = max(max_AccX, FilteredAccX);
      max_AccY = max(max_AccY, FilteredAccY);
      max_AccZ = max(max_AccZ, FilteredAccZ);
      max_GyroX = max(max_GyroX, FilteredGyroX);
      max_GyroY = max(max_GyroY, FilteredGyroY);
      max_GyroZ = max(max_GyroZ, FilteredGyroZ);
      min_AccX = min(min_AccX, FilteredAccX);
      min_AccY = min(min_AccY, FilteredAccY);
      min_AccZ = min(min_AccZ, FilteredAccZ);
      min_GyroX = min(min_GyroX, FilteredGyroX);
      min_GyroY = min(min_GyroY, FilteredGyroY);
      min_GyroZ = min(min_GyroZ, FilteredGyroZ);
      Array_AccX[DataCount] = FilteredAccX;
      Array_AccY[DataCount] = FilteredAccY;
      Array_AccZ[DataCount] = FilteredAccZ;
      Array_GyroX[DataCount] = FilteredGyroX;
      Array_GyroY[DataCount] = FilteredGyroY;
      Array_GyroZ[DataCount] = FilteredGyroZ;
      DataCount = DataCount + 1;
      if (DataCount > 30) {  // Atur hingga dirasa data sudah selalu turun terus
        StrideCheck1 = min(StrideCheck1, FilteredGyroZ);
        if ((StrideCheck1 != FilteredGyroZ)) {
          StrideCheck2 = 500.0;
          StrideChange = 1;
          DataProcess = 1;
        }
      }
    }

    if (DataProcess == 1) {
      //Proses kirim data
      avg_AccX = avg_AccX / DataCount;
      avg_AccY = avg_AccY / DataCount;
      avg_AccZ = avg_AccZ / DataCount;
      avg_GyroX = avg_GyroX / DataCount;
      avg_GyroY = avg_GyroY / DataCount;
      avg_GyroZ = avg_GyroZ / DataCount;
      stdev_AccX = get_stdev(Array_AccX, avg_AccX);
      stdev_AccY = get_stdev(Array_AccY, avg_AccY);
      stdev_AccZ = get_stdev(Array_AccZ, avg_AccZ);
      stdev_GyroX = get_stdev(Array_GyroX, avg_GyroX);
      stdev_GyroY = get_stdev(Array_GyroY, avg_GyroY);
      stdev_GyroZ = get_stdev(Array_GyroZ, avg_GyroZ);
      // DEBUG_PRINT();
      // publishMessage();
      // client.loop();
      DataCount = 0;
      RESET_DATA();
      DataProcess = 0;
    }

    // peakDetection.add(FilteredGyroZ);
    // int peak = peakDetection.getPeak();
    // Serial.print(peak);
    // Serial.print(";");
    Serial.print(StrideChange);
    Serial.print(';');
    DEBUG_PRINT_MPU();
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
    NormGyroX = GyroX / 131.0;
    NormGyroY = GyroY / 131.0;
    NormGyroZ = GyroZ / 131.0;
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
