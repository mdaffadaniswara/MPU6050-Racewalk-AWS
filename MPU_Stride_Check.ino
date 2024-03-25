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
float Array_AccX[101], Array_AccY[101], Array_AccZ[101], Array_GyroX[101], Array_GyroY[101], Array_GyroZ[101];

float StrideCheck1 = 500.0;
float StrideCheck2 = 500.0;
int DataCount, DataProcess, DataStart = 0;
int Start = 0;
int StrideChange = 2;

float rangeAcc = .0001220703125f;
float rangeGyro = .0152671755725191f;
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

float get_stdev(float data[], float mean, int arraySize) {
  float hasil = 0;
  float stdev;
  for (int i = 0; i < (arraySize + 1); i++) {
    hasil = hasil + (sq(data[i] - mean));
  }
  hasil = hasil / (arraySize + 1);
  stdev = sqrt(hasil);
  return stdev;
}

void DATA_PROCESSING() {
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
  max_AccX = (-600.0);
  max_AccY = (-600.0);
  max_AccZ = (-600.0);
  max_GyroX = (-600.0);
  max_GyroY = (-600.0);
  max_GyroZ = (-600.0);
  min_AccX = 600.0;
  min_AccY = 600.0;
  min_AccZ = 600.0;
  min_GyroX = 600.0;
  min_GyroY = 600.0;
  min_GyroZ = 600.0;
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
  Wire.beginTransmission(MPU_ADDR);  // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x1B);                  // Talk to register GYRO_CONFIG
  Wire.write(0x08);                  // Set gyroscope range +- 500 deg/s
  Wire.endTransmission(true);        //end the transmission

  Wire.beginTransmission(MPU_ADDR);  // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x1C);                  // Talk to register ACC_CONFIG
  Wire.write(0x08);                  // Set accelerometer range +- 4g
  Wire.endTransmission(true);        //end the transmission
  // peakDetection.begin(36, 5, 0.6);   // sets the lag, threshold and influence
  //change the threshold
  delay(20);
  // calculate_IMU_error();
}

// void loop(){}
void loop() {
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
    //Normalisasi Raw Data tersebut  Ref: 9.73
    NormAccX = (AccX * rangeAcc) * 9.80665f - 0.54;
    NormAccY = (AccY * rangeAcc) * 9.80665f + 0.12;
    NormAccZ = (AccZ * rangeAcc) * 9.80665f - 1.03;

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);  // Gyro data first register address 0x43
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);  // Read 6 registers total, each axis value is stored in 2 registers
    // For a 500deg/s range we have to divide first the raw value by 65.5, according to the datasheet
    GyroX = ((Wire.read() << 8) | (Wire.read()));  // X-axis value
    GyroY = ((Wire.read() << 8) | (Wire.read()));  // Y-axis value
    GyroZ = ((Wire.read() << 8) | (Wire.read()));  // Z-axis value
    // Correct the outputs with the calculated error values
    NormGyroX = (GyroX * rangeGyro) + 1.01;  // GyroErrorX ~(-0.56)
    NormGyroY = (GyroY * rangeGyro) + 2.89;  // GyroErrorY ~(2)
    NormGyroZ = (GyroZ * rangeGyro) - 1.08;  // GyroErrorZ ~ (-0.8)

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
      StrideChange = 1;                                         // StrideChange = 1 -> kaki x, StrideChange = 0 -> kaki y
      DataCount = 0;
      Start = 1;
    }

    if (StrideChange == 1) {
      DataStart = 1;
      //proses data
      DATA_PROCESSING();
      DataCount = DataCount + 1;
      if (DataCount > 45) {  // Atur hingga dirasa data sudah selalu turun terus
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
      DATA_PROCESSING();
      DataCount = DataCount + 1;
      if (DataCount > 45) {  // Atur hingga dirasa data sudah selalu turun terus
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
      stdev_AccX = get_stdev(Array_AccX, avg_AccX, DataCount);
      stdev_AccY = get_stdev(Array_AccY, avg_AccY, DataCount);
      stdev_AccZ = get_stdev(Array_AccZ, avg_AccZ, DataCount);
      stdev_GyroX = get_stdev(Array_GyroX, avg_GyroX, DataCount);
      stdev_GyroY = get_stdev(Array_GyroY, avg_GyroY, DataCount);
      stdev_GyroZ = get_stdev(Array_GyroZ, avg_GyroZ, DataCount);
      DEBUG_PRINT();
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
