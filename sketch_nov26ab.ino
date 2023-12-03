#include "Wire.h"
#include <Arduino.h>
#include <AES.h>
#include <base64.h>
#include <Base64.h>
#include <ArduinoJson.h>
#include "BluetoothSerial.h"

#define MPU_ADDR 0x68
DynamicJsonDocument doc(400);
DynamicJsonDocument doc_d(400);

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
int16_t AccX, AccY, AccZ;
int16_t GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
int roll, pitch, yaw;
float AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
float rangePerDigit = .00006103515625f;
float NormAccX, NormAccY, NormAccZ;
float NormGyroX, NormGyroY, NormGyroZ;
int c = 0;

char buffer[32];

void setup() {
  Serial.begin(9600);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU_ADDR);  // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0);                     // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  /*
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);
  */
  // Call this function if you need to get the IMU error values for your module
  // calculate_IMU_error();
  SerialBT.begin("ESP32test");  //Bluetooth device name
  Serial.println("Bluetooth telah dihidupkan!");

  delay(20);
}

String decryptField(String encryptedHexString, uint8_t *key, uint8_t *iv) {
  // Convert hexadecimal string to binary data
  int encryptedDataLen = encryptedHexString.length() / 2;
  byte encryptedData[encryptedDataLen];
  for (int i = 0; i < encryptedDataLen; i++) {
    char hexBuffer[3];
    encryptedHexString.substring(2 * i, 2 * i + 2).toCharArray(hexBuffer, 3);
    encryptedData[i] = strtol(hexBuffer, NULL, 16);
  }

  // Decryption
  AES aes;
  aes.set_key(key, 16);

  byte decryptedData[encryptedDataLen];
  aes.do_aes_decrypt(encryptedData, encryptedDataLen, decryptedData, key, 128, iv);

  // Convert binary data to string
  char decryptedCharArray[sizeof(float) + 1];
  memcpy(decryptedCharArray, decryptedData, sizeof(float));
  decryptedCharArray[sizeof(float)] = '\0';

  // Convert char array to String
  String decryptedString = String(decryptedCharArray);

  return decryptedString;
}

String encryptField(float input, uint8_t *key, uint8_t *iv) {
  // Padding
  int paddedInputLen = sizeof(float);
  char *paddedInput = (char *)malloc(paddedInputLen);

  if (!paddedInput) {
    Serial.println("Failed to allocate memory");
    return "";
  }

  memcpy(paddedInput, &input, sizeof(float));

  // Encryption
  AES aes;
  aes.set_key(key, 16);

  byte encryptedData[paddedInputLen];
  aes.do_aes_encrypt((byte *)paddedInput, paddedInputLen, encryptedData, key, 128, iv);

  // Convert binary data to hexadecimal string
  String encryptedHexString = "";
  for (int i = 0; i < paddedInputLen; i++) {
    char hexBuffer[3];
    sprintf(hexBuffer, "%02X", encryptedData[i]);
    encryptedHexString += hexBuffer;
  }

  // Clean up allocated memory
  free(paddedInput);

  return encryptedHexString;
}


void loop() {
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
  NormAccX = AccX * rangePerDigit * 9.80665f + 0.13;
  NormAccY = AccY * rangePerDigit * 9.80665f - 0.06;
  NormAccZ = AccZ * rangePerDigit * 9.80665f - 10.05;
  // // Calculating Roll and Pitch from the accelerometer data
  // accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  // accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
  // pitch = -(atan2(NormAccX, sqrt(NormAccY * NormAccY + NormAccZ * NormAccZ)) * 180.0) / M_PI + 3.85;  //error -3.85
  // roll = (atan2(NormAccY, NormAccZ) * 180.0) / M_PI - 0.75;                                           // error +0.75
  // === Read gyroscope data === //
  // previousTime = currentTime;                         // Previous time is stored before the actual time read
  // currentTime = millis();                             // Current time actual time read
  // elapsedTime = (currentTime - previousTime) / 1000;  // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);  // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);           // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = ((Wire.read() << 8) | (Wire.read()));  // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = ((Wire.read() << 8) | (Wire.read()));
  GyroZ = ((Wire.read() << 8) | (Wire.read()));
  // Correct the outputs with the calculated error values
  NormGyroX = (GyroX / 131.0) + 0.50;  // GyroErrorX ~(-0.56)
  NormGyroY = (GyroY / 131.0) + 4.18;  // GyroErrorY ~(2)
  NormGyroZ = (GyroZ / 131.0) - 0.56;  // GyroErrorZ ~ (-0.8)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  // gyroAngleX = gyroAngleX + GyroX * elapsedTime;  // deg/s * s = deg
  // gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  // yaw = yaw + GyroZ * elapsedTime;
  // yaw masih salah

  // Print the values on the serial monitor
  // Serial.print("Pitch : ");
  // Serial.print(pitch);
  // Serial.print(" Roll : ");
  // Serial.print(roll);
  // Serial.print(" Yaw: ");
  // Serial.println(yaw);
  //------PRINT TO SERIAL----------
  Serial.print(NormAccX);
  Serial.print(";");
  Serial.print(NormAccY);
  Serial.print(";");
  Serial.print(NormAccZ);
  Serial.print(";");
  Serial.print(NormGyroX);
  Serial.print(";");
  Serial.print(NormGyroY);
  Serial.print(";");
  Serial.print(NormGyroZ);
  Serial.println(";");
  //------PRINT TO json---------
  // Your AES encryption keys (should be kept secure)
  uint8_t key[16] = {0};  // Your encryption key (16 bytes)
  uint8_t iv[16] = {0};   // Your initialization vector (16 bytes)

  // encrypt_any_length_string(enc_input, (uint8_t *)enc_key, (uint8_t *)enc_iv);
  String encryptedData =  encryptField(NormGyroZ, key, iv);
  // doc["accX"] = encryptedData;
  // delay(100);
  // encryptedData = encryptField(NormAccY, key, iv);
  // doc["accY"] = encryptedData;
  //   delay(100);
  // encryptedData = encryptField(NormAccZ, key, iv);
  // doc["accZ"] = encryptedData;
  //   delay(100);
  // encryptedData = encryptField(NormGyroX, key, iv);
  // doc["gyroX"] = encryptedData;
  // delay(100);
  // encryptedData = encryptField(NormGyroY, key, iv);
  // doc["gyroY"] = encryptedData;
  // delay(100);
  // encryptedData = encryptField(NormGyroZ, key, iv);
  doc["gyroZ"] = encryptedData;
  String decryptedGyroZ = decryptField(encryptedData, key, iv);
  doc_d["gyroZ"] = decryptedGyroZ;
  serializeJson(doc, Serial);
  Serial.println();
  serializeJson(doc_d, Serial);
  Serial.println();
  //------PRINT TO bluetoooth---------
  // SerialBT.print(NormAccX);
  // SerialBT.print(';');
  // SerialBT.print(NormAccY);
  // SerialBT.print(';');
  // SerialBT.print(NormAccZ);
  // SerialBT.print(';');
  // SerialBT.print(NormGyroX);
  // SerialBT.print(';');
  // SerialBT.print(NormGyroY);
  // SerialBT.print(';');
  // SerialBT.print(NormGyroZ);
  // SerialBT.print(';');
  // SerialBT.print('\n');
  //delay(10);
  delay(5000);
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
