#include "Wire.h"
#include <Arduino.h>
#include <AESLib.h>
#include <base64.h>
#include <Base64.h>
#include <ArduinoJson.h>
// #include "BluetoothSerial.h"

AESLib aesLib;
#define MPU_ADDR 0x68
DynamicJsonDocument doc(1024);
DynamicJsonDocument doc_d(256);

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// BluetoothSerial SerialBT;
int ID = 01;
unsigned long getTime;
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

char buffer[100];
char ciphertext[30];
byte aes_key[] = { 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30 };
byte aes_iv[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

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
  // SerialBT.begin("ESP32test");  //Bluetooth device name
  Serial.println("Bluetooth telah dihidupkan!");
  aes_init();
  delay(20);
}

void aes_init() {
  aesLib.gen_iv(aes_iv);
  aesLib.set_paddingmode((paddingMode)2);  //ZeroLength
  // encrypt("AAAAAAAAAA", aes_iv); // workaround for incorrect B64 functionality on first run... initing b64 is not enough
}

String encrypt(char* msg, byte iv[]) {
  // unsigned long ms = micros();
  int msgLen = strlen(msg);
  char encrypted[2 * msgLen];
  aesLib.encrypt64((byte*)msg, msgLen, encrypted, aes_key, sizeof(aes_key), iv);
  // Serial.print("Encryption took: ");
  // Serial.print(micros() - ms);
  // Serial.println("us");
  return String(encrypted);
}

String decrypt(char* msg, byte iv[]) {
  // unsigned long ms = micros();
  int msgLen = strlen(msg);
  char decrypted[msgLen];  // half may be enough
  aesLib.decrypt64(msg, msgLen, (byte*)decrypted, aes_key, sizeof(aes_key), iv);
  // Serial.print("Decryption [2] took: ");
  // Serial.print(micros() - ms);
  // Serial.println("us");
  return String(decrypted);
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
  // Ambil waktu pengukuran
  getTime = millis();
  unsigned long seconds = getTime / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  seconds %= 60;
  minutes %= 60;
  String getTime_str = String(hours) + ":" + String(minutes) + ":" + String(seconds);

  //------PRINT TO SERIAL----------
  Serial.print(ID);
  Serial.print(";");
  Serial.print(getTime_str);
  Serial.print(";");
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
  // encrypt_any_length_string(enc_input, (uint8_t *)enc_key, (uint8_t *)enc_iv);
  byte enc_iv[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  // iv_block gets written to, reqires always fresh copy.
  byte dec_iv[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  // iv_block gets written to, requires always fresh copy.
  
  sprintf(buffer, "%d", ID);
  String encryptedData = encrypt(buffer, enc_iv);
  doc["ID"] = encryptedData;
  sprintf(ciphertext, "%s", encryptedData.c_str());
  String decryptedData = decrypt(ciphertext, dec_iv);
  doc_d["ID"] = decryptedData;

  sprintf(buffer, "%s", getTime_str);
  encryptedData = encrypt(buffer, enc_iv);
  doc["time"] = encryptedData;
  sprintf(ciphertext, "%s", encryptedData.c_str());
  decryptedData = decrypt(ciphertext, dec_iv);
  doc_d["time"] = decryptedData;

  sprintf(buffer, "%.2f", NormAccX);
  encryptedData = encrypt(buffer, enc_iv);
  doc["accX"] = encryptedData;
  sprintf(ciphertext, "%s", encryptedData.c_str());
  decryptedData = decrypt(ciphertext, dec_iv);
  doc_d["accX"] = decryptedData;

  sprintf(buffer, "%.2f", NormAccY);
  encryptedData = encrypt(buffer, enc_iv);
  doc["accY"] = encryptedData;
  sprintf(ciphertext, "%s", encryptedData.c_str());
  decryptedData = decrypt(ciphertext, dec_iv);
  doc_d["accY"] = decryptedData;
  
  sprintf(buffer, "%.2f", NormAccZ);
  encryptedData = encrypt(buffer, enc_iv);
  doc["accZ"] = encryptedData;
  sprintf(ciphertext, "%s", encryptedData.c_str());
  decryptedData = decrypt(ciphertext, dec_iv);
  doc_d["accZ"] = decryptedData;

  sprintf(buffer, "%.2f", NormGyroX);
  encryptedData = encrypt(buffer, enc_iv);
  doc["gyroX"] = encryptedData;
  sprintf(ciphertext, "%s", encryptedData.c_str());
  decryptedData = decrypt(ciphertext, dec_iv);
  doc_d["gyroX"] = decryptedData;

  sprintf(buffer, "%.2f", NormGyroY);
  encryptedData = encrypt(buffer, enc_iv);
  doc["gyroY"] = encryptedData;
  sprintf(ciphertext, "%s", encryptedData.c_str());
  decryptedData = decrypt(ciphertext, dec_iv);
  doc_d["gyroY"] = decryptedData;

  sprintf(buffer, "%.2f", NormGyroZ);
  encryptedData = encrypt(buffer, enc_iv);
  doc["gyroZ"] = encryptedData;
  sprintf(ciphertext, "%s", encryptedData.c_str());
  decryptedData = decrypt(ciphertext, dec_iv);
  doc_d["gyroZ"] = decryptedData;

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
