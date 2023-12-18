#include "Wire.h"
#include <Arduino.h>
#include <AESLib.h>
#include <base64.h>
#include <Base64.h>
#include <ArduinoJson.h>
// #include "BluetoothSerial.h"

AESLib aesLib;
#define MPU_ADDR 0x68
DynamicJsonDocument doc(1536);
// DynamicJsonDocument doc_d(1024);

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
float avg_AccX, avg_AccY, avg_AccZ, avg_GyroX, avg_GyroY, avg_GyroZ;
float max_AccX, max_AccY, max_AccZ, max_GyroX, max_GyroY, max_GyroZ;
float min_AccX, min_AccY, min_AccZ, min_GyroX, min_GyroY, min_GyroZ;
float stdev_AccX, stdev_AccY, stdev_AccZ, stdev_GyroX, stdev_GyroY, stdev_GyroZ;
float Array_AccX[51], Array_AccY[51], Array_AccZ[51], Array_GyroX[51], Array_GyroY[51], Array_GyroZ[51];
float AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
float rangePerDigit = .00006103515625f;
float NormAccX, NormAccY, NormAccZ;
float NormGyroX, NormGyroY, NormGyroZ;
int c = 0;
int count = 0;

char buffer[100];
char ciphertext[30];
byte aes_key[] = { 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30 };
byte aes_iv[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

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

void setup() {
  Serial.begin(921600);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU_ADDR);  // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B --> Power ON
  Wire.write(0);                     // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  // calculate_IMU_error();
  aes_init();
  delay(20);
}

void aes_init() {
  aesLib.gen_iv(aes_iv);
  aesLib.set_paddingmode((paddingMode)2);  //Padding mode: ZeroLength
}

// Fungsi: melakukan enkripsi AES 128 CBC berdasarkan key dan inisialization vector (iv)
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
  // -----------------Read accelerometer data-----------------
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);  // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = ((Wire.read() << 8) | (Wire.read()));  // X-axis value
  AccY = ((Wire.read() << 8) | (Wire.read()));  // Y-axis value
  AccZ = ((Wire.read() << 8) | (Wire.read()));  // Z-axis value
  //Normalisasi Raw Data linear acceleration dan koreksi kalibrasi
  NormAccX = AccX * rangePerDigit * 9.80665f + 0.13;   // AccErrorX ~ -0.13
  NormAccY = AccY * rangePerDigit * 9.80665f - 0.06;   // AccErrorY ~ 0.06
  NormAccZ = AccZ * rangePerDigit * 9.80665f - 10.05;  // AccErrorZ ~ 10.05
  //didapatkan data linear acceleration dalam m/s^2

  // -----------------Read gyrometer data-----------------
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);  // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);           // Read 6 registers total, each axis value is stored in 2 registers
  GyroX = ((Wire.read() << 8) | (Wire.read()));  // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = ((Wire.read() << 8) | (Wire.read()));
  GyroZ = ((Wire.read() << 8) | (Wire.read()));
  //Normalisasi Raw Data angular rotation dan koreksi kalibrasi
  NormGyroX = (GyroX / 131.0) + 0.50;  // GyroErrorX ~(-0.56)
  NormGyroY = (GyroY / 131.0) + 4.18;  // GyroErrorY ~(2)
  NormGyroZ = (GyroZ / 131.0) - 0.56;  // GyroErrorZ ~ (-0.8)
  //didapatkan data angular rotation dalam deg/s

  // -----------------Get time pengukuran-----------------
  getTime = millis();
  unsigned long seconds = getTime / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  seconds %= 60;
  minutes %= 60;
  String getTime_str = String(hours) + ":" + String(minutes) + ":" + String(seconds);

  // -----------------Print to JSON format-----------------
  // AES encryption keys (should be kept secure)
  byte enc_iv[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  // iv_block gets written to, reqires always fresh copy.
  byte dec_iv[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  // iv_block gets written to, requires always fresh copy.
  previousTime = esp_timer_get_time();

  if (count == 50) {
    avg_AccX = avg_AccX / 50;
    avg_AccY = avg_AccY / 50;
    avg_AccZ = avg_AccZ / 50;
    avg_GyroX = avg_GyroX / 50;
    avg_GyroY = avg_GyroY / 50;
    avg_GyroZ = avg_GyroZ / 50;
    stdev_AccX = get_stdev(Array_AccX, avg_AccX);
    stdev_AccY = get_stdev(Array_AccY, avg_AccY);
    stdev_AccZ = get_stdev(Array_AccZ, avg_AccZ);
    stdev_GyroX = get_stdev(Array_GyroX, avg_GyroX);
    stdev_GyroY = get_stdev(Array_GyroY, avg_GyroY);
    stdev_GyroZ = get_stdev(Array_GyroZ, avg_GyroZ);
    count = 0;
    // ------Data nomor pelari------
    sprintf(buffer, "%d", ID);
    String encryptedData = encrypt(buffer, enc_iv);
    doc["ID"] = encryptedData;
    // sprintf(ciphertext, "%s", encryptedData.c_str());
    // String decryptedData = decrypt(ciphertext, dec_iv);
    // doc_d["ID"] = decryptedData;

    // ------Data waktu pengambilan data------
    sprintf(buffer, "%s", getTime_str);
    encryptedData = encrypt(buffer, enc_iv);
    doc["time"] = encryptedData;
    // sprintf(ciphertext, "%s", encryptedData.c_str());
    // decryptedData = decrypt(ciphertext, dec_iv);
    // doc_d["time"] = decryptedData;

    // ------Data acceleration X------
    sprintf(buffer, "%.2f", avg_AccX);
    encryptedData = encrypt(buffer, enc_iv);
    doc["avg_AccX"] = encryptedData;
    // sprintf(ciphertext, "%s", encryptedData.c_str());
    // decryptedData = decrypt(ciphertext, dec_iv);
    // doc_d["accX"] = decryptedData;

    // ------Data acceleration Y-----
    sprintf(buffer, "%.2f", avg_AccY);
    encryptedData = encrypt(buffer, enc_iv);
    doc["avg_AccY"] = encryptedData;
    // sprintf(ciphertext, "%s", encryptedData.c_str());
    // decryptedData = decrypt(ciphertext, dec_iv);
    // doc_d["accY"] = decryptedData;

    // ------Data acceleration Z------
    sprintf(buffer, "%.2f", avg_AccZ);
    encryptedData = encrypt(buffer, enc_iv);
    doc["avg_AccZ"] = encryptedData;
    // sprintf(ciphertext, "%s", encryptedData.c_str());
    // decryptedData = decrypt(ciphertext, dec_iv);
    // doc_d["accZ"] = decryptedData;

    // ------Data angular rotation X------
    sprintf(buffer, "%.2f", avg_GyroX);
    encryptedData = encrypt(buffer, enc_iv);
    doc["avg_GyroX"] = encryptedData;
    // sprintf(ciphertext, "%s", encryptedData.c_str());
    // decryptedData = decrypt(ciphertext, dec_iv);
    // doc_d["gyroX"] = decryptedData;

    // ------Data angular rotation Y------
    sprintf(buffer, "%.2f", avg_GyroY);
    encryptedData = encrypt(buffer, enc_iv);
    doc["avg_GyroY"] = encryptedData;
    // sprintf(ciphertext, "%s", encryptedData.c_str());
    // decryptedData = decrypt(ciphertext, dec_iv);
    // doc_d["gyroY"] = decryptedData;

    // ------Data angular rotation Z------
    sprintf(buffer, "%.2f", avg_GyroZ);
    encryptedData = encrypt(buffer, enc_iv);
    doc["avg_GyroZ"] = encryptedData;


    // ------Data acceleration X------
    sprintf(buffer, "%.2f", max_AccX);
    encryptedData = encrypt(buffer, enc_iv);
    doc["max_AccX"] = encryptedData;
    // sprintf(ciphertext, "%s", encryptedData.c_str());
    // decryptedData = decrypt(ciphertext, dec_iv);
    // doc_d["accX"] = decryptedData;

    // ------Data acceleration Y-----
    sprintf(buffer, "%.2f", max_AccY);
    encryptedData = encrypt(buffer, enc_iv);
    doc["max_AccY"] = encryptedData;
    // sprintf(ciphertext, "%s", encryptedData.c_str());
    // decryptedData = decrypt(ciphertext, dec_iv);
    // doc_d["accY"] = decryptedData;

    // ------Data acceleration Z------
    sprintf(buffer, "%.2f", max_AccZ);
    encryptedData = encrypt(buffer, enc_iv);
    doc["max_AccZ"] = encryptedData;
    // sprintf(ciphertext, "%s", encryptedData.c_str());
    // decryptedData = decrypt(ciphertext, dec_iv);
    // doc_d["accZ"] = decryptedData;

    // ------Data angular rotation X------
    sprintf(buffer, "%.2f", max_GyroX);
    encryptedData = encrypt(buffer, enc_iv);
    doc["max_GyroX"] = encryptedData;
    // sprintf(ciphertext, "%s", encryptedData.c_str());
    // decryptedData = decrypt(ciphertext, dec_iv);
    // doc_d["gyroX"] = decryptedData;

    // ------Data angular rotation Y------
    sprintf(buffer, "%.2f", max_GyroY);
    encryptedData = encrypt(buffer, enc_iv);
    doc["max_GyroY"] = encryptedData;
    // sprintf(ciphertext, "%s", encryptedData.c_str());
    // decryptedData = decrypt(ciphertext, dec_iv);
    // doc_d["gyroY"] = decryptedData;

    // ------Data angular rotation Z------
    sprintf(buffer, "%.2f", max_GyroZ);
    encryptedData = encrypt(buffer, enc_iv);
    doc["max_GyroZ"] = encryptedData;

    // ------Data acceleration X------
    sprintf(buffer, "%.2f", min_AccX);
    encryptedData = encrypt(buffer, enc_iv);
    doc["min_AccX"] = encryptedData;
    // sprintf(ciphertext, "%s", encryptedData.c_str());
    // decryptedData = decrypt(ciphertext, dec_iv);
    // doc_d["accX"] = decryptedData;

    // ------Data acceleration Y-----
    sprintf(buffer, "%.2f", min_AccY);
    encryptedData = encrypt(buffer, enc_iv);
    doc["min_AccY"] = encryptedData;
    // sprintf(ciphertext, "%s", encryptedData.c_str());
    // decryptedData = decrypt(ciphertext, dec_iv);
    // doc_d["accY"] = decryptedData;

    // ------Data acceleration Z------
    sprintf(buffer, "%.2f", min_AccZ);
    encryptedData = encrypt(buffer, enc_iv);
    doc["min_AccZ"] = encryptedData;
    // sprintf(ciphertext, "%s", encryptedData.c_str());
    // decryptedData = decrypt(ciphertext, dec_iv);
    // doc_d["accZ"] = decryptedData;

    // ------Data angular rotation X------
    sprintf(buffer, "%.2f", min_GyroX);
    encryptedData = encrypt(buffer, enc_iv);
    doc["min_GyroX"] = encryptedData;
    // sprintf(ciphertext, "%s", encryptedData.c_str());
    // decryptedData = decrypt(ciphertext, dec_iv);
    // doc_d["gyroX"] = decryptedData;

    // ------Data angular rotation Y------
    sprintf(buffer, "%.2f", min_GyroY);
    encryptedData = encrypt(buffer, enc_iv);
    doc["min_GyroY"] = encryptedData;
    // sprintf(ciphertext, "%s", encryptedData.c_str());
    // decryptedData = decrypt(ciphertext, dec_iv);
    // doc_d["gyroY"] = decryptedData;

    // ------Data angular rotation Z------
    sprintf(buffer, "%.2f", min_GyroZ);
    encryptedData = encrypt(buffer, enc_iv);
    doc["min_GyroZ"] = encryptedData;

    // ------Data acceleration X------
    sprintf(buffer, "%.2f", stdev_AccX);
    encryptedData = encrypt(buffer, enc_iv);
    doc["stdev_AccX"] = encryptedData;
    // sprintf(ciphertext, "%s", encryptedData.c_str());
    // decryptedData = decrypt(ciphertext, dec_iv);
    // doc_d["accX"] = decryptedData;

    // ------Data acceleration Y-----
    sprintf(buffer, "%.2f", stdev_AccY);
    encryptedData = encrypt(buffer, enc_iv);
    doc["stdev_AccY"] = encryptedData;
    // sprintf(ciphertext, "%s", encryptedData.c_str());
    // decryptedData = decrypt(ciphertext, dec_iv);
    // doc_d["accY"] = decryptedData;

    // ------Data acceleration Z------
    sprintf(buffer, "%.2f", stdev_AccZ);
    encryptedData = encrypt(buffer, enc_iv);
    doc["stdev_AccZ"] = encryptedData;
    // sprintf(ciphertext, "%s", encryptedData.c_str());
    // decryptedData = decrypt(ciphertext, dec_iv);
    // doc_d["accZ"] = decryptedData;

    // ------Data angular rotation X------
    sprintf(buffer, "%.2f", stdev_GyroX);
    encryptedData = encrypt(buffer, enc_iv);
    doc["stdev_GyroX"] = encryptedData;
    // sprintf(ciphertext, "%s", encryptedData.c_str());
    // decryptedData = decrypt(ciphertext, dec_iv);
    // doc_d["gyroX"] = decryptedData;

    // ------Data angular rotation Y------
    sprintf(buffer, "%.2f", stdev_GyroY);
    encryptedData = encrypt(buffer, enc_iv);
    doc["stdev_GyroY"] = encryptedData;
    // sprintf(ciphertext, "%s", encryptedData.c_str());
    // decryptedData = decrypt(ciphertext, dec_iv);
    // doc_d["gyroY"] = decryptedData;

    // ------Data angular rotation Z------
    sprintf(buffer, "%.2f", stdev_GyroZ);
    encryptedData = encrypt(buffer, enc_iv);
    doc["stdev_GyroZ"] = encryptedData;
    //send to cloud
    serializeJson(doc, Serial);
    Serial.println();
    doc.clear();
    avg_AccX = 0;
    avg_AccY = 0;
    avg_AccZ = 0;
    avg_GyroX = 0;
    avg_GyroY = 0;
    max_GyroZ = 0;
    max_AccX = 0;
    max_AccY = 0;
    max_AccZ = 0;
    max_GyroX = 0;
    max_GyroY = 0;
    max_GyroZ = 0;
    min_AccX = 0;
    min_AccY = 0;
    min_AccZ = 0;
    min_GyroX = 0;
    min_GyroY = 0;
    min_GyroZ = 0;
    stdev_AccX = 0;
    stdev_AccY = 0;
    stdev_AccZ = 0;
    stdev_GyroX = 0;
    stdev_GyroY = 0;
    stdev_GyroZ = 0;
  }
  avg_AccX = avg_AccX + NormAccX;
  avg_AccY = avg_AccY + NormAccY;
  avg_AccZ = avg_AccZ + NormAccZ;
  avg_GyroX = avg_GyroX + NormGyroX;
  avg_GyroY = avg_GyroY + NormGyroY;
  avg_GyroZ = avg_GyroZ + NormGyroZ;
  max_AccX = max(max_AccX, NormAccX);
  max_AccY = max(max_AccY, NormAccY);
  max_AccZ = max(max_AccZ, NormAccZ);
  max_GyroX = max(max_GyroX, NormGyroX);
  max_GyroY = max(max_GyroY, NormGyroY);
  max_GyroZ = max(max_GyroZ, NormGyroZ);
  min_AccX = min(min_AccX, NormAccX);
  min_AccY = min(min_AccY, NormAccY);
  min_AccZ = min(min_AccZ, NormAccZ);
  min_GyroX = min(min_GyroX, NormGyroX);
  min_GyroY = min(min_GyroY, NormGyroY);
  min_GyroZ = min(min_GyroZ, NormGyroZ);
  Array_AccX[count] = NormAccX;
  Array_AccY[count] = NormAccY;
  Array_AccZ[count] = NormAccZ;
  Array_GyroX[count] = NormGyroX;
  Array_GyroY[count] = NormGyroY;
  Array_GyroZ[count] = NormGyroZ;
  count++;


  // sprintf(ciphertext, "%s", encryptedData.c_str());
  // decryptedData = decrypt(ciphertext, dec_iv);
  // doc_d["gyroZ"] = decryptedData;
  //debugging
  // Serial.println(doc.capacity());
  // Serial.println(doc.overflowed());
  // elapsedTime = (esp_timer_get_time() - previousTime);

  // Serial.println(elapsedTime);
  // serializeJson(doc_d, Serial);
  // Serial.println();
  delay(10);
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