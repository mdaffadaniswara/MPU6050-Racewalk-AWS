#include "secrets.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
#include <Wire.h>
// #include <Arduino.h>


#define MPU_ADDR 0x68
#define FILTER_ORDER 3
#define AWS_IOT_PUBLISH_TOPIC "device/2/data"        // sesuaikan dengan yang di setting di AWS IOT CORE
#define AWS_IOT_SUBSCRIBE_TOPIC "device/2/data_sub"  // sesuaikan dengan yang di setting di AWS IOT CORE

WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

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

float rangePerDigit = .00006103515625f;
int c = 0;
int count = 0;
int counter = 0;
float FILTER_COEFF_B[FILTER_ORDER + 1] = { 1.0000, -0.5772, 0.4218, -0.0563 };
float FILTER_COEFF_A[FILTER_ORDER + 1] = { 0.0985, 0.2956, 0.2956, 0.0985 };
float BUFFER_A[FILTER_ORDER + 1];
float BUFFER_B[FILTER_ORDER + 1];

void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("Connected to AP successfully!");
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.wifi_sta_disconnected.reason);
  Serial.println("Trying to Reconnect");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectAWS() {
  // WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.setServer(AWS_IOT_ENDPOINT, 8883);

  // Create a message handler
  client.setCallback(messageHandler);

  Serial.println("Terhubung dengan AWS IOT Core");
  delay(100);
  while (!client.connect(THINGNAME)) {
    Serial.print(".");
    delay(100);
  }

  if (!client.connected()) {
    Serial.println("AWS IoT Timeout!");
    return;
  }

  // Subscribe to a topic
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);

  Serial.println("Terhubung dengan AWS IoT Core");
}

void publishMessage() {
  StaticJsonDocument<2000> doc;
  doc["Average Acceleration X"] = avg_AccX;
  doc["Average Acceleration Y"] = avg_AccY;
  doc["Average Acceleration Z"] = avg_AccZ;
  doc["Max Acceleration X"] = max_AccX;
  doc["Max Acceleration Y"] = max_AccY;
  doc["Max Acceleration Z"] = max_AccZ;
  doc["Min Acceleration X"] = min_AccX;
  doc["Min Acceleration Y"] = min_AccY;
  doc["Min Acceleration Z"] = min_AccZ;
  doc["STDev Acceleration X"] = stdev_AccX;
  doc["STDev Acceleration Y"] = stdev_AccY;
  doc["STDev Acceleration Z"] = stdev_AccZ;
  doc["Average Gyroscope X"] = avg_GyroX;
  doc["Average Gyroscope Y"] = avg_GyroY;
  doc["Average Gyroscope Z"] = avg_GyroZ;
  doc["Max Gyroscope X"] = max_GyroX;
  doc["Max Gyroscope Y"] = max_GyroY;
  doc["Max Gyroscope Z"] = max_GyroZ;
  doc["Min Gyroscope X"] = min_GyroX;
  doc["Min Gyroscope Y"] = min_GyroY;
  doc["Min Gyroscope Z"] = min_GyroZ;
  doc["STDev Gyroscope X"] = stdev_GyroX;
  doc["STDev Gyroscope Y"] = stdev_GyroY;
  doc["STDev Gyroscope Z"] = stdev_GyroZ;
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);  // print to client

  client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
}

void messageHandler(char* topic, byte* payload, unsigned int length) {
  Serial.print("incoming: ");
  Serial.println(topic);

  StaticJsonDocument<200> doc;
  deserializeJson(doc, payload);
  const char* message = doc["message"];
  Serial.println(message);
}

float filter(float input, int counter) {
  float FILTERED_DATA;

  // for (int i = FILTER_ORDER; i > 0; i--) {
  //   BUFFER_A[i] = BUFFER_A[i - 1];
  //   BUFFER_B[i] = BUFFER_B[i - 1];
  // }
  BUFFER_B[counter] = input;
  FILTERED_DATA = 0;
  for (int i = 0; i <= FILTER_ORDER; i++) {
    FILTERED_DATA = FILTERED_DATA + BUFFER_B[(FILTER_ORDER + counter - i + 1) % (FILTER_ORDER + 1)] * FILTER_COEFF_B[i];
  }
  for (int i = 0; i < FILTER_ORDER; i++) {
    FILTERED_DATA = FILTERED_DATA - BUFFER_A[(FILTER_ORDER + counter - i + 1) % (FILTER_ORDER + 1)] * FILTER_COEFF_A[i + 1];
  }
  BUFFER_A[counter] = FILTERED_DATA;

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

void setup() {
  Serial.begin(1000000);
  client.setBufferSize(4096);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU_ADDR);  // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0);                     // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  // Call this function if you need to get the IMU error values for your module
  // calculate_IMU_error();
  // SerialBT.begin("ESP32test");  //Bluetooth device name
  for (int i = 0; i <= FILTER_ORDER; i++) {
    BUFFER_A[i] = 0;
    BUFFER_B[i] = 0;
  }
  delay(20);
  WiFi.onEvent(WiFiStationConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(WiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  connectAWS();
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
  NormAccX = AccX * rangePerDigit * 9.80665f - 0.82;
  NormAccY = AccY * rangePerDigit * 9.80665f - 0.01;
  NormAccZ = AccZ * rangePerDigit * 9.80665f - 10.08;
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
  NormGyroX = (GyroX / 131.0) + 0.49;  // GyroErrorX ~(-0.56)
  NormGyroY = (GyroY / 131.0) + 3.90;  // GyroErrorY ~(2)
  NormGyroZ = (GyroZ / 131.0) - 0.47;  // GyroErrorZ ~ (-0.8)

  //processing data
  if (count == 50) {
    // if (WiFi.status() != WL_CONNECTED){
    //   WiFi.disconnect();
    //   WiFi.reconnect();
    // }
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
    publishMessage();
    client.loop();
    count = 0;
    avg_AccX = 0;
    avg_AccY = 0;
    avg_AccZ = 0;
    avg_GyroX = 0;
    avg_GyroY = 0;
    avg_GyroZ = 0;
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

  // float test;
  // test = filter(NormAccX, counter);
  // ------PRINT TO serial---------
  // Serial.print(NormAccX);
  // Serial.print(';');
  // Serial.print(NormAccY);
  // Serial.print(';');
  // Serial.print(NormAccZ);
  // Serial.print(';');
  // Serial.print(NormGyroX);
  // Serial.print(';');
  // Serial.print(NormGyroY);
  // Serial.print(';');
  // Serial.print(NormGyroZ);
  // Serial.println(';');
  // counter = counter + 1;
  // if (counter > FILTER_ORDER) {
  //   counter = 0;
  // }
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