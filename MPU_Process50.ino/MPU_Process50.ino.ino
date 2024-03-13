#include "secrets.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WiFiMulti.h>
WiFiMulti wifiMulti;
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

float rangePerDigit = .00006103515625f;
int c = 0;
int count = 0;
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

const int PIN_config = 18;
uint32_t button_time, last_button;

void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("Connected to AP successfully!");
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  // Serial.println("Disconnected from WiFi access point");
  // Serial.print("WiFi lost connection. Reason: ");
  // Serial.println(info.wifi_sta_disconnected.reason);
  Serial.println("Trying to Reconnect");
  WiFi.begin(WIFI_SSID1, WIFI_PASSWORD1);
  // wifiMulti.run();
}

void IRAM_ATTR ISR() {
  button_time = millis();
  if (button_time - last_button >= 500) {
    calculate_IMU_error();
    last_button = button_time;
  }
}

void reconnectAWS() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.println("Menghubungkan dengan AWS IOT Core");
    // Attempt to connect
    if (client.connect(THINGNAME)) {
      Serial.println("Terhubung dengan AWS IoT Core");
      client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
      // } else {
      //   Serial.print("failed, rc=");
      //   Serial.print(client.state());
      //   Serial.println(" try again in 5 seconds");
      //   // Wait 5 seconds before retrying
      //   delay(5000);
    }
  }
}

void connectAWS() {
  // WiFi.mode(WIFI_STA);
  // WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

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

  reconnectAWS();
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

void setup() {
  Serial.begin(1000000);
  client.setBufferSize(4096);

  pinMode(PIN_config, INPUT_PULLUP);
  attachInterrupt(PIN_config, ISR, FALLING);

  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU_ADDR);  // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0);                     // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  // Call this function if you need to get the IMU error values for your module
  // calculate_IMU_error();
  // SerialBT.begin("ESP32test");  //Bluetooth device name
  delay(20);
  WiFi.mode(WIFI_STA);
  // wifiMulti.addAP(WIFI_SSID1, WIFI_PASSWORD1);
  // wifiMulti.addAP(WIFI_SSID2, WIFI_PASSWORD2);
  // wifiMulti.addAP(WIFI_SSID3, WIFI_PASSWORD3);

  WiFi.onEvent(WiFiStationConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  // WiFi.onEvent(WiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  WiFi.begin(WIFI_SSID1, WIFI_PASSWORD1);
  // wifiMulti.run();
  connectAWS();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Trying to Reconnect");
    WiFi.begin(WIFI_SSID1, WIFI_PASSWORD1);
  }
  if (!client.connected()) {
    reconnectAWS();
  }
  if (millis() - last >= intervalMPU) {  // === Read acceleromter data === //
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);  // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);  // Read 6 registers total, each axis value is stored in 2 registers
    //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
    AccX = ((Wire.read() << 8) | (Wire.read()));  // X-axis value
    AccY = ((Wire.read() << 8) | (Wire.read()));  // Y-axis value
    AccZ = ((Wire.read() << 8) | (Wire.read()));  // Z-axis value
    //Normalisasi Raw Data tersebut
    AccErrorX = 0.04;
    AccErrorY = (-0.10);
    AccErrorZ = 0.367;
    NormAccX = AccX * rangePerDigit * 9.80665f - AccErrorX; // 1/16384 -> rangePerDigit
    NormAccY = AccY * rangePerDigit * 9.80665f - AccErrorY;
    NormAccZ = AccZ * rangePerDigit * 9.80665f - AccErrorZ;

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);  // Gyro data first register address: 0x43
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);           // Read 6 registers total, each axis value is stored in 2 registers
    GyroX = ((Wire.read() << 8) | (Wire.read()));  // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
    GyroY = ((Wire.read() << 8) | (Wire.read()));
    GyroZ = ((Wire.read() << 8) | (Wire.read()));
    // Correct the outputs with the calculated error values
    GyroErrorX = (-0.01);
    GyroErrorY = (-0.08);
    GyroErrorZ = 0.01;
    NormGyroX = (GyroX * 0.017453 / 131.0) - GyroErrorX;  // convert deg/s to rad/s (* 0.017453)
    NormGyroY = (GyroY * 0.017453 / 131.0) - GyroErrorY;  
    NormGyroZ = (GyroZ * 0.017453 / 131.0) - GyroErrorZ;  

    // Filtering acquired data
    float FilteredAccX, FilteredAccY, FilteredAccZ, FilteredGyroX, FilteredGyroY, FilteredGyroZ;
    FilteredAccX = FILTER(NormAccX, BUFFER_B_acc_x, BUFFER_A_acc_x, counter);
    FilteredAccY = FILTER(NormAccY, BUFFER_B_acc_y, BUFFER_A_acc_y, counter);
    FilteredAccZ = FILTER(NormAccZ, BUFFER_B_acc_z, BUFFER_A_acc_z, counter);
    FilteredGyroX = FILTER(NormGyroX, BUFFER_B_gyro_x, BUFFER_A_gyro_x, counter);
    FilteredGyroY = FILTER(NormGyroY, BUFFER_B_gyro_y, BUFFER_A_gyro_y, counter);
    FilteredGyroZ = FILTER(NormGyroZ, BUFFER_B_gyro_z, BUFFER_A_gyro_z, counter);

    // Processing data to get time domain information
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
    Array_AccX[count] = FilteredAccX;
    Array_AccY[count] = FilteredAccY;
    Array_AccZ[count] = FilteredAccZ;
    Array_GyroX[count] = FilteredGyroX;
    Array_GyroY[count] = FilteredGyroY;
    Array_GyroZ[count] = FilteredGyroZ;
    count = count + 1;
    last += intervalMPU;

    counter = counter + 1;
    if (counter > FILTER_ORDER) {
      counter = 0;
    }
  }
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
