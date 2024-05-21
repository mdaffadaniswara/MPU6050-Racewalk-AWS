#include "Wire.h"
#include <Arduino.h>
#include "secrets.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <queue>
#include <Ticker.h>

#define MPU_ADDR 0x68
#define FILTER_ORDER 3
const uint8_t button_PIN = 23;
#define AWS_IOT_PUBLISH_TOPIC "RaceMate/1/data"         // sesuaikan dengan yang di setting di AWS IOT CORE
#define AWS_IOT_SUBSCRIBE_TOPIC "RaceMate/1/subscribe"  // sesuaikan dengan yang di setting di AWS IOT CORE
String ID = "Athlete_01";
unsigned long getTime;
String getTime_str;

WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);
std::queue<String> messageQueue;


uint32_t intervalMPU = 10;
uint32_t last;
int16_t AccX, AccY, AccZ;
int16_t GyroX, GyroY, GyroZ;
float NormAccX, NormAccY, NormAccZ;
float NormGyroX, NormGyroY, NormGyroZ;
float AccOffsetX, AccOffsetY, AccOffsetZ, GyroOffsetX, GyroOffsetY, GyroOffsetZ;

float avg_AccX, avg_AccY, avg_AccZ, avg_GyroX, avg_GyroY, avg_GyroZ;
float max_AccX, max_AccY, max_AccZ, max_GyroX, max_GyroY, max_GyroZ;
float min_AccX, min_AccY, min_AccZ, min_GyroX, min_GyroY, min_GyroZ;
float stdev_AccX, stdev_AccY, stdev_AccZ, stdev_GyroX, stdev_GyroY, stdev_GyroZ;
float Array_AccX[201], Array_AccY[201], Array_AccZ[201], Array_GyroX[201], Array_GyroY[201], Array_GyroZ[201];

float StrideCheck1 = 500.0;
float StrideCheck2 = 500.0;
float MaxCheck = (-99.0);
int DataCount, DataProcess, DataStart = 0;
int Start = 2;
int StrideChange = 2;
int MaxStop = 0;
int DataThreshold = 38;

float refG = 9.8037f;
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
int flag_isr = 0;
uint32_t last_button_time = 0;
uint32_t button_time;

float MEAN_SCALER[24] = { 3.76271545, -10.65023171, -1.31324797, -4.06612195, 2.92721545, -3.66056504, 23.30673577, 2.80148374, 7.02997154, 68.79730894, 155.73365447, 191.5308374, -13.49200813, -26.86039837, -9.91650407, -97.37462195, -212.9678252, -249.09442683, 9.02141057, 6.86697561, 4.01061382, 48.09696748, 93.22341463, 147.83123171 };
float STD_SCALER[24] = { 3.10959788, 1.95026829, 1.62881921, 35.00904887, 20.91665539, 114.55124895, 7.65555349, 6.23099875, 3.14651746, 66.22459166, 51.50013034, 228.86725294, 6.44327081, 5.38475926, 3.31712396, 44.98519303, 54.66344971, 40.38267696, 2.97103616, 1.89276914, 1.05704692, 28.52512897, 23.00918837, 83.8071792 };

void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("Connected to AP successfully!");
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("Trying to Reconnect");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // wifiMulti.run();
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
  doc["ID"] = ID;
  doc["event_timestamp"] = getTime_str;
  doc["acc_x_avg"] = avg_AccX;
  doc["acc_y_avg"] = avg_AccY;
  doc["acc_z_avg"] = avg_AccZ;
  doc["gyro_x_avg"] = avg_GyroX;
  doc["gyro_y_avg"] = avg_GyroY;
  doc["gyro_z_avg"] = avg_GyroZ;
  doc["acc_x_max"] = max_AccX;
  doc["acc_y_max"] = max_AccY;
  doc["acc_z_max"] = max_AccZ;
  doc["gyro_x_max"] = max_GyroX;
  doc["gyro_y_max"] = max_GyroY;
  doc["gyro_z_max"] = max_GyroZ;
  doc["acc_x_min"] = min_AccX;
  doc["acc_y_min"] = min_AccY;
  doc["acc_z_min"] = min_AccZ;
  doc["gyro_x_min"] = min_GyroX;
  doc["gyro_y_min"] = min_GyroY;
  doc["gyro_z_min"] = min_GyroZ;
  doc["acc_x_stdev"] = stdev_AccX;
  doc["acc_y_stdev"] = stdev_AccY;
  doc["acc_z_stdev"] = stdev_AccZ;
  doc["gyro_x_stdev"] = stdev_GyroX;
  doc["gyro_y_stdev"] = stdev_GyroY;
  doc["gyro_z_stdev"] = stdev_GyroZ;
  char jsonBuffer[4096];
  serializeJson(doc, Serial);  // print to client
  Serial.println();
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
  Serial.print(FilteredAccX, 4);
  Serial.print(';');
  // // // Serial.print(NormAccY);
  // // // Serial.print(';');
  Serial.print(FilteredAccY, 4);
  Serial.print(';');
  // // // Serial.print(NormAccZ);
  // // // Serial.print(';');
  Serial.print(FilteredAccZ, 4);
  Serial.print(';');
  // Serial.print(NormGyroX);
  // Serial.print(';');
  Serial.print(FilteredGyroX, 4);
  Serial.print(';');
  // // Serial.print(NormGyroY);
  // // Serial.print(';');
  Serial.print(FilteredGyroY, 4);
  Serial.print(';');
  // Serial.print(NormGyroZ);
  // Serial.print(';');
  Serial.print(FilteredGyroZ, 4);
  Serial.print(';');
  Serial.print(DataThreshold);
  Serial.print(';');
  Serial.print(StrideChange);
  Serial.println(';');
}

void DEBUG_PRINT() {
  // Serial.print()
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

void IRAM_ATTR isr() {
  if (millis() - last_button_time > 500) {
    flag_isr = 1;
    last_button_time = millis();
  }
}

void setup() {
  Serial.begin(230400);
  client.setBufferSize(4096);

  pinMode(button_PIN, INPUT_PULLUP);
  attachInterrupt(button_PIN, isr, FALLING);

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
  WiFi.mode(WIFI_STA);

  WiFi.onEvent(WiFiStationConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(WiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  connectAWS();
}

// void loop(){}
void loop() {
  if (flag_isr == 1) {
    calculate_IMU_error(&AccOffsetX, &AccOffsetY, &AccOffsetZ, &GyroOffsetX, &GyroOffsetY, &GyroOffsetZ);
    flag_isr = 0;
  }
  // if (WiFi.status() != WL_CONNECTED) {
  //   Serial.println("Trying to Reconnect");
  //   WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // }
  if (!client.connected()) {
    reconnectAWS();
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
    NormAccX = (AccX * rangeAcc) * 9.80665f - 0.3094;
    NormAccY = (AccY * rangeAcc) * 9.80665f + 0.2234;
    NormAccZ = (AccZ * rangeAcc) * 9.80665f - 0.9274;
    // NormAccX = (AccX * rangeAcc) * 9.80665f - AccOffsetX;
    // NormAccY = (AccY * rangeAcc) * 9.80665f - AccOffsetY;
    // NormAccZ = (AccZ * rangeAcc) * 9.80665f - AccOffsetZ;

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);  // Gyro data first register address 0x43
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);  // Read 6 registers total, each axis value is stored in 2 registers
    // For a 500deg/s range we have to divide first the raw value by 65.5, according to the datasheet
    GyroX = ((Wire.read() << 8) | (Wire.read()));  // X-axis value
    GyroY = ((Wire.read() << 8) | (Wire.read()));  // Y-axis value
    GyroZ = ((Wire.read() << 8) | (Wire.read()));  // Z-axis value
    // Correct the outputs with the calculated error values
    NormGyroX = (GyroX * rangeGyro) + 1.0229;  // GyroErrorX ~(-0.56)
    NormGyroY = (GyroY * rangeGyro) + 2.7209;  // GyroErrorY ~(2)
    NormGyroZ = (GyroZ * rangeGyro) - 1.0408;  // GyroErrorZ ~ (-0.8)
    // NormGyroX = (GyroX * rangeGyro) - GyroOffsetX;  // GyroErrorX ~(-0.56)
    // NormGyroY = (GyroY * rangeGyro) - GyroOffsetY;  // GyroErrorY ~(2)
    // NormGyroZ = (GyroZ * rangeGyro) - GyroOffsetX;  // GyroErrorZ ~ (-0.8)

    FilteredAccX = FILTER(NormAccX, BUFFER_B_acc_x, BUFFER_A_acc_x, counter);
    FilteredAccY = FILTER(NormAccY, BUFFER_B_acc_y, BUFFER_A_acc_y, counter);
    FilteredAccZ = FILTER(NormAccZ, BUFFER_B_acc_z, BUFFER_A_acc_z, counter);
    FilteredGyroX = FILTER(NormGyroX, BUFFER_B_gyro_x, BUFFER_A_gyro_x, counter);
    FilteredGyroY = FILTER(NormGyroY, BUFFER_B_gyro_y, BUFFER_A_gyro_y, counter);
    FilteredGyroZ = FILTER(NormGyroZ, BUFFER_B_gyro_z, BUFFER_A_gyro_z, counter);

    // Tanda stride check dengan deteksi bentuk maxima
    if ((FilteredGyroZ >= 300.0) && (Start == 2)) {
      MaxCheck = max(MaxCheck, FilteredGyroZ);
      if (MaxCheck != FilteredGyroZ) {  //grafik sudah sampai maxima dan mulai turun
        Start = 0;
        DataThreshold = 38;
      }
    }
    // Stride check by detecting local minima
    if (Start == 0) {
      StrideCheck1 = min(StrideCheck1, FilteredGyroZ);
      if (StrideCheck1 != FilteredGyroZ) {  // Menandakan grafik gyro-Z sudah naik kembali -- menandakan pembacaan sebelumnya merupakan local minima
        MaxCheck = (-99.0);
        StrideChange = 1;  // StrideChange = 1 -> kaki x, StrideChange = 0 -> kaki y
        DataCount = 0;
        Start = 1;
      }
    }
    // Divide data dan proses berdasarkan stride
    if (StrideChange == 1) {
      //proses data
      DATA_PROCESSING();
      DataCount = DataCount + 1;
      if (DataCount > DataThreshold) {  // Atur hingga dirasa data sudah selalu turun terus
        StrideCheck2 = min(StrideCheck2, FilteredGyroZ);
        if (StrideCheck2 < FilteredGyroZ) {
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
      if ((FilteredGyroZ >= 300.0) && (MaxStop == 0)) {
        MaxCheck = max(MaxCheck, FilteredGyroZ);
        if (MaxCheck > FilteredGyroZ) {  //grafik sudah sampai maxima dan mulai turun
          DataThreshold = (DataCount + 8);
          MaxStop = 1;
        }
      }
      if ((MaxStop == 1) && (DataCount > DataThreshold)) {
        StrideCheck1 = min(StrideCheck1, FilteredGyroZ);
        if (StrideCheck1 < FilteredGyroZ) {
          StrideCheck2 = 500.0;
          MaxCheck = (-99.0);
          StrideChange = 1;
          MaxStop = 0;
          DataProcess = 1;
        }
      }
      if (DataCount >= 100) {
        StrideCheck2 = 500.0;
        MaxCheck = (-99.0);
        MaxStop = 0;
        DataCount = 0;
        RESET_DATA();
        Start = 2;
        StrideChange = 2;
      }
    }
    if (DataProcess == 1) {
      //Proses kirim data
      getTime = millis();
      unsigned long miliseconds = getTime % 1000;
      unsigned long seconds = getTime / 1000;
      unsigned long minutes = seconds / 60;
      unsigned long hours = minutes / 60;
      seconds %= 60;
      minutes %= 60;
      getTime_str = String(hours) + ":" + String(minutes) + ":" + String(seconds) + ":" + String(miliseconds);

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
      if (avg_AccX < 5.0f || DataCount <= 3) {
        Start = 2;
        StrideChange = 2;
      }
      // Normalisasi data
      avg_AccX = (avg_AccX - MEAN_SCALER[0]) / STD_SCALER[0];
      avg_AccY = (avg_AccY - MEAN_SCALER[1]) / STD_SCALER[1];
      avg_AccZ = (avg_AccZ - MEAN_SCALER[2]) / STD_SCALER[2];
      avg_GyroX = (avg_GyroX - MEAN_SCALER[3]) / STD_SCALER[3];
      avg_GyroY = (avg_GyroY - MEAN_SCALER[4]) / STD_SCALER[4];
      avg_GyroZ = (avg_GyroZ - MEAN_SCALER[5]) / STD_SCALER[5];
      max_AccX = (max_AccX - MEAN_SCALER[6]) / STD_SCALER[6];
      max_AccY = (max_AccY - MEAN_SCALER[7]) / STD_SCALER[7];
      max_AccZ = (max_AccZ - MEAN_SCALER[8]) / STD_SCALER[8];
      max_GyroX = (max_GyroX - MEAN_SCALER[9]) / STD_SCALER[9];
      max_GyroY = (max_GyroY - MEAN_SCALER[10]) / STD_SCALER[10];
      max_GyroZ = (max_GyroZ - MEAN_SCALER[11]) / STD_SCALER[11];
      min_AccX = (min_AccX - MEAN_SCALER[12]) / STD_SCALER[12];
      min_AccY = (min_AccY - MEAN_SCALER[13]) / STD_SCALER[13];
      min_AccZ = (min_AccZ - MEAN_SCALER[14]) / STD_SCALER[14];
      min_GyroX = (min_GyroX - MEAN_SCALER[15]) / STD_SCALER[15];
      min_GyroY = (min_GyroY - MEAN_SCALER[16]) / STD_SCALER[16];
      min_GyroZ = (min_GyroZ - MEAN_SCALER[17]) / STD_SCALER[17];
      stdev_AccX = (stdev_AccX - MEAN_SCALER[18]) / STD_SCALER[18];
      stdev_AccY = (stdev_AccY - MEAN_SCALER[19]) / STD_SCALER[19];
      stdev_AccZ = (stdev_AccZ - MEAN_SCALER[20]) / STD_SCALER[20];
      stdev_GyroX = (stdev_GyroX - MEAN_SCALER[21]) / STD_SCALER[21];
      stdev_GyroY = (stdev_GyroY - MEAN_SCALER[22]) / STD_SCALER[22];
      stdev_GyroZ = (stdev_GyroZ - MEAN_SCALER[23]) / STD_SCALER[23];
      // DEBUG_PRINT();
      publishMessage();
      client.loop();
      DataCount = 0;
      RESET_DATA();
      DataProcess = 0;
    }
    // DEBUG_PRINT_MPU();
    counter = counter + 1;
    if (counter > FILTER_ORDER) {
      counter = 0;
    }
    // counter = (counter + 1) % (FILTER_ORDER + 1);
    //   delay(10);
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
