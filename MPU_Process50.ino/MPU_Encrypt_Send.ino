#include "secrets.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <Wire.h>
#include <AESLib.h>

AESLib aesLib;
#define MPU_ADDR 0x68
#define AWS_IOT_PUBLISH_TOPIC "RaceMate/+/data"        // sesuaikan dengan yang di setting di AWS IOT CORE
#define AWS_IOT_SUBSCRIBE_TOPIC "RaceMate/1/data_sub"  // sesuaikan dengan yang di setting di AWS IOT CORE

WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);
StaticJsonDocument<2000> doc;

int ID = 01;
unsigned long getTime;
uint32_t intervalMPU = 10;
uint32_t last;
int DataProcess;

float avg_AccX, avg_AccY, avg_AccZ, avg_GyroX, avg_GyroY, avg_GyroZ;
float max_AccX, max_AccY, max_AccZ, max_GyroX, max_GyroY, max_GyroZ;
float min_AccX, min_AccY, min_AccZ, min_GyroX, min_GyroY, min_GyroZ;
float stdev_AccX, stdev_AccY, stdev_AccZ, stdev_GyroX, stdev_GyroY, stdev_GyroZ;


char buffer[100];
char ciphertext[30];
byte aes_key[] = { 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30 };
byte aes_iv[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.println("Connected to AP successfully!");
}

void reconnectAWS() {
  while (!client.connected()) {
    Serial.println("Menghubungkan dengan AWS IOT Core");
    if (client.connect(THINGNAME)) {
      Serial.println("Terhubung dengan AWS IoT Core");
      client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
    }
  }
}

void connectAWS() {
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

void aes_init() {
  aesLib.gen_iv(aes_iv);
  aesLib.set_paddingmode((paddingMode)2);  //Padding mode: ZeroLength
}

// Fungsi: melakukan enkripsi AES 128 CBC berdasarkan key dan inisialization vector (iv)
String encrypt(char* msg, byte iv[]) {
  int msgLen = strlen(msg);
  char encrypted[2 * msgLen];
  aesLib.encrypt64((byte*)msg, msgLen, encrypted, aes_key, sizeof(aes_key), iv);
  return String(encrypted);
}

String decrypt(char* msg, byte iv[]) {
  int msgLen = strlen(msg);
  char decrypted[msgLen];
  aesLib.decrypt64(msg, msgLen, (byte*)decrypted, aes_key, sizeof(aes_key), iv);
  return String(decrypted);
}

void setup() {
  Serial.begin(230400);
  client.setBufferSize(4096);

  Wire.begin();  // Initialize comunication
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  delay(20);
  aes_init();

  WiFi.mode(WIFI_STA);
  WiFi.onEvent(WiFiStationConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.begin(WIFI_SSID1, WIFI_PASSWORD1);
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
    // === Read accelerometer data === //
    // === Read gyroscope data === //
    // === Melakukan filter pada data hasil pembacaan === //
    // === Memulai identifikasi langkah === //
    // Jika langkah selesai: DataProcess = 1
    // Proses semua data yang telah dimasukkan ke perhitungan
    if (DataProcess == 1) {
      // Proses hitung data
      // Ambil waktu sekarang
      getTime = millis();
      unsigned long seconds = getTime / 1000;
      unsigned long minutes = seconds / 60;
      unsigned long hours = minutes / 60;
      seconds %= 60;
      minutes %= 60;
      String getTime_str = String(hours) + ":" + String(minutes) + ":" + String(seconds);
      // Reset nilai enc_iv dan dec_iv
      byte enc_iv[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  // iv_block gets written to, reqires always fresh copy.
      byte dec_iv[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };  // iv_block gets written to, requires always fresh copy.

      // Memulai Enkripsi
      // === Data nomor pelari ===
      sprintf(buffer, "%d", ID);
      String encryptedData = encrypt(buffer, enc_iv);
      doc["ID"] = encryptedData;
      // === Data waktu pengambilan data ===
      sprintf(buffer, "%s", getTime_str);
      encryptedData = encrypt(buffer, enc_iv);
      doc["time"] = encryptedData;
      // === Data Rata-rata ===
      sprintf(buffer, "%.2f", avg_AccX);
      encryptedData = encrypt(buffer, enc_iv);
      doc["acc_x_avg"] = encryptedData;
      sprintf(buffer, "%.2f", avg_AccY);
      encryptedData = encrypt(buffer, enc_iv);
      doc["acc_y_avg"] = encryptedData;
      sprintf(buffer, "%.2f", avg_AccZ);
      encryptedData = encrypt(buffer, enc_iv);
      doc["acc_z_avg"] = encryptedData;
      sprintf(buffer, "%.2f", avg_GyroX);
      encryptedData = encrypt(buffer, enc_iv);
      doc["gyro_x_avg"] = encryptedData;
      sprintf(buffer, "%.2f", avg_GyroY);
      encryptedData = encrypt(buffer, enc_iv);
      doc["gyro_y_avg"] = encryptedData;
      sprintf(buffer, "%.2f", avg_GyroZ);
      encryptedData = encrypt(buffer, enc_iv);
      doc["gyro_z_avg"] = encryptedData;
      // === Data Maksimal ===
      sprintf(buffer, "%.2f", max_AccX);
      encryptedData = encrypt(buffer, enc_iv);
      doc["acc_x_max"] = encryptedData;
      sprintf(buffer, "%.2f", max_AccY);
      encryptedData = encrypt(buffer, enc_iv);
      doc["acc_y_max"] = encryptedData;
      sprintf(buffer, "%.2f", max_AccZ);
      encryptedData = encrypt(buffer, enc_iv);
      doc["acc_z_max"] = encryptedData;
      sprintf(buffer, "%.2f", max_GyroX);
      encryptedData = encrypt(buffer, enc_iv);
      doc["gyro_x_max"] = encryptedData;
      sprintf(buffer, "%.2f", max_GyroY);
      encryptedData = encrypt(buffer, enc_iv);
      doc["gyro_y_max"] = encryptedData;
      sprintf(buffer, "%.2f", max_GyroZ);
      encryptedData = encrypt(buffer, enc_iv);
      doc["gyro_z_max"] = encryptedData;
      // === Data Minimal ===
      sprintf(buffer, "%.2f", min_AccX);
      encryptedData = encrypt(buffer, enc_iv);
      doc["acc_x_min"] = encryptedData;
      sprintf(buffer, "%.2f", min_AccY);
      encryptedData = encrypt(buffer, enc_iv);
      doc["acc_y_min"] = encryptedData;
      sprintf(buffer, "%.2f", min_AccZ);
      encryptedData = encrypt(buffer, enc_iv);
      doc["acc_z_min"] = encryptedData;
      sprintf(buffer, "%.2f", min_GyroX);
      encryptedData = encrypt(buffer, enc_iv);
      doc["gyro_x_min"] = encryptedData;
      sprintf(buffer, "%.2f", min_GyroY);
      encryptedData = encrypt(buffer, enc_iv);
      doc["gyro_y_min"] = encryptedData;
      sprintf(buffer, "%.2f", min_GyroZ);
      encryptedData = encrypt(buffer, enc_iv);
      doc["gyro_z_min"] = encryptedData;
      // === Data Standar Deviasi ===
      sprintf(buffer, "%.2f", stdev_AccX);
      encryptedData = encrypt(buffer, enc_iv);
      doc["acc_x_stdev"] = encryptedData;
      sprintf(buffer, "%.2f", stdev_AccY);
      encryptedData = encrypt(buffer, enc_iv);
      doc["acc_y_stdev"] = encryptedData;
      sprintf(buffer, "%.2f", stdev_AccZ);
      encryptedData = encrypt(buffer, enc_iv);
      doc["acc_z_stdev"] = encryptedData;
      sprintf(buffer, "%.2f", stdev_GyroX);
      encryptedData = encrypt(buffer, enc_iv);
      doc["gyro_x_stdev"] = encryptedData;
      sprintf(buffer, "%.2f", stdev_GyroY);
      encryptedData = encrypt(buffer, enc_iv);
      doc["gyro_y_stdev"] = encryptedData;
      sprintf(buffer, "%.2f", stdev_GyroZ);
      encryptedData = encrypt(buffer, enc_iv);
      doc["gyro_z_stdev"] = encryptedData;
      publishMessage();
      client.loop();
      // RESET_DATA();
      doc.clear();
      DataProcess = 0;
    }
    last += intervalMPU;
  }
}
