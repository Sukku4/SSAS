#include <WiFi.h>
#include <PubSubClient.h>
#include <AESLib.h>

// Wi-Fi credentials
const char* ssid = "Your_SSID";
const char* password = "Your_PASSWORD";

// MQTT Broker details
const char* mqtt_server = "mqtt.example.com";
const int mqtt_port = 8883;
const char* mqtt_user = "username";
const char* mqtt_pass = "password";

// AES Key (16 bytes)
char aes_key[] = "1234567890abcdef";

// Certificates (replace with actual CA, cert, and key)
const char* root_ca = "-----BEGIN CERTIFICATE-----\n...\n-----END CERTIFICATE-----";
const char* client_cert = "-----BEGIN CERTIFICATE-----\n...\n-----END CERTIFICATE-----";
const char* private_key = "-----BEGIN PRIVATE KEY-----\n...\n-----END PRIVATE KEY-----";

// Wi-Fi and MQTT Clients
WiFiClientSecure espClient;
PubSubClient client(espClient);

// AESLib instance
AESLib aesLib;

// Encrypt function
String encryptData(String plainText) {
  char encrypted[64];
  aesLib.encrypt((byte*)plainText.c_str(), (byte*)encrypted, aes_key);
  return String(encrypted);
}

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Set certificates
  espClient.setCACert(root_ca);
  espClient.setCertificate(client_cert);
  espClient.setPrivateKey(private_key);

  // Set MQTT server
  client.setServer(mqtt_server, mqtt_port);
}

void loop() {
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  // Example sensor data
  String sensorData = "temperature=25.3,humidity=60.2,soilMoisture=45";
  String encryptedData = encryptData(sensorData);

  // Publish encrypted data
  client.publish("SSAS/sensors", encryptedData.c_str());
  delay(5000); // Transmit every 5 seconds
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_pass)) {
      Serial.println("Connected to MQTT broker with TLS");
    } else {
      delay(5000);
    }
  }
}

