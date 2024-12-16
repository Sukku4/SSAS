#include "comm_mqtt.h"
#include <WiFi.h>
#include <PubSubClient.h>

WiFiClient espClient;
PubSubClient client(espClient);

void setupMQTT(const char* broker) {
  WiFi.begin("Your_SSID", "Your_PASSWORD");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  client.setServer(broker, 1883);
}

void publishData(const char* topic, const String& payload) {
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();
  client.publish(topic, payload.c_str(), false); // QoS 0 for reduced power
}

String aggregateSensorData() {
  String payload = "timestamp=2024-11-22T10:00:00Z,";
  payload += "temperature=25.3,humidity=60.2,soilMoisture=45,";
  payload += "lightIntensity=800.5,windSpeed=3.5,ndvi=0.85,co2=400";
  return payload;
}

void reconnectMQTT() {
  while (!client.connected()) {
    if (client.connect("ESP32Client")) {
      Serial.println("Connected to MQTT broker");
    } else {
      delay(5000);
    }
  }
}
