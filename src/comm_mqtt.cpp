#include "comm_mqtt.h"
#include <WiFi.h>
#include <PubSubClient.h>

WiFiClient espClient;
PubSubClient client(espClient);

void setupMQTT(const char* broker) {
    client.setServer(broker, 1883);
}

void publishData(const char* topic, const String& payload) {
    if (!client.connected()) {
        reconnectMQTT();
    }
    client.publish(topic, payload.c_str());
}

void reconnectMQTT() {
    while (!client.connected()) {
        if (client.connect("SSAS_Client")) {
            Serial.println("Connected to MQTT Broker.");
        } else {
            Serial.println("MQTT connection failed. Retrying...");
            delay(5000);
        }
    }
}
