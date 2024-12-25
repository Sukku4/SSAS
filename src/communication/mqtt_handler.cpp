#include "communication/mqtt_handler.h"
#include <PubSubClient.h>

MQTTHandler::MQTTHandler() : client(wifiClient) {}

void MQTTHandler::setup() {
    client.setServer(MQTT_BROKER, MQTT_PORT);
    connect();
}

void MQTTHandler::connect() {
    while (!client.connected()) {
        if (client.connect("SSAS_Client")) {
            // Subscribe to topics if needed
        } else {
            delay(5000);
        }
    }
}

void MQTTHandler::publishSensorData(const std::map<String, float>& data) {
    for (const auto& [key, value] : data) {
        client.publish(("sensor/" + key).c_str(), String(value).c_str());
    }
}

void MQTTHandler::publishAlert(const String& message) {
    client.publish("alerts", message.c_str());
}
