#include "comm_mqtt.h"
#include <WiFi.h>
#include <PubSubClient.h>

WiFiClient espClient;
PubSubClient client(espClient);

void setupWiFi() {
    delay(10);
    WiFi.begin(SSID, PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }
}

void reconnect() {
    while (!client.connected()) {
        if (client.connect("ESP32Client")) {
            client.subscribe("inTopic");
        } else {
            delay(5000);
        }
    }
}

void sendToCloud(String payload) {
    client.publish("outTopic", payload.c_str());
}
