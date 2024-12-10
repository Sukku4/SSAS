#ifndef COMM_MQTT_H
#define COMM_MQTT_H

#define MQTT_SERVER "your_mqtt_server"
#define MQTT_PORT 1883
#define SSID "your_wifi_ssid"
#define PASSWORD "your_wifi_password"

void setupWiFi();
void reconnect();
void sendToCloud(String payload);

#endif // COMM_MQTT_H
