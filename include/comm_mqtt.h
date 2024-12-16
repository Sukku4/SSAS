#ifndef COMM_MQTT_H
#define COMM_MQTT_H

void setupMQTT(const char* broker);
void publishData(const char* topic, const String& payload);
String aggregateSensorData();
void reconnectMQTT();

#endif
