#ifndef SENSOR_DHT22_H
#define SENSOR_DHT22_H

#define DHTPIN 4

void setupDHT();
float readTemperature();
float readHumidity();

#endif // SENSOR_DHT22_H
