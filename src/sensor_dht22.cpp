#include "sensor_dht22.h"
#include <DHT.h>

DHT dht(DHTPIN, DHT22);

void setupDHT() {
    dht.begin();
}

float readTemperature() {
    return dht.readTemperature();
}

float readHumidity() {
    return dht.readHumidity();
}
