#include "sensor_dht22.h"
#include <DHT.h>

#define DHTPIN GPIO_NUM_4
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);

void setupDHT() {
    dht.begin();
}

float readTemperature() {
    return dht.readTemperature();
}

float readHumidity() {
    return dht.readHumidity();
}
