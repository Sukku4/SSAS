#include "sensor_dht22.h"
#include <DHT.h>

#define DHT_PIN 4
#define DHT_TYPE DHT22

DHT dht(DHT_PIN, DHT_TYPE);

void initDHT22() {
    dht.begin();
}

float getTemperature() {
    return dht.readTemperature();
}

float getHumidity() {
    return dht.readHumidity();
}
