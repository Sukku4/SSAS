#include "sensors/dht22_sensor.h"
#include <DHT.h>

DHT22Sensor::DHT22Sensor() 
    : dht(DHT_PIN, DHT22) {}

void DHT22Sensor::begin() {
    dht.begin();
}

float DHT22Sensor::readTemperature() {
    float temp = dht.readTemperature();
    if (isnan(temp)) {
        // Error handling
        return -999.0;
    }
    return temp;
}

float DHT22Sensor::readHumidity() {
    float humid = dht.readHumidity();
    if (isnan(humid)) {
        // Error handling
        return -1.0;
    }
    return humid;
}
