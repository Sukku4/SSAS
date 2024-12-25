#include "sensor_light.h"
#include <Wire.h>
#include <BH1750.h>

BH1750 lightMeter;

void setupLightSensor() {
    Wire.begin();
    lightMeter.begin();
}

float readLightIntensity() {
    return lightMeter.readLightLevel();
}
