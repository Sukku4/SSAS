#include "sensor_bh1750.h"
#include <Wire.h>
#include <BH1750.h>

BH1750 lightMeter;

void initBH1750() {
    Wire.begin();
    lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
}

float getLightIntensity() {
    return lightMeter.readLightLevel();
}
