#include "sensor_bh1750.h"
#include <BH1750.h>

BH1750 lightMeter(0x23);

void setupBH1750() {
    lightMeter.begin();
}

uint16_t readLightIntensity() {
    return lightMeter.readLightLevel();
}
