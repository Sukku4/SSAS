#include "decision_making.h"

void setupDecisionMaking() {
    // Initialize decision-making logic
}

void processSensorData(float temperature, float humidity, float lightIntensity) {
    // Decision-making based on sensor thresholds
    if (temperature > 35.0) {
        Serial.println("Warning: High temperature detected!");
    }
    if (lightIntensity < 200.0) {
        Serial.println("Low light detected. Adjusting settings...");
    }
}
