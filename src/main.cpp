#include <Arduino.h>
#include <WiFi.h>
#include <esp_sleep.h>
#include "sensor_dht22.h"
#include "sensor_light.h"
#include "comm_mqtt.h"
#include "power_management.h"
#include "auth_control.h"
#include "feedback_control.h"
#include "decision_making.h"

// GPIO Definitions
#define OVERRIDE_SWITCH_PIN GPIO_NUM_15

void setup() {
    Serial.begin(115200);

    // Initialize all modules
    setupWiFi();
    setupMQTT("broker.hivemq.com");
    setupPowerManagement();
    setupDHT();
    setupLightSensor();
    setupAuthControl();
    setupFeedbackControl();
    setupDecisionMaking();

    // Set GPIO for manual override
    pinMode(OVERRIDE_SWITCH_PIN, INPUT_PULLUP);

    Serial.println("SSAS System Initialized.");
}

void loop() {
    // Check for manual override
    if (digitalRead(OVERRIDE_SWITCH_PIN) == LOW) {
        Serial.println("Manual override active. Skipping automation tasks.");
        delay(1000);
        return;
    }

    // Perform automated tasks
    float temperature = readTemperature();
    float humidity = readHumidity();
    float lightIntensity = readLightIntensity();

    // Decision-making based on sensor data
    processSensorData(temperature, humidity, lightIntensity);

    // Feedback control
    applyFeedback(temperature, humidity);

    // Transmit data
    String payload = createCSVPayload(temperature, humidity, lightIntensity);
    publishData("SSAS/sensors", payload);

    // Enter deep sleep to conserve power
    enterDeepSleep(60 * 1000000); // 60 seconds
}
