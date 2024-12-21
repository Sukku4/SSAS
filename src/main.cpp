#include <Arduino.h>
#include <BH1750.h>
#include <DHT.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <PubSubClient.h>

#include "sensor_dht22.h"
#include "sensor_bh1750.h"
#include "comm_mqtt.h"
#include "power_management.h"
#include "interrupt_handler.h"
#include "data_filter.h"
#include "pid_control.h"

// FreeRTOS includes
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Pin Definitions
#define DHTPIN 4
#define HEATER_PIN 12
#define FAN_PIN 13
#define HUMIDIFIER_PIN 14
#define OVERRIDE_SWITCH_PIN 15 // Manual override switch
#define MANUAL_HEATER_PIN 16   // Manual heater control
#define MANUAL_FAN_PIN 17      // Manual fan control
#define MANUAL_HUMIDIFIER_PIN 18 // Manual humidifier control

// System States
bool isManualMode = false; // Flag for manual override mode

// Function prototypes
void checkManualOverride();
void manualControlTask(void *pvParameters);
void automatedControlTask(void *pvParameters);

void setup() {
    Serial.begin(115200);

    // Initialize sensors
    dht.begin();

    // Initialize actuators
    pinMode(HEATER_PIN, OUTPUT);
    pinMode(FAN_PIN, OUTPUT);
    pinMode(HUMIDIFIER_PIN, OUTPUT);

    // Initialize manual control pins
    pinMode(MANUAL_HEATER_PIN, INPUT_PULLUP);
    pinMode(MANUAL_FAN_PIN, INPUT_PULLUP);
    pinMode(MANUAL_HUMIDIFIER_PIN, INPUT_PULLUP);
    pinMode(OVERRIDE_SWITCH_PIN, INPUT_PULLUP);

    // Create FreeRTOS tasks
    xTaskCreate(manualControlTask, "Manual Control Task", 2048, NULL, 1, NULL);
    xTaskCreate(automatedControlTask, "Automated Control Task", 4096, NULL, 1, NULL);
}

void loop() {
    // Periodically check the state of the manual override switch
    checkManualOverride();
}

// Check if the system should switch to manual mode
void checkManualOverride() {
    if (digitalRead(OVERRIDE_SWITCH_PIN) == LOW) {
        isManualMode = true;
        Serial.println("Manual mode activated");
    } else {
        isManualMode = false;
        Serial.println("Automated mode activated");
    }
}

// Task for handling manual controls
void manualControlTask(void *pvParameters) {
    while (true) {
        if (isManualMode) {
            // Read manual inputs
            if (digitalRead(MANUAL_HEATER_PIN) == LOW) {
                digitalWrite(HEATER_PIN, HIGH); // Turn on heater
            } else {
                digitalWrite(HEATER_PIN, LOW); // Turn off heater
            }

            if (digitalRead(MANUAL_FAN_PIN) == LOW) {
                digitalWrite(FAN_PIN, HIGH); // Turn on fan
            } else {
                digitalWrite(FAN_PIN, LOW); // Turn off fan
            }

            if (digitalRead(MANUAL_HUMIDIFIER_PIN) == LOW) {
                digitalWrite(HUMIDIFIER_PIN, HIGH); // Turn on humidifier
            } else {
                digitalWrite(HUMIDIFIER_PIN, LOW); // Turn off humidifier
            }

            Serial.println("Manual control in progress...");
        }

        vTaskDelay(500 / portTICK_PERIOD_MS); // Check inputs every 500ms
    }
}

// Task for handling automated controls
void automatedControlTask(void *pvParameters) {
    while (true) {
        if (!isManualMode) {
            // Automated control logic
            float temperature = dht.readTemperature();
            float humidity = dht.readHumidity();

            // Example: Automated control logic for heater and fan
            if (temperature < 20.0) {
                digitalWrite(HEATER_PIN, HIGH);
                digitalWrite(FAN_PIN, LOW);
            } else if (temperature > 30.0) {
                digitalWrite(HEATER_PIN, LOW);
                digitalWrite(FAN_PIN, HIGH);
            } else {
                digitalWrite(HEATER_PIN, LOW);
                digitalWrite(FAN_PIN, LOW);
            }

            // Example: Automated control logic for humidifier
            if (humidity < 50.0) {
                digitalWrite(HUMIDIFIER_PIN, HIGH);
            } else {
                digitalWrite(HUMIDIFIER_PIN, LOW);
            }

            Serial.println("Automated control in progress...");
        }

        vTaskDelay(2000 / portTICK_PERIOD_MS); // Automated control every 2 seconds
    }
}
