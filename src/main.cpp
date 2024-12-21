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
#define CO2_RX_PIN 16
#define CO2_TX_PIN 17
#define HEATER_PIN 12
#define FAN_PIN 13
#define HUMIDIFIER_PIN 14
#define OVERRIDE_SWITCH_PIN 15    // Manual override switch
#define MANUAL_HEATER_PIN 16      // Manual heater control
#define MANUAL_FAN_PIN 17         // Manual fan control
#define MANUAL_HUMIDIFIER_PIN 18  // Manual humidifier control

// Set Points
float targetTemperature = 25.0; // °C
float targetHumidity = 60.0;    // %

// System States
bool isManualMode = false; // Flag for manual override mode

// Global Objects
BH1750 lightMeter(0x23);
DHT dht(DHTPIN, DHT22);
SoftwareSerial mySerial(CO2_RX_PIN, CO2_TX_PIN);
TinyGPSPlus gps;

WiFiClient espClient;
PubSubClient client(espClient);

// PID Controllers
PIDController tempPID(2.0, 0.5, 1.0); // Tuned PID coefficients for temperature
PIDController humidityPID(2.0, 0.5, 1.0); // Tuned PID coefficients for humidity

// Function prototypes
float readCO2();
float readWindSpeed();
String createCSVPayload(float temperature, float humidity, uint16_t lightIntensity, float co2, float windSpeed);
void sensorTask(void *pvParameters);
void controlTask(void *pvParameters);
void feedbackTask(void *pvParameters);
void manualControlTask(void *pvParameters);
void makeDecisions(float temperature, float humidity, float lightIntensity, float co2, float windSpeed);
void checkManualOverride();

void setup() {
    Serial.begin(115200);

    // Initialize sensors
    lightMeter.begin();
    dht.begin();
    mySerial.begin(9600);

    // Initialize actuators
    pinMode(HEATER_PIN, OUTPUT);
    pinMode(FAN_PIN, OUTPUT);
    pinMode(HUMIDIFIER_PIN, OUTPUT);

    // Initialize manual control pins
    pinMode(MANUAL_HEATER_PIN, INPUT_PULLUP);
    pinMode(MANUAL_FAN_PIN, INPUT_PULLUP);
    pinMode(MANUAL_HUMIDIFIER_PIN, INPUT_PULLUP);
    pinMode(OVERRIDE_SWITCH_PIN, INPUT_PULLUP);

    // Initialize Wi-Fi and MQTT
    setupWiFi();
    setupMQTT("broker.hivemq.com");

    // Initialize power management
    setupPowerManagement();

    // Set up interrupt
    setupInterrupt(INTERRUPT_PIN);

    // Create FreeRTOS tasks
    xTaskCreate(sensorTask, "Sensor Task", 4096, NULL, 1, NULL);
    xTaskCreate(controlTask, "Control Task", 4096, NULL, 1, NULL);
    xTaskCreate(feedbackTask, "Feedback Task", 2048, NULL, 1, NULL);
    xTaskCreate(manualControlTask, "Manual Control Task", 2048, NULL, 1, NULL);
    xTaskCreate(powerManagementTask, "Power Management Task", 2048, NULL, 1, NULL);
}

void loop() {
    // Periodically check the state of the manual override switch
    checkManualOverride();

    // Maintain MQTT connection
    if (!client.connected()) {
        reconnect();
    }
    client.loop();
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
void controlTask(void *pvParameters) {
    while (true) {
        if (!isManualMode) {
            // Automated control logic
            float temperature = dht.readTemperature();
            float humidity = dht.readHumidity();
            float lightIntensity = lightMeter.readLightLevel();
            float co2 = readCO2();
            float windSpeed = readWindSpeed();

            // Make decisions based on sensor data
            makeDecisions(temperature, humidity, lightIntensity, co2, windSpeed);

            Serial.println("Automated control in progress...");
        }

        vTaskDelay(2000 / portTICK_PERIOD_MS); // Automated control every 2 seconds
    }
}

// Feedback task for dynamic set point adjustment
void feedbackTask(void *pvParameters) {
    while (true) {
        float currentTemperature = dht.readTemperature();
        float currentHumidity = dht.readHumidity();

        // Adjust set points dynamically based on feedback
        if (currentTemperature > targetTemperature + 2) {
            targetTemperature -= 0.5; // Gradually lower the set point
        } else if (currentTemperature < targetTemperature - 2) {
            targetTemperature += 0.5; // Gradually increase the set point
        }

        if (currentHumidity > targetHumidity + 5) {
            targetHumidity -= 1.0;
        } else if (currentHumidity < targetHumidity - 5) {
            targetHumidity += 1.0;
        }

        Serial.print("Adjusted Target Temperature: ");
        Serial.println(targetTemperature);
        Serial.print("Adjusted Target Humidity: ");
        Serial.println(targetHumidity);

        vTaskDelay(5000 / portTICK_PERIOD_MS); // Feedback check every 5 seconds
    }
}

// Decision-making function for actuator control
void makeDecisions(float temperature, float humidity, float lightIntensity, float co2, float windSpeed) {
    // Temperature Control
    if (temperature > 30.0) {
        digitalWrite(FAN_PIN, HIGH);    // Activate fan
        digitalWrite(HEATER_PIN, LOW); // Deactivate heater
    } else if (temperature < 20.0) {
        digitalWrite(FAN_PIN, LOW);    // Deactivate fan
        digitalWrite(HEATER_PIN, HIGH); // Activate heater
    } else {
        digitalWrite(FAN_PIN, LOW);
        digitalWrite(HEATER_PIN, LOW);
    }

    // Humidity Control
    if (humidity < 50.0) {
        digitalWrite(HUMIDIFIER_PIN, HIGH); // Activate humidifier
    } else if (humidity > 70.0) {
        digitalWrite(HUMIDIFIER_PIN, LOW); // Deactivate humidifier
    }
}

// Placeholder for CO2 sensor
float readCO2() {
    return 400.0; // Simulated value
}

// Placeholder for wind speed sensor
float readWindSpeed() {
    return 5.0; // Simulated value
}

// Create CSV payload for transmission
String createCSVPayload(float temperature, float humidity, uint16_t lightIntensity, float co2, float windSpeed) {
    String csv = "timestamp,temperature,humidity,lightIntensity,co2,windSpeed\n";
    csv += String(millis()) + "," + String(temperature) + "," + String(humidity) + "," +
           String(lightIntensity) + "," + String(co2) + "," + String(windSpeed);
    return csv;
}
