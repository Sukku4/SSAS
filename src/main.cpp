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
#define INTERRUPT_PIN 2

// Set Points
const float targetTemperature = 25.0; // °C
const float targetHumidity = 60.0;    // %

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
    xTaskCreate(powerManagementTask, "Power Management Task", 2048, NULL, 1, NULL);
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();
}

// Sensor data acquisition and aggregation task
void sensorTask(void *pvParameters) {
    while (true) {
        float temperature = dht.readTemperature();
        float humidity = dht.readHumidity();
        uint16_t lightIntensity = lightMeter.readLightLevel();
        float co2 = readCO2();
        float windSpeed = readWindSpeed();

        // Filter and calibrate data
        temperature = filterData(temperature);
        humidity = filterData(humidity);

        // Create CSV payload
        String payload = createCSVPayload(temperature, humidity, lightIntensity, co2, windSpeed);
        publishData("SSAS/sensors", payload);

        // Delay to prevent frequent reads
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

// Closed-loop control task
void controlTask(void *pvParameters) {
    while (true) {
        float currentTemperature = dht.readTemperature();
        float currentHumidity = dht.readHumidity();

        // Temperature control
        float tempError = targetTemperature - currentTemperature;
        float tempAdjustment = tempPID.calculate(tempError);

        if (tempAdjustment > 0) {
            digitalWrite(HEATER_PIN, HIGH); // Turn on heater
            digitalWrite(FAN_PIN, LOW);    // Turn off fan
        } else if (tempAdjustment < 0) {
            digitalWrite(HEATER_PIN, LOW); // Turn off heater
            digitalWrite(FAN_PIN, HIGH);  // Turn on fan
        } else {
            digitalWrite(HEATER_PIN, LOW);
            digitalWrite(FAN_PIN, LOW);
        }

        // Humidity control
        float humidityError = targetHumidity - currentHumidity;
        float humidityAdjustment = humidityPID.calculate(humidityError);

        if (humidityAdjustment > 0) {
            digitalWrite(HUMIDIFIER_PIN, HIGH); // Turn on humidifier
        } else {
            digitalWrite(HUMIDIFIER_PIN, LOW);  // Turn off humidifier
        }

        // Delay for control loop
        vTaskDelay(2000 / portTICK_PERIOD_MS);
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
