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

// FreeRTOS includes
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define DHTPIN 4
#define CO2_RX_PIN 16
#define CO2_TX_PIN 17

BH1750 lightMeter(0x23);
DHT dht(DHTPIN, DHT22);
SoftwareSerial mySerial(CO2_RX_PIN, CO2_TX_PIN);
TinyGPSPlus gps;

WiFiClient espClient;
PubSubClient client(espClient);

// Function prototypes
float readCO2();
float readWindSpeed();
String createCSVPayload(float temperature, float humidity, uint16_t lightIntensity, float co2, float windSpeed);

void setup() {
    Serial.begin(115200);
    lightMeter.begin();
    dht.begin();
    mySerial.begin(9600);
    setupWiFi();
    setupMQTT("broker.hivemq.com");
    setupPowerManagement();
    client.setServer(MQTT_SERVER, MQTT_PORT);

    // Task creation for FreeRTOS
    xTaskCreate(sensorTask, "Sensor Task", 2048, NULL, 1, NULL);
    xTaskCreate(powerManagementTask, "Power Management Task", 2048, NULL, 1, NULL);
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();
}

// Sensor task for FreeRTOS
void sensorTask(void *pvParameters) {
    while (true) {
        float temperature = dht.readTemperature();
        float humidity = dht.readHumidity();
        uint16_t lightIntensity = lightMeter.readLightLevel();
        float co2 = readCO2();
        float windSpeed = readWindSpeed();

        // Create a CSV payload
        String payload = createCSVPayload(temperature, humidity, lightIntensity, co2, windSpeed);
        publishData("SSAS/sensors", payload);

        // Task delay (10 seconds)
        vTaskDelay(10000 / portTICK_PERIOD_MS);
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

// Create CSV payload
String createCSVPayload(float temperature, float humidity, uint16_t lightIntensity, float co2, float windSpeed) {
    String csv = "timestamp,temperature,humidity,lightIntensity,co2,windSpeed\n";
    csv += String(millis()) + "," + String(temperature) + "," + String(humidity) + "," +
           String(lightIntensity) + "," + String(co2) + "," + String(windSpeed);
    return csv;
}
