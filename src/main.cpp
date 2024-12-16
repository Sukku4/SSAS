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

void setup() {
    Serial.begin(115200);
    lightMeter.begin();
    dht.begin();
    mySerial.begin(9600);
    setupWiFi();
    Serial.begin(115200);
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
     // Aggregate sensor data for batch transmission
   String aggregatedData = aggregateSensorData();
   publishData("SSAS/sensors", aggregatedData);

  // Enter deep sleep to conserve power
    enterDeepSleep(600); // Sleep for 10 minutes
}

void sensorTask(void *pvParameters) {
    while (true) {
        float temperature = dht.readTemperature();
        float humidity = dht.readHumidity();
        uint16_t lightIntensity = lightMeter.readLightLevel();
        float co2 = readCO2();
        float windSpeed = readWindSpeed();

        String payload = createPayload(temperature, humidity, lightIntensity, co2, windSpeed);
        sendToCloud(payload);

        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

float readCO2() {
    // Implement CO2 sensor reading
    return 400.0; // Placeholder value
}

float readWindSpeed() {
    // Implement wind speed sensor reading
    return 5.0; // Placeholder value
}

String createPayload(float temperature, float humidity, uint16_t lightIntensity, float co2, float windSpeed) {
    // Create JSON payload
    String payload = "{";
    payload += "\"temperature\":" + String(temperature) + ",";
    payload += "\"humidity\":" + String(humidity) + ",";
    payload += "\"lightIntensity\":" + String(lightIntensity) + ",";
    payload += "\"co2\":" + String(co2) + ",";
    payload += "\"windSpeed\":" + String(windSpeed);
    payload += "}";
    return payload;
}
