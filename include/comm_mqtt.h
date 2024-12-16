#include <WiFi.h>
#include <PubSubClient.h> // MQTT Library

// Wi-Fi and MQTT Broker credentials
const char* ssid = "Your_SSID";
const char* password = "Your_PASSWORD";
const char* mqtt_server = "broker.hivemq.com"; 

WiFiClient espClient;
PubSubClient client(espClient);

// Sleep durations
#define DEEP_SLEEP_DURATION 60000000ULL // 60 seconds in microseconds
#define ACTIVE_MODE_DURATION 5000 // Active for 5 seconds

// Function to connect to Wi-Fi
void setupWiFi() {
    if (WiFi.status() == WL_CONNECTED) return;
    
    Serial.print("Connecting to Wi-Fi");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected to Wi-Fi");
}

// Function to connect to the MQTT broker
void reconnectMQTT() {
    while (!client.connected()) {
        Serial.print("Connecting to MQTT...");
        if (client.connect("ESP32_Client")) {
            Serial.println("Connected!");
        } else {
            Serial.print("Failed, rc=");
            Serial.print(client.state());
            delay(2000);
        }
    }
}

// Function to publish data to MQTT broker
void publishSensorData(String sensorData) {
    if (!client.connected()) reconnectMQTT();
    client.publish("SSAS/SensorData", sensorData.c_str());
    Serial.println("Data sent: " + sensorData);
}

// Optimized communication protocol logic
void sendData() {
    // Collect sensor data (dummy example)
    String sensorData = "{\"temp\":24.5,\"humidity\":60,\"soil_moisture\":30}";
    
    // Publish data to MQTT broker
    publishSensorData(sensorData);
    
    // Disconnect Wi-Fi after sending data to save power
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    Serial.println("Wi-Fi turned off to save power.");
}

void setup() {
    Serial.begin(115200);
    setupWiFi();
    client.setServer(mqtt_server, 1883);
    sendData();  // Send data immediately upon waking up
    
    // Enter deep sleep after sending data
    Serial.println("Entering deep sleep...");
    esp_deep_sleep_start();
}

void loop() {
    // Keep the loop empty as the system wakes up from deep sleep for operations
}
