#include <Arduino.h>
#include "communication/wifi_manager.h"
#include "communication/mqtt_handler.h"
#include "sensors/dht22_sensor.h"
#include "sensors/soil_moisture_sensor.h"
#include "actuators/heater.h"
#include "actuators/cooler.h"
#include "control/pid_controller.h"
#include "security/authentication.h"
#include "power/solar_management.h"
#include "utils/data_logger.h"

// System Configuration
struct SystemConfig {
    float targetTemperature = 25.0;
    float targetHumidity = 60.0;
    float targetSoilMoisture = 50.0;
} config;

// Global Instances
WiFiManager wifiManager;
MQTTHandler mqttHandler;
DHT22Sensor temperatureSensor;
SoilMoistureSensor soilSensor;
HeaterActuator heater;
CoolerActuator cooler;
PIDController pidController;
AuthenticationManager authManager;
SolarManagement solarSystem;
DataLogger dataLogger;

void setup() {
    Serial.begin(115200);
    
    // Initialize Subsystems
    wifiManager.connect();
    mqttHandler.setup();
    temperatureSensor.begin();
    soilSensor.begin();
    heater.initialize();
    cooler.initialize();
    
    // Authentication Setup
    authManager.initialize();
    
    // Power Management
    solarSystem.begin();
    
    // Data Logging
    dataLogger.start();
}

void loop() {
    // Read Sensors
    float temperature = temperatureSensor.readTemperature();
    float humidity = temperatureSensor.readHumidity();
    float soilMoisture = soilSensor.readMoisture();
    
    // Authentication Check
    if (!authManager.isAuthenticated()) {
        mqttHandler.publishAlert("SYSTEM: Unauthorized Access Attempt");
        return;
    }
    
    // PID Control
    float heaterOutput = pidController.computeHeaterOutput(
        temperature, config.targetTemperature
    );
    float coolerOutput = pidController.computeCoolerOutput(
        temperature, config.targetTemperature
    );
    
    // Actuator Control
    heater.setIntensity(heaterOutput);
    cooler.setIntensity(coolerOutput);
    
    // Power Management
    solarSystem.monitor();
    
    // Data Logging and MQTT Publishing
    mqttHandler.publishSensorData({
        {"temperature", temperature},
        {"humidity", humidity},
        {"soil_moisture", soilMoisture}
    });
    
    dataLogger.logData({
        {"temperature", temperature},
        {"humidity", humidity},
        {"soil_moisture", soilMoisture}
    });
    
    delay(5000); // 5-second interval
}
