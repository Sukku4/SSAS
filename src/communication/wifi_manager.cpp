#include "communication/wifi_manager.h"

void WiFiManager::connect() {
    WiFi.begin(SSID, PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }
}
