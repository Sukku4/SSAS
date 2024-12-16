#include "power_management.h"
#include <esp_sleep.h>

void setupPowerManagement() {
  Serial.println("Power management initialized");
}

void enterDeepSleep(uint64_t timeInSeconds) {
  Serial.printf("Entering deep sleep for %llu seconds...\n", timeInSeconds);
  esp_sleep_enable_timer_wakeup(timeInSeconds * 1000000); // Convert to microseconds
  esp_deep_sleep_start();
}
