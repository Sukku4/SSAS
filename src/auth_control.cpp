#include "auth_control.h"
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>

// Dummy credentials for testing
const char* validUsername = "admin";
const char* validPassword = "password123";

bool authenticateUser(const char* username, const char* password) {
    return (strcmp(username, validUsername) == 0 && strcmp(password, validPassword) == 0);
}
