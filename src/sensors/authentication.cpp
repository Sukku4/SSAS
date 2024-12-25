#include "security/authentication.h"
#include "utils/encryption.h"

bool AuthenticationManager::initialize() {
    // Load user credentials, set up initial admin
    users.push_back({
        "admin", 
        EncryptionUtil::hashPassword("admin123"),
        UserRole::ADMIN
    });
    return true;
}

bool AuthenticationManager::authenticate(
    const String& username , const String& password) {
    for (const auto& user : users) {
        if (user.username == username && 
            EncryptionUtil::verifyPassword(password, user.passwordHash)) {
            currentUser  = user;
            return true;
        }
    }
    return false;
}

bool AuthenticationManager::isAuthenticated() {
    return !currentUser .username.isEmpty();
}

void AuthenticationManager::logout() {
    currentUser  = User();
}
