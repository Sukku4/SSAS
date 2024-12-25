#include "utils/encryption.h"
#include <SHA256.h>

String EncryptionUtil::hashPassword(const String& password) {
    SHA256 sha256;
    sha256.update(password.c_str(), password.length());
    return sha256.finalize().toString();
}

bool EncryptionUtil::verifyPassword(const String& password, const String& hash) {
    return hashPassword(password) == hash;
}
