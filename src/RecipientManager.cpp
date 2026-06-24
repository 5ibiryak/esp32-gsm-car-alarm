#include "RecipientManager.h"
#include <Preferences.h>
#include <string.h>

namespace {
constexpr const char* kNvsNamespace = "sms";
constexpr const char* kKeyCount     = "rcpt_count";
}

// ---- Validation / normalization -------------------------------------------

String RecipientManager::normalizePhoneNumber(const String& raw) {
    // Strip spaces, CR, LF, tabs. Keep leading '+' and digits.
    String s;
    s.reserve(raw.length());
    for (size_t i = 0; i < raw.length(); ++i) {
        const char c = raw.charAt(i);
        if (c == ' ' || c == '\r' || c == '\n' || c == '\t') continue;
        s += c;
    }
    if (s.length() == 0) return s;

    // Russian convenience: 8XXXXXXXXXX -> +7XXXXXXXXXX, 7XXXXXXXXXX -> +7XXX…
    if (s.length() == 11 && s.charAt(0) == '8') {
        s = String("+7") + s.substring(1);
    } else if (s.length() == 11 && s.charAt(0) == '7') {
        s = String('+') + s;
    }
    return s;
}

bool RecipientManager::isValidPhoneNumber(const String& phone) {
    if (phone.length() < 8 || phone.length() > 16) return false;
    size_t i = 0;
    if (phone.charAt(0) == '+') {
        if (phone.length() == 1) return false;
        i = 1;
    }
    for (; i < phone.length(); ++i) {
        if (!isDigit(phone.charAt(i))) return false;
    }
    return true;
}

bool RecipientManager::phonesMatch(const char* a, const char* b) {
    if (!a || !b) return false;
    if (strcmp(a, b) == 0) return true;
    const size_t la = strlen(a);
    const size_t lb = strlen(b);
    if (la >= 10 && lb >= 10) {
        return strcmp(a + la - 10, b + lb - 10) == 0;
    }
    return false;
}

// ---- Storage primitives ---------------------------------------------------

void RecipientManager::storeAt(size_t slot, const String& phone) {
    if (slot >= kMax) return;
    const size_t cap = kPhoneLen - 1;
    const size_t n   = phone.length() < cap ? phone.length() : cap;
    memset(_recipients[slot], 0, kPhoneLen);
    memcpy(_recipients[slot], phone.c_str(), n);
}

bool RecipientManager::contains(const String& raw) const {
    const String phone = normalizePhoneNumber(raw);
    if (phone.length() == 0) return false;
    for (size_t i = 0; i < _count; ++i) {
        if (phonesMatch(_recipients[i], phone.c_str())) return true;
    }
    return false;
}

bool RecipientManager::add(const String& raw) {
    const String phone = normalizePhoneNumber(raw);
    if (!isValidPhoneNumber(phone)) return false;
    if (contains(phone))            return false;
    if (_count >= kMax)             return false;
    storeAt(_count, phone);
    ++_count;
    return true;
}

bool RecipientManager::remove(const String& raw) {
    const String phone = normalizePhoneNumber(raw);
    if (phone.length() == 0) return false;
    for (size_t i = 0; i < _count; ++i) {
        if (!phonesMatch(_recipients[i], phone.c_str())) continue;
        for (size_t j = i + 1; j < _count; ++j) {
            memcpy(_recipients[j - 1], _recipients[j], kPhoneLen);
        }
        --_count;
        memset(_recipients[_count], 0, kPhoneLen);
        return true;
    }
    return false;
}

// ---- Rendering ------------------------------------------------------------

String RecipientManager::listText() const {
    String r = "MoskvichAlarm RECIPIENTS:";
    if (_count == 0) {
        r += "\n(empty)";
        return r;
    }
    for (size_t i = 0; i < _count; ++i) {
        r += "\n";
        r += String((unsigned long)(i + 1));
        r += ") ";
        r += _recipients[i];
    }
    return r;
}

String RecipientManager::compactListText() const {
    String r = "Recipients:";
    if (_count == 0) {
        r += "\n(empty)";
        return r;
    }
    for (size_t i = 0; i < _count; ++i) {
        r += "\n";
        r += String((unsigned long)(i + 1));
        r += " ";
        r += _recipients[i];
    }
    return r;
}

// ---- NVS persistence ------------------------------------------------------

bool RecipientManager::loadDefaults() {
    _count = 0;
    for (size_t i = 0; i < SMS_ALERT_RECIPIENT_DEFAULT_COUNT && _count < kMax; ++i) {
        const String phone = normalizePhoneNumber(String(SMS_ALERT_RECIPIENTS_DEFAULT[i]));
        if (isValidPhoneNumber(phone)) {
            storeAt(_count, phone);
            ++_count;
        }
    }
    return true;
}

bool RecipientManager::loadFromNvsOrDefaults() {
    Preferences prefs;
    if (!prefs.begin(kNvsNamespace, true)) {
        loadDefaults();
        return false;        // signal: NVS unavailable, defaults used
    }

    const uint32_t stored = prefs.getUInt(kKeyCount, 0xFFFFFFFFu);
    if (stored == 0xFFFFFFFFu || stored > kMax) {
        prefs.end();
        loadDefaults();
        return false;
    }

    _count = 0;
    for (uint32_t i = 0; i < stored && _count < kMax; ++i) {
        char key[8];
        snprintf(key, sizeof(key), "rcpt%lu", (unsigned long)i);
        const String phone = normalizePhoneNumber(prefs.getString(key, String("")));
        if (isValidPhoneNumber(phone)) {
            storeAt(_count, phone);
            ++_count;
        }
    }
    prefs.end();

    if (_count == 0) {
        loadDefaults();
    }
    return true;
}

bool RecipientManager::saveToNvs() {
    Preferences prefs;
    if (!prefs.begin(kNvsNamespace, false)) return false;

    prefs.putUInt(kKeyCount, (uint32_t)_count);
    for (size_t i = 0; i < _count; ++i) {
        char key[8];
        snprintf(key, sizeof(key), "rcpt%lu", (unsigned long)i);
        prefs.putString(key, _recipients[i]);
    }
    // Wipe any stale slots beyond the current count.
    for (size_t i = _count; i < kMax; ++i) {
        char key[8];
        snprintf(key, sizeof(key), "rcpt%lu", (unsigned long)i);
        prefs.remove(key);
    }
    prefs.end();
    return true;
}

void RecipientManager::begin() {
    const bool nvsOk = loadFromNvsOrDefaults();
    if (!nvsOk) {
        Serial.println(F("[SMS] recipient NVS load failed, using defaults"));
    }

    Serial.print  (F("[SMS] alert recipient max="));
    Serial.println(kMax);
    Serial.print  (F("[SMS] alert recipients loaded: "));
    Serial.println(_count);
    for (size_t i = 0; i < _count; ++i) {
        Serial.print  (F("[SMS] recipient "));
        Serial.print  ((unsigned long)(i + 1));
        Serial.print  (F(": "));
        Serial.println(_recipients[i]);
    }
}
