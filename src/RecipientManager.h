#pragma once
#include <Arduino.h>
#include "config.h"

// Runtime, NVS-backed list of SMS alert recipients.
//
// Editable at runtime by whitelisted SMS senders via ADD / DEL / LIST.
// Persists to NVS namespace "sms" so changes survive reboot.
// SMS_COMMAND_WHITELIST is intentionally compile-time only.
class RecipientManager {
public:
    static constexpr size_t kMax      = SMS_ALERT_RECIPIENT_MAX;
    static constexpr size_t kPhoneLen = 20;     // +<15 digits>\0 fits with room to spare

    void   begin();                              // loads from NVS or defaults

    size_t      count() const { return _count; }
    const char* at(size_t i) const { return (i < _count) ? _recipients[i] : ""; }

    bool   contains(const String& phone) const;
    bool   add(const String& phone);             // false on invalid / duplicate / full
    bool   remove(const String& phone);          // false on not-found / would-empty

    String listText()        const;              // "MoskvichAlarm RECIPIENTS:\n1) +...\n..."
    String compactListText() const;              // "Recipients:\n1 +...\n..."

    // Public helpers / class invariants.
    static String normalizePhoneNumber(const String& raw);
    static bool   isValidPhoneNumber(const String& phone);
    static bool   phonesMatch(const char* a, const char* b);

    // Persistence — separate so callers can react to write failures.
    bool   loadFromNvsOrDefaults();              // returns true on any load (defaults count)
    bool   saveToNvs();                          // returns false on NVS error

private:
    bool   loadDefaults();
    void   storeAt(size_t slot, const String& phone);

    char   _recipients[kMax][kPhoneLen] = {{0}};
    size_t _count = 0;
};
