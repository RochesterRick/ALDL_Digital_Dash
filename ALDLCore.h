#pragma once

#include <Arduino.h>
#include <Preferences.h>

// ------------------------------------------------------------
// DataField definition
// ------------------------------------------------------------
struct DataField {
  const char* id;            // persistent unique key
  const char* name;          // display name
  const char* symbol;        // unit symbol
  uint16_t adsByteNumber;    // ADS btByteNumber (1-based)
  uint8_t size;              // 1 or 2 bytes
  float scale;               // multiplier
  float offsetValue;         // add after scaling
  uint8_t precision;         // decimals for display
};

// ------------------------------------------------------------
// Persistent ID storage for 6 slots
// ------------------------------------------------------------
class PersistentIDs {
public:
  void begin() {
    prefs.begin("aldl_ids", false);

    for (int i = 0; i < SLOT_COUNT; i++) {
      String key = String("slot") + i;
      String v = prefs.getString(key.c_str(), defaultID(i));
      ids[i] = v;
    }
  }

  const char* getID(int slot) const {
    if (slot < 0 || slot >= SLOT_COUNT) return "";
    return ids[slot].c_str();
  }

  void setID(int slot, const char* newID) {
    if (slot < 0 || slot >= SLOT_COUNT || !newID) return;
    ids[slot] = newID;
  }

  void save() {
    for (int i = 0; i < SLOT_COUNT; i++) {
      String key = String("slot") + i;
      prefs.putString(key.c_str(), ids[i]);
    }
  }

private:
  static constexpr int SLOT_COUNT = 6;
  Preferences prefs;
  String ids[SLOT_COUNT];

  const char* defaultID(int slot) const {
    switch (slot) {
      case 0: return "CMDF4_ENGINE_SPEED";
      case 1: return "CMDF4_MPH";
      case 2: return "CMDF4_TPS";
      case 3: return "CMDF4_COOLANT_TEMP";
      case 4: return "CMDF4_BATTERY_VOLTS";
      case 5: return "CMDF4_MPG";
      default: return "CMDF4_ENGINE_SPEED";
    }
  }
};

// ------------------------------------------------------------
// Helper: find field by persistent ID
// ------------------------------------------------------------
inline const DataField* findFieldByID(const char* id, const DataField* fields, size_t count) {
  if (!id || !fields) return nullptr;
  for (size_t i = 0; i < count; i++) {
    if (strcmp(id, fields[i].id) == 0) return &fields[i];
  }
  return nullptr;
}

// ------------------------------------------------------------
// F4 field list (ADS matched, 1-based btByteNumber values)
// ------------------------------------------------------------
static const DataField COMMAND1_FIELDS[] = {
  { "CMDF4_ENGINE_SPEED",   "RPM",             "rpm",  21, 2, 0.125f, 0.0f, 0 },
  { "CMDF4_MPH",            "MPH",             "mph",  26, 1, 1.0f,   0.0f, 0 },
  { "CMDF4_TPS",            "T.P.S.",          "%",    17, 1, 0.392f, 0.0f, 0 },
  { "CMDF4_COOLANT_TEMP",   "Coolant Temp",    "°F",   65, 1, 1.35f, -40.0f, 0 },
  { "CMDF4_BATTERY_VOLTS",  "Battery",         "V",    33, 1, 0.1f,   0.0f, 1 },
  { "CMDF4_TRANS_TEMP",     "Trans Temp",      "°F",   66, 1, 1.35f, -40.0f, 0 },
  { "CMDF4_CURRENT_GEAR",   "Current Gear",    "",     38, 1, 1.0f,   1.0f, 0 },
  { "CMDF4_TCC_PWM",        "TCC PWM",         "%",    28, 1, 0.392f, 0.0f, 0 },
  { "CMDF4_BPW",            "Base Pulse",      "ms",   11, 1, 0.01f,  0.0f, 2 },
  { "CMDF4_SPARK_ADV",      "Spark Advance",   "°",    43, 1, 0.3516f,0.0f, 1 },
  { "CMDF4_IAC",            "IAC Counts",      "",     24, 1, 1.0f,   0.0f, 0 },
  { "CMDF4_MAP",            "MAP",             "kPa",  16, 1, 0.5f,   0.0f, 0 },
  { "CMDF4_O2",             "O2 Sensor",       "mV",   7,  1, 4.0f,   0.0f, 0 },
  { "CMDF4_INT",            "Integrator",      "",     9,  1, 1.0f,   0.0f, 0 },
  { "CMDF4_BLM",            "BLM",             "",     10, 1, 1.0f,   0.0f, 0 },
  { "CMDF4_KNOCK",          "Knock Count",     "",     45, 1, 1.0f,   0.0f, 0 },
  { "CMDF4_KNOCK_RETARD",   "Knock Retard",    "°",    46, 1, 0.35f,  0.0f, 1 },
  { "CMDF4_MPG",            "MPG",             "mpg",  0,  0, 0.0f,   0.0f, 1 },
};

static const size_t COMMAND1_FIELD_COUNT = sizeof(COMMAND1_FIELDS) / sizeof(COMMAND1_FIELDS[0]);
