// ALDLCore.h
#ifndef ALDL_CORE_H
#define ALDL_CORE_H

#include <Arduino.h>
#include <EEPROM.h>

// ----------------- DataField definition -----------------
struct DataField {
  const char* id;
  const char* name;
  const char* symbol;
  uint8_t adsByteNumber;   // matches ADS btByteNumber exactly
  uint8_t size;
  float scale;
  float offsetValue;
  uint8_t precision;
};


// ----------------- Command 0 (F5) fields -----------------
const DataField COMMAND0_FIELDS[] = {
  { "CMDF5_COOLANT_TEMP",    "Coolant Temp",        "°F",  0x0E, 1, 1.35,    -40,     0 },
  { "CMDF5_RPM",             "RPM",                 "RPM", 0x14, 2, 0.125,    0,      0 },
  { "CMDF5_BATTERY_VOLT",    "Battery Voltage",     "V",   0x0F, 1, 0.1,      0,      1 },
  { "CMDF5_GEAR_POSITION",   "Gear Position",       "G",   0x25, 1, 1,        1,      0 },
  { "CMDF5_MPH",             "MPH",                 "MPH", 0x23, 1, 0.5,      0,      0 },
  { "CMDF5_TORQUE_PRESSURE", "Torque_Pressure",     "PSI", 0x1B, 1, 1,        0,      0 },
  { "CMDF5_SHIFT1_2",        "Shift1-2",            "s",   0x2F, 1, 0.025,    0,      1 },
  { "CMDF5_SHIFT2_3",        "Shift2-3",            "s",   0x30, 1, 0.025,    0,      1 },
  { "CMDF5_TPS",             "TPS",                 "%",   0x12, 1, 0.392157, 0,      0 },
  { "CMDF5_BAROMETRIC",      "Barometric Pressure", "kPa", 0x13, 1, 0.369,    10.354, 1 },
};

const size_t COMMAND0_FIELD_COUNT = sizeof(COMMAND0_FIELDS) / sizeof(COMMAND0_FIELDS[0]);


// ----------------- Command 1 (F4) fields -----------------
// Updated notes based on real ESP32 raw captures:
// NOTE:
// The byte number below matches the ADS file's btByteNumber exactly.
// ADS numbering is 1-based byte position.
// Raw C/C++ frame indexing is 0-based, so decode with:
//   frame[byteNumber - 1]

const DataField COMMAND1_FIELDS[] = {
  { "CMDF4_MPG",                  "M.P.G.",           "MPG",   0,  0, 1.0,       0.0,      1 },

  { "CMDF4_IAC_POSITION",         "IAC Pos",          "ct",    5,  1, 1.0,       0.0,      0 },
  { "CMDF4_IAC_DESIRED",          "IAC Desired",      "ct",   11,  1, 1.0,       0.0,      0 },

  { "CMDF4_COOLANT_TEMP_C",       "Eng Temp",         "°C",   15,  1, 0.75,    -40.0,      0 },
  { "CMDF4_COOLANT_TEMP_F",       "Eng Temp",         "°F",   15,  1, 1.35,    -40.0,      0 },

  { "CMDF4_BATTERY_VOLT",         "Battery",          "V",    16,  1, 0.1,       0.0,      1 },

  { "CMDF4_TPS_VOLTAGE",          "TPS Voltage",      "V",    17,  1, 0.019608,  0.0,      2 },
  { "CMDF4_TPS_PERCENT",          "T.P.S.",           "%",    42,  1, 0.392157,  0.0,      0 },

  { "CMDF4_MAP_VOLTAGE",          "MAP Voltage",      "V",    18,  1, 0.019608,  0.0,      2 },
  { "CMDF4_MAP_KPA",              "M.A.P.",           "kPa",  18,  1, 0.369,    10.354,    1 },

  { "CMDF4_O2_SENSOR",            "O2 Sensor",        "mV",   19,  1, 4.420,     0.0,      0 },
  { "CMDF4_AFR_EST",              "AFR Est",          "afr",   0,  0, 1.0,       0.0,      1 },

  { "CMDF4_BARO_VOLTAGE",         "Baro Voltage",     "V",    27,  1, 0.019608,  0.0,      2 },
  { "CMDF4_BARO_KPA",             "Barometric",       "kPa",  27,  1, 0.369,    10.354,    1 },
  { "CMDF4_BARO",                 "Baro",             "inHg", 27,  1, 0.2953f,   0.0f,     1 },

  { "CMDF4_MPH",                  "Veh Speed",        "MPH",  31,  1, 1.0,       0.0,      0 },

  { "CMDF4_FUEL_PUMP_VOLT",       "Fuel Pump V",      "V",    33,  1, 0.1,       0.0,      1 },

  { "CMDF4_ENGINE_SPEED",         "RPM",              "RPM",  34,  1, 25.0,      0.0,      0 },
  { "CMDF4_DESIRED_IDLE_SPEED",   "Desired Idle",     "RPM",  41,  1, 12.5,      0.0,      0 },

  { "CMDF4_EGR_DUTY",             "EGR Cycle",        "%",    37,  1, 0.392157,  0.0,      0 },

  { "CMDF4_ENGINE_RUN_TIME",      "Engine Time",      "s",    39,  2, 1.0,       0.0,      0 }, 

  { "CMDF4_SPARK_ADVANCE",        "Spark Advance",    "°",    45,  2, 0.351563,  0.0,      0 },

  { "CMDF4_KNOCK_COUNTER",        "Knock Counter",    "ct",   47,  1, 1.0,       0.0,      0 },

  { "CMDF4_INT",                  "INT",              "",     49,  1, 1.0,       0.0,      0 },

  { "CMDF4_RICH_LEAN_TRANS",      "Rich/Lean Trans",  "",     51,  1, 1.0,       0.0,      0 },

  { "CMDF4_BLM_CELL",             "BLM Cell",         "",     54,  1, 1.0,       0.0,      0 },
  { "CMDF4_BLM",                  "BLM",              "",     55,  1, 1.0,       0.0,      0 },

  { "CMDF4_KNOCK_RETARD",         "Knock Retard",     "°",    56,  1, 0.175781,  0.0,      0 },

  { "CMDF4_BPW",                  "Inject BPW",      "ms",    57,  2, 0.015259,  0.0,      2 },

  { "CMDF4_ACTUAL_EGR_POSITION",  "Act EGR Pos",      "%",    60,  1, 0.392157,  0.0,      0 },

  { "CMDF4_CCP_DUTY",             "CCP Duty Cycle",   "%",    62,  1, 0.392157,  0.0,      0 },

  { "CMDF4_MAT_C",                "M.A.T",            "°C",   63,  1, 0.75,    -40.0,      0 },
  { "CMDF4_MAT_F",                "M.A.T",            "°F",   63,  1, 1.35,    -40.0,      0 },
};

const size_t COMMAND1_FIELD_COUNT = sizeof(COMMAND1_FIELDS) / sizeof(COMMAND1_FIELDS[0]);


// ----------------- Display positions -----------------
struct DisplayCoord {
  int x;
  int y;
};

// Big value positions
const DisplayCoord labelCenters[6] = {
  { 85,  84 },
  { 231, 84 },
  { 390, 84 },
  { 85,  243 },
  { 231, 243 },
  { 390, 243 },
};

// Header label positions
const DisplayCoord labelHeaders[6] = {
  { 85,  34 },
  { 231, 34 },
  { 390, 34 },
  { 85,  200 },
  { 231, 200 },
  { 390, 200 },
};


// ----------------- Persistent IDs -----------------
const int MAX_ID_LENGTH = 40;
const int NUM_IDS       = 6;
const int EEPROM_SIZE   = 512;

struct StoredSettings {
  char ids[NUM_IDS][MAX_ID_LENGTH + 1];
};

class PersistentIDs {
public:
  void begin() {
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.get(0, settings);
    if (settings.ids[0][0] == '\0') {
      resetToDefaults();
      save();
    }
  }

  void save() {
    EEPROM.put(0, settings);
    EEPROM.commit();
  }

  void setID(int index, const char* id) {
    if (index < 0 || index >= NUM_IDS) return;
    strncpy(settings.ids[index], id, MAX_ID_LENGTH);
    settings.ids[index][MAX_ID_LENGTH] = '\0';
  }

  const char* getID(int index) {
    if (index >= 0 && index < NUM_IDS) {
      return settings.ids[index];
    }
    return nullptr;
  }

  void resetToDefaults() {
    strncpy(settings.ids[0], "CMDF4_MPG",           MAX_ID_LENGTH);
    strncpy(settings.ids[1], "CMDF4_COOLANT_TEMP",  MAX_ID_LENGTH);
    strncpy(settings.ids[2], "CMDF4_BATTERY_VOLT",  MAX_ID_LENGTH);
    strncpy(settings.ids[3], "CMDF4_O2",            MAX_ID_LENGTH);
    strncpy(settings.ids[4], "CMDF4_MAP",           MAX_ID_LENGTH);
    strncpy(settings.ids[5], "CMDF4_IAC_PRES",      MAX_ID_LENGTH);
  }

private:
  StoredSettings settings;
};


// ----------------- Field lookup helper -----------------
inline DataField* findFieldByID(const char* id, const DataField* list, size_t count) {
  for (size_t i = 0; i < count; i++) {
    if (strcmp(id, list[i].id) == 0) {
      return (DataField*)&list[i];
    }
  }
  return nullptr;
}

#endif // ALDL_CORE_H
