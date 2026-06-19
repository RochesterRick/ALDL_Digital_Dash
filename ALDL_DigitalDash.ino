// ALDL_DigitalDash.ino — JC8048W550C (ESP32-8048S050C) DASH UI + ALDL (F4/F5 fields)
//Rotate ALDL dash display for portrait mounting

//- Changed display orientation for 90-degree clockwise mounted screen
//- Changed tile layout from 3 columns x 2 rows to 2 columns x 3 rows
//- Verified on bench and in truck with live ALDL data
//- Touch/display layout appears correct

#include <Arduino.h>
#include "esp_display_panel.hpp"
#include "lvgl_v8_port.h"

#define ENABLE_AHT 1

#if ENABLE_AHT
  #include "driver/i2c.h"
#endif

#define USE_CUSTOM_FONT_64 0
#if USE_CUSTOM_FONT_64
  #include "montserrat_64.c"
  LV_FONT_DECLARE(lv_font_montserrat_64);
#endif

#include "ALDLCore.h"

// ----------------- ALDL pins -----------------
#define ENABLE_ALDL 1
#define ALDL_RX_PIN     17
#define ALDL_TX_PIN     18
#define ALDL_EN_RX_PIN  11
#define TX_OE_PIN       12

#define ALDL_BAUD                    8192
#define ALDL_POLL_MS                 500
#define ALDL_RESPONSE_DELAY_MS       40
#define ALDL_RX_WINDOW_MS            60
#define ALDL_INTERBYTE_TIMEOUT_MS    6
#define BIT_TIME_US                  122

#define FRAME_MAX_LEN          96
#define F4_VALID_LEN           64
#define F5_VALID_LEN           70
#define FRAME_LOG_CAPACITY     50

#if ENABLE_AHT
// ----------------- AHT sensor on dedicated I2C bus -----------------
#define AHT_I2C_SDA_PIN 10
#define AHT_I2C_SCL_PIN 13
#define AHT_I2C_PORT    I2C_NUM_1
#define AHT_I2C_FREQ_HZ 100000
#define AHT_I2C_TIMEOUT_MS 10
#define AHT_MAX_READ_FAILURES 3
#define AHT_I2C_ADDR_DEFAULT 0x38
#define AHT_CMD_CALIBRATE 0xE1
#define AHT_CMD_TRIGGER 0xAC
#define AHT_CMD_SOFTRESET 0xBA
#define AHT_STATUS_BUSY 0x80
#define AHT_STATUS_CALIBRATED 0x08
#define AHT_DEBUG_VERBOSE 0
#endif

#define ENABLE_WIFI_WEB 0
#if ENABLE_WIFI_WEB
#include <WiFi.h>
#include <WebServer.h>
static const char* AP_SSID = "ALDL-Dash";
static const char* AP_PASS = "12345678";
static WebServer server(80);
#endif
static bool validate_len_for_source(FieldSource src, size_t len);
static const uint32_t TILE_ACCENT_HEX[6] = {
  0x00E5FF, 0xFFD400, 0x00E5FF, 0xFF2D95, 0x00E5FF, 0xB084FF
};
static const uint32_t TILE_VALUE_HEX[6] = {
  0x39FF14, 0xFFD400, 0x5DF2FF, 0xFF69B4, 0x5DF2FF, 0xB084FF
};

#if ENABLE_AHT
static const char* AHT_CABIN_ID = "AHT_CABIN";
static const char* AHT_OLD_OUTSIDE_TEMP_ID = "AHT_OUTSIDE_TEMP_F";
static const char* AHT_OLD_OUTSIDE_HUMIDITY_ID = "AHT_OUTSIDE_HUMIDITY";
static const DataField AHT_FIELDS[] = {
  { "AHT_CABIN", "Cabin", "", 0, 0, 1.0f, 0.0f, 1 },
};
static const size_t AHT_FIELD_COUNT = sizeof(AHT_FIELDS) / sizeof(AHT_FIELDS[0]);
#endif

PersistentIDs idStore;

enum LayoutMode : uint8_t { LAYOUT_TILES = 0, LAYOUT_LIST_DETAIL = 1 };
static LayoutMode layout_mode = LAYOUT_TILES;

static const int kSlotCount = 6;
static int selected_slot = 0;

// --- TILE layout objects ---
static lv_obj_t *tiles[kSlotCount]       = {nullptr};
static lv_obj_t *tileHeaders[kSlotCount] = {nullptr};
static lv_obj_t *tileSymbols[kSlotCount] = {nullptr};
static lv_obj_t *tileValues[kSlotCount]  = {nullptr};
static lv_obj_t *valueRows[kSlotCount]   = {nullptr};

// --- LIST+DETAIL objects ---
static lv_obj_t *list_root = nullptr;
static lv_obj_t *list_col  = nullptr;
static lv_obj_t *detail_col = nullptr;
static lv_obj_t *list_items[kSlotCount]       = {nullptr};
static lv_obj_t *list_name_lbl[kSlotCount]    = {nullptr};
static lv_obj_t *list_value_lbl[kSlotCount]   = {nullptr};
static lv_obj_t *list_symbol_lbl[kSlotCount]  = {nullptr};
static lv_obj_t *list_value_row[kSlotCount]   = {nullptr};
static lv_obj_t *detail_name = nullptr;
static lv_obj_t *detail_value = nullptr;
static lv_obj_t *detail_symbol = nullptr;
static lv_obj_t *detail_value_row = nullptr;

// F4 and F5 last-good frames kept separately
static uint8_t latestFrameF4[F4_VALID_LEN] = {0};
static size_t  latestFrameF4Len = 0;
static bool    haveValidFrameF4 = false;
static uint32_t latestFrameF4Millis = 0;

static uint8_t latestFrameF5[F5_VALID_LEN] = {0};
static size_t  latestFrameF5Len = 0;
static bool    haveValidFrameF5 = false;
static uint32_t latestFrameF5Millis = 0;

// Last raw receive attempt, for diagnostics only
static uint8_t latestRawFrame[FRAME_MAX_LEN] = {0};
static size_t  latestRawFrameLen = 0;
static char    latestRawFrameHex[FRAME_MAX_LEN * 3 + 1] = {0};
static char    latestJson[2200] = "{\"status\":\"no data yet\"}";

static uint32_t goodFrameCountF4 = 0;
static uint32_t goodFrameCountF5 = 0;
static uint32_t badLengthCountF4 = 0;
static uint32_t badLengthCountF5 = 0;
static uint32_t timeoutCountF4 = 0;
static uint32_t timeoutCountF5 = 0;
static size_t   lastRejectedLengthF4 = 0;
static size_t   lastRejectedLengthF5 = 0;
static uint32_t normalized67CountF4 = 0;
static uint32_t normalized68CountF5 = 0;

float externalTempF = 0.0f;
float externalHumidity = 0.0f;
bool ahtPresent = false;
#if ENABLE_AHT
static uint8_t ahtReadFailureCount = 0;
static bool ahtReadFailed = false;
static bool ahtReadFailureReported = false;
#endif

static char frameLog[FRAME_LOG_CAPACITY][FRAME_MAX_LEN * 3 + 64] = {{0}};
static uint16_t frameLogCount = 0;
static uint16_t frameLogHead = 0;

// Styles
static lv_style_t style_selected;
static bool style_selected_inited = false;

// ----------------- Forward declarations -----------------
static void build_ui();
static void build_ui_tiles();
static void build_ui_list_detail();
static void update_latest_raw_frame_buffers(const uint8_t* frame, size_t len);
static void tile_event_cb(lv_event_t *e);
static void list_item_event_cb(lv_event_t *e);
static void advance_saved_field(int slot_idx);
static void update_slot_labels_from_id(int idx);
static void update_detail_from_slot(int idx);

static const DataField* getFieldByCombinedIndex(int idx, FieldSource* srcOut = nullptr);
static int getCombinedFieldCount();
static int combined_index_of_id(const char* id);
static const DataField* findFieldForSelectedID(const char* id, FieldSource* srcOut = nullptr);

static void rebuild_latest_json();
static void append_frame_log(const char* tag, const uint8_t* frame, size_t len);
static const DataField* find_field_by_name_fragment(const char* needle);
static bool decode_field_to_text_from_frame(const DataField* f, const uint8_t* frame, size_t len, char* out, size_t outSize);
static bool decode_selected_id_to_text(const char* id, char* out, size_t outSize);
static void update_readings_from_frames();
static void determineRequiredCommands(bool &needF4, bool &needF5);
#if ENABLE_AHT
static void migrate_legacy_aht_ids();
static bool aht_is_selected();
static void aht_init();
static void aht_update();
#endif

#if ENABLE_WIFI_WEB
static void wifi_init();
static void wifi_handle();
static void handleRoot();
static void handleData();
static void handleJson();
static void handleLog();
#endif
// ----------------- Helpers -----------------
static const DataField* getFieldByCombinedIndex(int idx, FieldSource* srcOut)
{
  if (idx < 0) return nullptr;
  if (idx < (int)COMMAND1_FIELD_COUNT) {
    if (srcOut) *srcOut = FIELD_F4;
    return &COMMAND1_FIELDS[idx];
  }
  idx -= (int)COMMAND1_FIELD_COUNT;
  if (idx < (int)COMMAND0_FIELD_COUNT) {
    if (srcOut) *srcOut = FIELD_F5;
    return &COMMAND0_FIELDS[idx];
  }
#if ENABLE_AHT
  idx -= (int)COMMAND0_FIELD_COUNT;
  if (idx < (int)AHT_FIELD_COUNT) {
    if (srcOut) *srcOut = FIELD_NONE;
    return &AHT_FIELDS[idx];
  }
#endif
  if (srcOut) *srcOut = FIELD_NONE;
  return nullptr;
}

static int getCombinedFieldCount()
{
  int count = (int)COMMAND1_FIELD_COUNT + (int)COMMAND0_FIELD_COUNT;
#if ENABLE_AHT
  count += (int)AHT_FIELD_COUNT;
#endif
  return count;
}

static int combined_index_of_id(const char* id)
{
  if (!id) return -1;
  for (int i = 0; i < getCombinedFieldCount(); i++) {
    const DataField* f = getFieldByCombinedIndex(i, nullptr);
    if (f && strcmp(id, f->id) == 0) return i;
  }
  return -1;
}

static const DataField* findFieldForSelectedID(const char* id, FieldSource* srcOut)
{
  FieldSource src = getFieldSourceByID(id);
  if (srcOut) *srcOut = src;
  if (src == FIELD_F4) return findFieldByID(id, COMMAND1_FIELDS, COMMAND1_FIELD_COUNT);
  if (src == FIELD_F5) return findFieldByID(id, COMMAND0_FIELDS, COMMAND0_FIELD_COUNT);
#if ENABLE_AHT
  if (id) {
    for (size_t i = 0; i < AHT_FIELD_COUNT; i++) {
      if (strcmp(id, AHT_FIELDS[i].id) == 0) return &AHT_FIELDS[i];
    }
  }
#endif
  return nullptr;
}

static lv_coord_t symbol_gap_px_for_font(const lv_font_t* vf)
{
  int h = vf ? (int)lv_font_get_line_height(vf) : 28;
  int gap = h / 4;
  if (gap < 10) gap = 10;
  if (gap > 80) gap = 80;
  return (lv_coord_t)gap;
}

static void set_row_gap_by_value_label(lv_obj_t* row, lv_obj_t* value_label)
{
  if (!row || !value_label) return;
  const lv_font_t* vf = lv_obj_get_style_text_font(value_label, 0);
  lv_obj_set_style_pad_column(row, symbol_gap_px_for_font(vf), 0);
}

static void update_slot_labels_from_id(int idx)
{
  if (idx < 0 || idx >= kSlotCount) return;
  const char* id = idStore.getID(idx);
  FieldSource src = FIELD_NONE;
  const DataField* f = findFieldForSelectedID(id, &src);

  const char* name = (f && f->name) ? f->name : (id ? id : "-");
  const char* sym  = (f && f->symbol && f->symbol[0]) ? f->symbol : "";

  if (tileHeaders[idx]) lv_label_set_text(tileHeaders[idx], name);
  if (tileSymbols[idx]) lv_label_set_text(tileSymbols[idx], sym);

  if (list_name_lbl[idx]) lv_label_set_text(list_name_lbl[idx], name);
  if (list_symbol_lbl[idx]) lv_label_set_text(list_symbol_lbl[idx], sym);

  if (tileSymbols[idx]) {
    if (!sym || sym[0] == '\0') lv_obj_add_flag(tileSymbols[idx], LV_OBJ_FLAG_HIDDEN);
    else                        lv_obj_clear_flag(tileSymbols[idx], LV_OBJ_FLAG_HIDDEN);
  }
  if (list_symbol_lbl[idx]) {
    if (!sym || sym[0] == '\0') lv_obj_add_flag(list_symbol_lbl[idx], LV_OBJ_FLAG_HIDDEN);
    else                        lv_obj_clear_flag(list_symbol_lbl[idx], LV_OBJ_FLAG_HIDDEN);
  }
}

static void update_detail_from_slot(int idx)
{
  if (!detail_name || !detail_value || !detail_symbol || !detail_value_row) return;

  const char* id = idStore.getID(idx);
  const DataField* f = findFieldForSelectedID(id, nullptr);

  const char* name = (f && f->name) ? f->name : (id ? id : "-");
  const char* sym  = (f && f->symbol && f->symbol[0]) ? f->symbol : "";

  lv_label_set_text(detail_name, name);

  const char* vtxt = (list_value_lbl[idx]) ? lv_label_get_text(list_value_lbl[idx]) : "--";
  lv_label_set_text(detail_value, vtxt ? vtxt : "--");

  lv_label_set_text(detail_symbol, sym);
  if (!sym || sym[0] == '\0') lv_obj_add_flag(detail_symbol, LV_OBJ_FLAG_HIDDEN);
  else                        lv_obj_clear_flag(detail_symbol, LV_OBJ_FLAG_HIDDEN);

  lv_obj_set_style_text_color(detail_name,   lv_color_hex(TILE_ACCENT_HEX[idx]), 0);
  lv_obj_set_style_text_color(detail_symbol, lv_color_hex(TILE_ACCENT_HEX[idx]), 0);
  lv_obj_set_style_text_color(detail_value,  lv_color_hex(TILE_VALUE_HEX[idx]),  0);
}

static void advance_saved_field(int slot_idx)
{
  const char* cur_id = idStore.getID(slot_idx);
  int cur = combined_index_of_id(cur_id);

  int next = (cur < 0) ? 0 : (cur + 1);
  if (next >= getCombinedFieldCount()) next = 0;

  const DataField* f = getFieldByCombinedIndex(next, nullptr);
  if (!f) return;

  idStore.setID(slot_idx, f->id);
  idStore.save();

  update_slot_labels_from_id(slot_idx);

  if (tileValues[slot_idx]) lv_label_set_text(tileValues[slot_idx], "--");
  if (list_value_lbl[slot_idx]) lv_label_set_text(list_value_lbl[slot_idx], "--");

  update_readings_from_frames();

  if (selected_slot == slot_idx && layout_mode == LAYOUT_LIST_DETAIL) {
    update_detail_from_slot(slot_idx);
  }
}

static void apply_selection_style_tiles(int idx)
{
  if (idx < 0 || idx >= kSlotCount) return;
  if (selected_slot >= 0 && selected_slot < kSlotCount && tiles[selected_slot]) {
    lv_obj_remove_style(tiles[selected_slot], &style_selected, 0);
  }
  selected_slot = idx;
  if (tiles[idx]) lv_obj_add_style(tiles[idx], &style_selected, 0);
}

static void apply_selection_style_list(int idx)
{
  if (idx < 0 || idx >= kSlotCount) return;
  if (selected_slot >= 0 && selected_slot < kSlotCount && list_items[selected_slot]) {
    lv_obj_remove_style(list_items[selected_slot], &style_selected, 0);
  }
  selected_slot = idx;
  if (list_items[idx]) lv_obj_add_style(list_items[idx], &style_selected, 0);
}

// ----------------- Events -----------------
static void tile_event_cb(lv_event_t *e)
{
  int idx = (int)(intptr_t)lv_event_get_user_data(e);
  apply_selection_style_tiles(idx);
  advance_saved_field(idx);
}

static void list_item_event_cb(lv_event_t *e)
{
  int idx = (int)(intptr_t)lv_event_get_user_data(e);
  apply_selection_style_list(idx);
  advance_saved_field(idx);
  update_detail_from_slot(idx);
}

// ----------------- Latest data builders -----------------
static void update_latest_raw_frame_buffers(const uint8_t* frame, size_t len)
{
  if (!frame) return;
  if (len > FRAME_MAX_LEN) len = FRAME_MAX_LEN;

  if (len > 0) memcpy(latestRawFrame, frame, len);
  latestRawFrameLen = len;

  size_t pos = 0;
  latestRawFrameHex[0] = '\0';
  for (size_t i = 0; i < len; i++) {
    int written = snprintf(&latestRawFrameHex[pos], sizeof(latestRawFrameHex) - pos,
                           (i + 1 < len) ? "%02X " : "%02X", frame[i]);
    if (written <= 0) break;
    pos += (size_t)written;
    if (pos >= sizeof(latestRawFrameHex) - 1) break;
  }
}

static void append_frame_log(const char* tag, const uint8_t* frame, size_t len)
{
  size_t slot = frameLogHead % FRAME_LOG_CAPACITY;
  char hexbuf[FRAME_MAX_LEN * 3 + 1];
  hexbuf[0] = '\0';
  size_t pos = 0;
  size_t capped = (len > FRAME_MAX_LEN) ? FRAME_MAX_LEN : len;
  for (size_t i = 0; i < capped; i++) {
    int written = snprintf(&hexbuf[pos], sizeof(hexbuf) - pos,
                           (i + 1 < capped) ? "%02X " : "%02X", frame[i]);
    if (written <= 0) break;
    pos += (size_t)written;
    if (pos >= sizeof(hexbuf) - 1) break;
  }

  snprintf(frameLog[slot], sizeof(frameLog[slot]), "%lu,%s,%u,%s",
           (unsigned long)millis(),
           tag ? tag : "RX",
           (unsigned)capped,
           hexbuf);

  frameLogHead = (frameLogHead + 1) % FRAME_LOG_CAPACITY;
  if (frameLogCount < FRAME_LOG_CAPACITY) frameLogCount++;
}

static const DataField* find_field_by_name_fragment(const char* needle)
{
  if (!needle || !needle[0]) return nullptr;
  for (size_t i = 0; i < COMMAND1_FIELD_COUNT; i++) {
    const char* n = COMMAND1_FIELDS[i].name;
    if (n && strstr(n, needle)) return &COMMAND1_FIELDS[i];
  }
  return nullptr;
}

static bool decode_field_to_text_from_frame(const DataField* f, const uint8_t* frame, size_t len, char* out, size_t outSize)
{
  if (!out || outSize == 0) return false;
  out[0] = '\0';
  if (!f || !frame || len == 0) return false;

  if (strcmp(f->id, "CMDF5_TCC_LOCK") == 0) {
    if (len <= 0x1D) return false;
    bool locked = (frame[0x1D] & 0x40) == 0x40;
    snprintf(out, outSize, "%s", locked ? "Locked" : "UnLocked");
    return true;
  }

  if (strcmp(f->id, "CMDF4_MPG") == 0) {
    const DataField* mphF = findFieldByID("CMDF4_MPH", COMMAND1_FIELDS, COMMAND1_FIELD_COUNT);
    const DataField* bpwF = findFieldByID("CMDF4_BPW", COMMAND1_FIELDS, COMMAND1_FIELD_COUNT);
    const DataField* rpmF = findFieldByID("CMDF4_ENGINE_SPEED", COMMAND1_FIELDS, COMMAND1_FIELD_COUNT);

    if (!mphF || !bpwF || !rpmF) return false;

    char mphBuf[24], bpwBuf[24], rpmBuf[24];
    if (!decode_field_to_text_from_frame(mphF, frame, len, mphBuf, sizeof(mphBuf))) return false;
    if (!decode_field_to_text_from_frame(bpwF, frame, len, bpwBuf, sizeof(bpwBuf))) return false;
    if (!decode_field_to_text_from_frame(rpmF, frame, len, rpmBuf, sizeof(rpmBuf))) return false;

    float mph = atof(mphBuf);
    float bpw = atof(bpwBuf);
    float rpm = atof(rpmBuf);

    if (mph < 1.0f || bpw <= 0.0f || rpm <= 0.0f) {
      snprintf(out, outSize, "0.0");
      return true;
    }

    float mpg = mph / (0.0006148f * rpm * bpw);
    if (mpg < 0.0f) mpg = 0.0f;
    if (mpg > 99.9f) mpg = 99.9f;

    snprintf(out, outSize, "%.1f", (double)mpg);
    return true;
  }

  if (strcmp(f->id, "CMDF4_AFR_EST") == 0) {
    const DataField* o2F = findFieldByID("CMDF4_O2_SENSOR", COMMAND1_FIELDS, COMMAND1_FIELD_COUNT);
    if (!o2F) return false;

    char o2Buf[24];
    if (!decode_field_to_text_from_frame(o2F, frame, len, o2Buf, sizeof(o2Buf))) return false;

    float mv = atof(o2Buf);
    float afr = 14.7f - ((mv - 450.0f) / 100.0f) * 0.25f;

    if (afr < 13.0f) afr = 13.0f;
    if (afr > 16.0f) afr = 16.0f;

    snprintf(out, outSize, "%.1f", (double)afr);
    return true;
  }

  if (f->adsByteNumber <= 0) return false;
  size_t idx = (size_t)(f->adsByteNumber - 1);
  size_t need = idx + (size_t)f->size;
  if (need > len) return false;

  uint32_t raw = 0;
  if (f->size == 1) {
    raw = frame[idx];
  } else {
    raw = ((uint32_t)frame[idx] << 8) | frame[idx + 1];
  }

  float val = (float)raw * f->scale + f->offsetValue;
  int prec = (int)f->precision;
  if (prec < 0) prec = 0;
  if (prec > 4) prec = 4;
  snprintf(out, outSize, "%.*f", prec, (double)val);
  return true;
}

#if ENABLE_AHT
static bool decode_aht_id_to_text(const char* id, char* out, size_t outSize)
{
  if (!id || !out || outSize == 0) return false;

  if (strcmp(id, AHT_CABIN_ID) != 0) return false;

  if (ahtReadFailed) {
    snprintf(out, outSize, "ERR\nERR");
    return true;
  }

  if (!ahtPresent) {
    snprintf(out, outSize, "---\n---");
    return true;
  }

  snprintf(out, outSize, "%.1f°F\n%.1f%%", (double)externalTempF, (double)externalHumidity);
  return true;
}
#endif

static bool decode_selected_id_to_text(const char* id, char* out, size_t outSize)
{
  if (!out || outSize == 0) return false;
  out[0] = '\0';
#if ENABLE_AHT
  if (decode_aht_id_to_text(id, out, outSize)) return true;
#endif
  FieldSource src = FIELD_NONE;
  const DataField* f = findFieldForSelectedID(id, &src);
  if (!f) return false;

  if (src == FIELD_F4) {
    if (!haveValidFrameF4) return false;
    return decode_field_to_text_from_frame(f, latestFrameF4, latestFrameF4Len, out, outSize);
  }
  if (src == FIELD_F5) {
    if (!haveValidFrameF5) return false;
    return decode_field_to_text_from_frame(f, latestFrameF5, latestFrameF5Len, out, outSize);
  }
  return false;
}

static void rebuild_latest_json()
{
  const DataField* rpmF  = find_field_by_name_fragment("RPM");
  const DataField* tpsF  = find_field_by_name_fragment("T.P.S.");
  if (!tpsF) tpsF = find_field_by_name_fragment("TPS");
  const DataField* battF = find_field_by_name_fragment("Battery");
  const DataField* tempF = find_field_by_name_fragment("Coolant");
  if (!tempF) tempF = find_field_by_name_fragment("Temp");

  char rpmBuf[24] = "--";
  char tpsBuf[24] = "--";
  char battBuf[24] = "--";
  char tempBuf[24] = "--";

  if (haveValidFrameF4) {
    decode_field_to_text_from_frame(rpmF, latestFrameF4, latestFrameF4Len, rpmBuf, sizeof(rpmBuf));
    decode_field_to_text_from_frame(tpsF, latestFrameF4, latestFrameF4Len, tpsBuf, sizeof(tpsBuf));
    decode_field_to_text_from_frame(battF, latestFrameF4, latestFrameF4Len, battBuf, sizeof(battBuf));
    decode_field_to_text_from_frame(tempF, latestFrameF4, latestFrameF4Len, tempBuf, sizeof(tempBuf));
  }

  size_t pos = 0;
  int n = snprintf(latestJson + pos, sizeof(latestJson) - pos,
                 "{\"ms_f4\":%lu,\"ms_f5\":%lu,\"selected_slot\":%d,"
                 "\"f4_valid\":%s,\"f5_valid\":%s,"
                 "\"anchors\":{\"RPM\":\"%s\",\"TPS\":\"%s\",\"Batt\":\"%s\",\"Temp\":\"%s\"},\"slots\":[",
                 (unsigned long)latestFrameF4Millis,
                 (unsigned long)latestFrameF5Millis,
                 selected_slot,
                 haveValidFrameF4 ? "true" : "false",
                 haveValidFrameF5 ? "true" : "false",
                 rpmBuf, tpsBuf, battBuf, tempBuf);
  if (n < 0) return;
  pos += (size_t)n;
  if (pos >= sizeof(latestJson)) {
    latestJson[sizeof(latestJson) - 1] = '\0';
    return;
  }

  for (int i = 0; i < kSlotCount; i++) {
    const char* id = idStore.getID(i);
    FieldSource src = FIELD_NONE;
    const DataField* f = findFieldForSelectedID(id, &src);
    const char* name = (f && f->name) ? f->name : "-";
    const char* symbol = (f && f->symbol) ? f->symbol : "";

    char valueBuf[24] = "--";
    decode_selected_id_to_text(id, valueBuf, sizeof(valueBuf));

    const char* srcName = (src == FIELD_F4) ? "F4" : ((src == FIELD_F5) ? "F5" : "?");
#if ENABLE_AHT
    if (id && strcmp(id, AHT_CABIN_ID) == 0) {
      srcName = "AHT";
    }
#endif
    n = snprintf(latestJson + pos, sizeof(latestJson) - pos,
                 "%s{\"slot\":%d,\"name\":\"%s\",\"value\":\"%s\",\"symbol\":\"%s\",\"src\":\"%s\"}",
                 (i == 0) ? "" : ",",
                 i, name, valueBuf, symbol, srcName);
    if (n < 0) return;
    pos += (size_t)n;
    if (pos >= sizeof(latestJson)) {
      latestJson[sizeof(latestJson) - 1] = '\0';
      return;
    }
  }

  n = snprintf(latestJson + pos, sizeof(latestJson) - pos,
               "],\"last_rx_raw\":\"%s\",\"last_rx_len\":%u,"
               "\"good_f4\":%lu,\"good_f5\":%lu,"
               "\"bad_f4\":%lu,\"bad_f5\":%lu,"
               "\"timeouts_f4\":%lu,\"timeouts_f5\":%lu,"
               "\"norm67_f4\":%lu,\"norm68_f5\":%lu,"
               "\"last_rejected_f4\":%u,\"last_rejected_f5\":%u,\"log_count\":%u}",
               latestRawFrameHex,
               (unsigned)latestRawFrameLen,
               (unsigned long)goodFrameCountF4,
               (unsigned long)goodFrameCountF5,
               (unsigned long)badLengthCountF4,
               (unsigned long)badLengthCountF5,
               (unsigned long)timeoutCountF4,
               (unsigned long)timeoutCountF5,
               (unsigned long)normalized67CountF4,
               (unsigned long)normalized68CountF5,
               (unsigned)lastRejectedLengthF4,
               (unsigned)lastRejectedLengthF5,
               (unsigned)frameLogCount);
  if (n < 0) return;
  if ((size_t)n >= sizeof(latestJson) - pos) latestJson[sizeof(latestJson) - 1] = '\0';
}

#if ENABLE_WIFI_WEB
static void handleRoot()
{
  const char html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ALDL Dash</title>
  <style>
    body { background:#0d1117; color:#e6edf3; font-family:Arial,sans-serif; margin:0; padding:16px; }
    h1 { color:#7ee787; }
    .card { background:#161b22; border:1px solid #30363d; border-radius:12px; padding:14px; margin-bottom:14px; }
    .row { display:flex; justify-content:space-between; gap:12px; padding:8px 0; border-bottom:1px solid #222; }
    .row:last-child { border-bottom:none; }
    .name { color:#79c0ff; }
    .val  { color:#7ee787; font-weight:bold; }
    .raw  { white-space:pre-wrap; word-break:break-word; font-family:monospace; color:#f2cc60; }
    .small { color:#8b949e; font-size:14px; }
  </style>
</head>
<body>
  <h1>ESP32 ALDL Dash</h1>
  <div class="card">
    <div class="small">Anchor values</div>
    <div id="anchors">waiting...</div>
  </div>
  <div class="card"><div class="small">Live data from /json</div><div id="slots">waiting...</div></div>
  <div class="card"><div class="small">Most recent receive attempt</div><div id="lastrx" class="raw">waiting...</div></div>
  <div class="card"><div class="small">Frame log</div><div><a href="/log" target="_blank">Open last 50 receive attempts</a></div></div>
  <script>
    async function refresh() {
      try {
        const r = await fetch('/json');
        const j = await r.json();
        const a = j.anchors || {};
        document.getElementById('anchors').innerHTML = `
          <div class="row"><div class="name">RPM</div><div class="val">${a.RPM || '--'}</div></div>
          <div class="row"><div class="name">TPS</div><div class="val">${a.TPS || '--'}</div></div>
          <div class="row"><div class="name">Batt</div><div class="val">${a.Batt || '--'}</div></div>
          <div class="row"><div class="name">Temp</div><div class="val">${a.Temp || '--'}</div></div>
          <div class="row"><div class="name">Stats</div><div class="val">goodF4=${j.good_f4||0} goodF5=${j.good_f5||0}</div></div>`;

        let html = '';
        for (const s of (j.slots || [])) {
          html += `<div class="row"><div class="name">[${s.src}] ${s.name}</div><div class="val">${s.value} ${s.symbol || ''}</div></div>`;
        }
        document.getElementById('slots').innerHTML = html || 'no slots';
        document.getElementById('lastrx').textContent = `[len=${j.last_rx_len || 0}] ` + (j.last_rx_raw || 'no raw receive yet');
      } catch (e) {
        document.getElementById('anchors').innerHTML = 'error reading /json';
        document.getElementById('slots').innerHTML = 'error reading /json';
      }
    }
    refresh();
    setInterval(refresh, 500);
  </script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

static void handleData()
{
  server.send(200, "text/plain", latestRawFrameHex);
}

static void handleJson()
{
  server.send(200, "application/json", latestJson);
}

static void handleLog()
{
  String out;
  out.reserve(14000);
  out += "ms,tag,len,raw\n";

  uint16_t start = (frameLogCount < FRAME_LOG_CAPACITY) ? 0 : frameLogHead;
  for (uint16_t i = 0; i < frameLogCount; i++) {
    uint16_t idx = (start + i) % FRAME_LOG_CAPACITY;
    out += frameLog[idx];
    out += "\n";
  }

  server.send(200, "text/plain", out);
}

static void wifi_init()
{
  WiFi.mode(WIFI_AP);

  bool ok = WiFi.softAP(AP_SSID, AP_PASS);
  if (!ok) {
    Serial.println("WiFi SoftAP start failed.");
    return;
  }

  IPAddress ip = WiFi.softAPIP();
  Serial.print("WiFi AP started. SSID: ");
  Serial.println(AP_SSID);
  Serial.print("AP IP: ");
  Serial.println(ip);

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/json", handleJson);
  server.on("/log", handleLog);
  server.begin();

  Serial.println("HTTP server started on port 80");
  Serial.println("Connect phone to WiFi 'ALDL-Dash' then open http://192.168.4.1/");
}

static void wifi_handle()
{
  server.handleClient();
}
#endif

// ----------------- UI builders -----------------
static void init_selected_style_once()
{
  if (style_selected_inited) return;
  style_selected_inited = true;
  lv_style_init(&style_selected);
  lv_style_set_border_color(&style_selected, lv_color_hex(0xFFFFFF));
  lv_style_set_border_width(&style_selected, 4);
  lv_style_set_bg_color(&style_selected, lv_color_hex(0x151B38));
  lv_style_set_bg_opa(&style_selected, LV_OPA_50);
  lv_style_set_shadow_opa(&style_selected, LV_OPA_TRANSP);
  lv_style_set_shadow_width(&style_selected, 0);
  lv_style_set_shadow_spread(&style_selected, 0);
}

static void build_ui()
{
  init_selected_style_once();

  lv_obj_t *scr = lv_scr_act();
  lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clean(scr);

  lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_grad_dir(scr, LV_GRAD_DIR_NONE, 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

  memset(tiles, 0, sizeof(tiles));
  memset(tileHeaders, 0, sizeof(tileHeaders));
  memset(tileSymbols, 0, sizeof(tileSymbols));
  memset(tileValues, 0, sizeof(tileValues));
  memset(valueRows, 0, sizeof(valueRows));

  list_root = nullptr;
  list_col = nullptr;
  detail_col = nullptr;
  memset(list_items, 0, sizeof(list_items));
  memset(list_name_lbl, 0, sizeof(list_name_lbl));
  memset(list_value_lbl, 0, sizeof(list_value_lbl));
  memset(list_symbol_lbl, 0, sizeof(list_symbol_lbl));
  memset(list_value_row, 0, sizeof(list_value_row));
  detail_name = detail_value = detail_symbol = detail_value_row = nullptr;

  if (layout_mode == LAYOUT_TILES) build_ui_tiles();
  else                            build_ui_list_detail();

  if (selected_slot < 0 || selected_slot >= kSlotCount) selected_slot = 0;
  if (layout_mode == LAYOUT_TILES) apply_selection_style_tiles(selected_slot);
  else {
    apply_selection_style_list(selected_slot);
    update_detail_from_slot(selected_slot);
  }

  update_readings_from_frames();
}

static void build_ui_tiles()
{
  lv_obj_t *scr = lv_scr_act();
  static lv_coord_t col_dsc[] = { LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST };
  static lv_coord_t row_dsc[] = { LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST };

  lv_obj_set_grid_dsc_array(scr, col_dsc, row_dsc);
  lv_obj_set_layout(scr, LV_LAYOUT_GRID);
  lv_obj_set_style_pad_row(scr, 16, 0);
  lv_obj_set_style_pad_column(scr, 16, 0);
  lv_obj_set_style_pad_all(scr, 16, 0);

  for (int i = 0; i < kSlotCount; i++) {
    lv_obj_t *tile = lv_obj_create(scr);
    tiles[i] = tile;
    valueRows[i] = nullptr;

    lv_obj_add_event_cb(tile, tile_event_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
    lv_obj_clear_flag(tile, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(tile, 14, 0);
    lv_obj_set_style_border_width(tile, 2, 0);
    lv_obj_set_style_border_color(tile, lv_color_hex(TILE_ACCENT_HEX[i]), 0);
    lv_obj_set_style_shadow_opa(tile, LV_OPA_TRANSP, 0);
    lv_obj_set_style_bg_color(tile, lv_color_hex(0x0B1033), 0);
    lv_obj_set_style_bg_grad_dir(tile, LV_GRAD_DIR_NONE, 0);
    lv_obj_set_style_bg_opa(tile, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(tile, 14, 0);

    int r = (i / 2);
    int c = (i % 2);
    lv_obj_set_grid_cell(tile, LV_GRID_ALIGN_STRETCH, c, 1, LV_GRID_ALIGN_STRETCH, r, 1);
    #ifdef LV_LAYOUT_NONE
      lv_obj_set_layout(tile, LV_LAYOUT_NONE);
    #else
      lv_obj_set_layout(tile, 0);
    #endif

    tileHeaders[i] = lv_label_create(tile);
    lv_obj_set_style_bg_opa(tileHeaders[i], LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(tileHeaders[i], lv_color_hex(0x0B1033), 0);
    lv_obj_set_style_pad_all(tileHeaders[i], 2, 0);
    lv_obj_add_flag(tileHeaders[i], LV_OBJ_FLAG_FLOATING);
    lv_obj_add_flag(tileHeaders[i], LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_align(tileHeaders[i], LV_ALIGN_TOP_LEFT, 6, 6);
    #if defined(LV_FONT_MONTSERRAT_28) && LV_FONT_MONTSERRAT_28
      lv_obj_set_style_text_font(tileHeaders[i], &lv_font_montserrat_28, 0);
    #else
      lv_obj_set_style_text_font(tileHeaders[i], LV_FONT_DEFAULT, 0);
    #endif
    lv_obj_set_style_text_color(tileHeaders[i], lv_color_hex(TILE_ACCENT_HEX[i]), 0);
    lv_obj_set_style_text_letter_space(tileHeaders[i], 2, 0);

    tileValues[i] = lv_label_create(tile);
    lv_obj_set_style_bg_opa(tileValues[i], LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(tileValues[i], lv_color_hex(0x0B1033), 0);
    lv_obj_set_style_pad_all(tileValues[i], 2, 0);
    lv_obj_add_flag(tileValues[i], LV_OBJ_FLAG_FLOATING);
    lv_obj_add_flag(tileValues[i], LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_label_set_text(tileValues[i], "--");
    #if USE_CUSTOM_FONT_64
      lv_obj_set_style_text_font(tileValues[i], &lv_font_montserrat_64, 0);
    #elif defined(LV_FONT_MONTSERRAT_48) && LV_FONT_MONTSERRAT_48
      lv_obj_set_style_text_font(tileValues[i], &lv_font_montserrat_48, 0);
    #elif defined(LV_FONT_MONTSERRAT_40) && LV_FONT_MONTSERRAT_40
      lv_obj_set_style_text_font(tileValues[i], &lv_font_montserrat_40, 0);
    #else
      lv_obj_set_style_text_font(tileValues[i], LV_FONT_DEFAULT, 0);
    #endif
    lv_obj_set_style_text_color(tileValues[i], lv_color_hex(TILE_VALUE_HEX[i]), 0);
    lv_obj_set_style_text_align(tileValues[i], LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(tileValues[i], LV_ALIGN_CENTER, 0, 10);

    tileSymbols[i] = lv_label_create(tile);
    lv_obj_set_style_bg_opa(tileSymbols[i], LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(tileSymbols[i], lv_color_hex(0x0B1033), 0);
    lv_obj_set_style_pad_all(tileSymbols[i], 2, 0);
    lv_obj_add_flag(tileSymbols[i], LV_OBJ_FLAG_FLOATING);
    lv_obj_add_flag(tileSymbols[i], LV_OBJ_FLAG_EVENT_BUBBLE);
    #if defined(LV_FONT_MONTSERRAT_22) && LV_FONT_MONTSERRAT_22
      lv_obj_set_style_text_font(tileSymbols[i], &lv_font_montserrat_22, 0);
    #elif defined(LV_FONT_MONTSERRAT_28) && LV_FONT_MONTSERRAT_28
      lv_obj_set_style_text_font(tileSymbols[i], &lv_font_montserrat_28, 0);
    #else
      lv_obj_set_style_text_font(tileSymbols[i], LV_FONT_DEFAULT, 0);
    #endif
    lv_obj_set_style_text_color(tileSymbols[i], lv_color_hex(TILE_ACCENT_HEX[i]), 0);
    lv_obj_align(tileSymbols[i], LV_ALIGN_BOTTOM_LEFT, 8, -6);

    update_slot_labels_from_id(i);
  }
}

static void build_ui_list_detail()
{
  lv_obj_t *scr = lv_scr_act();

  list_root = lv_obj_create(scr);
  lv_obj_set_size(list_root, lv_pct(100), lv_pct(100));
  lv_obj_set_style_bg_opa(list_root, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(list_root, 0, 0);
  lv_obj_set_style_pad_all(list_root, 16, 0);
  lv_obj_clear_flag(list_root, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_layout(list_root, LV_LAYOUT_FLEX);
  lv_obj_set_flex_flow(list_root, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(list_root, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

  list_col = lv_obj_create(list_root);
  lv_obj_set_style_bg_color(list_col, lv_color_hex(0x0B1033), 0);
  lv_obj_set_style_bg_opa(list_col, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(list_col, 14, 0);
  lv_obj_set_style_border_width(list_col, 2, 0);
  lv_obj_set_style_border_color(list_col, lv_color_hex(0x00E5FF), 0);
  lv_obj_set_style_pad_all(list_col, 12, 0);
  lv_obj_set_size(list_col, lv_pct(35), lv_pct(100));
  lv_obj_clear_flag(list_col, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_layout(list_col, LV_LAYOUT_FLEX);
  lv_obj_set_flex_flow(list_col, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(list_col, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

  detail_col = lv_obj_create(list_root);
  lv_obj_set_style_bg_color(detail_col, lv_color_hex(0x0B1033), 0);
  lv_obj_set_style_bg_opa(detail_col, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(detail_col, 14, 0);
  lv_obj_set_style_border_width(detail_col, 2, 0);
  lv_obj_set_style_border_color(detail_col, lv_color_hex(0xB084FF), 0);
  lv_obj_set_style_pad_all(detail_col, 16, 0);
  lv_obj_set_size(detail_col, lv_pct(65), lv_pct(100));
  lv_obj_clear_flag(detail_col, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_layout(detail_col, LV_LAYOUT_FLEX);
  lv_obj_set_flex_flow(detail_col, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(detail_col, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

  for (int i = 0; i < kSlotCount; i++) {
    lv_obj_t *item = lv_obj_create(list_col);
    list_items[i] = item;
    lv_obj_clear_flag(item, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(item, list_item_event_cb, LV_EVENT_CLICKED, (void*)(intptr_t)i);
    lv_obj_set_size(item, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_style_radius(item, 12, 0);
    lv_obj_set_style_border_width(item, 2, 0);
    lv_obj_set_style_border_color(item, lv_color_hex(TILE_ACCENT_HEX[i]), 0);
    lv_obj_set_style_bg_color(item, lv_color_hex(0x08102C), 0);
    lv_obj_set_style_bg_opa(item, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(item, 10, 0);
    lv_obj_set_layout(item, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(item, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(item, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

    list_name_lbl[i] = lv_label_create(item);
    lv_obj_set_style_bg_opa(list_name_lbl[i], LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(list_name_lbl[i], lv_color_hex(0x08102C), 0);
    lv_obj_set_style_pad_all(list_name_lbl[i], 2, 0);
    lv_obj_add_flag(list_name_lbl[i], LV_OBJ_FLAG_EVENT_BUBBLE);
    #if defined(LV_FONT_MONTSERRAT_22) && LV_FONT_MONTSERRAT_22
      lv_obj_set_style_text_font(list_name_lbl[i], &lv_font_montserrat_22, 0);
    #else
      lv_obj_set_style_text_font(list_name_lbl[i], LV_FONT_DEFAULT, 0);
    #endif
    lv_obj_set_style_text_color(list_name_lbl[i], lv_color_hex(TILE_ACCENT_HEX[i]), 0);

    list_value_row[i] = lv_obj_create(item);
    lv_obj_add_flag(list_value_row[i], LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_clear_flag(list_value_row[i], LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(list_value_row[i], LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(list_value_row[i], 0, 0);
    lv_obj_set_style_pad_all(list_value_row[i], 0, 0);
    lv_obj_set_size(list_value_row[i], lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_layout(list_value_row[i], 0);

    list_value_lbl[i] = lv_label_create(list_value_row[i]);
    lv_obj_set_style_bg_opa(list_value_lbl[i], LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(list_value_lbl[i], lv_color_hex(0x08102C), 0);
    lv_obj_set_style_pad_all(list_value_lbl[i], 2, 0);
    lv_obj_add_flag(list_value_lbl[i], LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_label_set_text(list_value_lbl[i], "--");
    #if defined(LV_FONT_MONTSERRAT_28) && LV_FONT_MONTSERRAT_28
      lv_obj_set_style_text_font(list_value_lbl[i], &lv_font_montserrat_28, 0);
    #else
      lv_obj_set_style_text_font(list_value_lbl[i], LV_FONT_DEFAULT, 0);
    #endif
    lv_obj_set_style_text_color(list_value_lbl[i], lv_color_hex(TILE_VALUE_HEX[i]), 0);
    lv_obj_set_style_min_width(list_value_lbl[i], 100, 0);
    lv_obj_set_style_text_align(list_value_lbl[i], LV_TEXT_ALIGN_RIGHT, 0);

    list_symbol_lbl[i] = lv_label_create(list_value_row[i]);
    lv_obj_set_style_bg_opa(list_symbol_lbl[i], LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(list_symbol_lbl[i], lv_color_hex(0x08102C), 0);
    lv_obj_set_style_pad_all(list_symbol_lbl[i], 2, 0);
    lv_obj_add_flag(list_symbol_lbl[i], LV_OBJ_FLAG_EVENT_BUBBLE);
    #if defined(LV_FONT_MONTSERRAT_22) && LV_FONT_MONTSERRAT_22
      lv_obj_set_style_text_font(list_symbol_lbl[i], &lv_font_montserrat_22, 0);
    #else
      lv_obj_set_style_text_font(list_symbol_lbl[i], LV_FONT_DEFAULT, 0);
    #endif
    lv_obj_set_style_text_color(list_symbol_lbl[i], lv_color_hex(TILE_ACCENT_HEX[i]), 0);

    lv_obj_align(list_value_lbl[i], LV_ALIGN_RIGHT_MID, -4, 0);
    lv_obj_align_to(list_symbol_lbl[i], list_value_lbl[i], LV_ALIGN_OUT_RIGHT_MID, 2, 0);

    update_slot_labels_from_id(i);
  }

  detail_name = lv_label_create(detail_col);
  lv_obj_set_style_bg_opa(detail_name, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(detail_name, lv_color_hex(0x0B1033), 0);
  lv_obj_set_style_pad_all(detail_name, 2, 0);
  #if defined(LV_FONT_MONTSERRAT_28) && LV_FONT_MONTSERRAT_28
    lv_obj_set_style_text_font(detail_name, &lv_font_montserrat_28, 0);
  #else
    lv_obj_set_style_text_font(detail_name, LV_FONT_DEFAULT, 0);
  #endif
  lv_obj_set_style_text_letter_space(detail_name, 2, 0);

  detail_value_row = lv_obj_create(detail_col);
  lv_obj_clear_flag(detail_value_row, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_bg_opa(detail_value_row, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(detail_value_row, 0, 0);
  lv_obj_set_style_pad_all(detail_value_row, 0, 0);
  lv_obj_set_size(detail_value_row, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_set_layout(detail_value_row, LV_LAYOUT_FLEX);
  lv_obj_set_flex_flow(detail_value_row, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(detail_value_row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

  detail_value = lv_label_create(detail_value_row);
  lv_obj_set_style_bg_opa(detail_value, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(detail_value, lv_color_hex(0x0B1033), 0);
  lv_obj_set_style_pad_all(detail_value, 2, 0);
  #if USE_CUSTOM_FONT_64
    lv_obj_set_style_text_font(detail_value, &lv_font_montserrat_64, 0);
  #elif defined(LV_FONT_MONTSERRAT_48) && LV_FONT_MONTSERRAT_48
    lv_obj_set_style_text_font(detail_value, &lv_font_montserrat_48, 0);
  #elif defined(LV_FONT_MONTSERRAT_40) && LV_FONT_MONTSERRAT_40
    lv_obj_set_style_text_font(detail_value, &lv_font_montserrat_40, 0);
  #else
    lv_obj_set_style_text_font(detail_value, LV_FONT_DEFAULT, 0);
  #endif

  detail_symbol = lv_label_create(detail_value_row);
  lv_obj_set_style_bg_opa(detail_symbol, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(detail_symbol, lv_color_hex(0x0B1033), 0);
  lv_obj_set_style_pad_all(detail_symbol, 2, 0);
  #if defined(LV_FONT_MONTSERRAT_28) && LV_FONT_MONTSERRAT_28
    lv_obj_set_style_text_font(detail_symbol, &lv_font_montserrat_28, 0);
  #else
    lv_obj_set_style_text_font(detail_symbol, LV_FONT_DEFAULT, 0);
  #endif

  update_detail_from_slot(selected_slot);
}

#if ENABLE_ALDL
static HardwareSerial ALDLSerial(1);

static inline void aldl_enable_rx(bool en) {
  digitalWrite(ALDL_EN_RX_PIN, en ? LOW : HIGH);
}

static void bitBangSendByte(uint8_t b)
{
  digitalWrite(TX_OE_PIN, LOW);
  digitalWrite(ALDL_TX_PIN, HIGH);
  delayMicroseconds(BIT_TIME_US);
  digitalWrite(ALDL_TX_PIN, LOW);
  delayMicroseconds(BIT_TIME_US);

  for (int i = 0; i < 8; i++) {
    digitalWrite(ALDL_TX_PIN, (b & (1 << i)) ? HIGH : LOW);
    delayMicroseconds(BIT_TIME_US);
  }

  digitalWrite(ALDL_TX_PIN, HIGH);
  delayMicroseconds(BIT_TIME_US);
  digitalWrite(TX_OE_PIN, HIGH);
}

static inline void sendCommandF4()
{
  bitBangSendByte(0xF4);
  bitBangSendByte(0x57);
  bitBangSendByte(0x01);
  bitBangSendByte(0x00);
  bitBangSendByte(0xB4);
}

static inline void sendCommandF5()
{
  bitBangSendByte(0xF5);
  bitBangSendByte(0x57);
  bitBangSendByte(0x01);
  bitBangSendByte(0x00);
  bitBangSendByte(0xB3);
}

static size_t receive_raw_frame(uint8_t* out, size_t maxLen)
{
  if (!out || maxLen == 0) return 0;

  memset(out, 0, maxLen);
  delay(ALDL_RESPONSE_DELAY_MS);

  size_t n = 0;
  uint32_t start = millis();
  uint32_t lastByteMs = start;
  bool gotAny = false;

  while ((millis() - start) < ALDL_RX_WINDOW_MS && n < maxLen) {
    while (ALDLSerial.available() > 0 && n < maxLen) {
      int c = ALDLSerial.read();
      if (c >= 0) {
        out[n++] = (uint8_t)c;
        gotAny = true;
        lastByteMs = millis();
      }
    }

    if (gotAny && (millis() - lastByteMs) >= ALDL_INTERBYTE_TIMEOUT_MS) {
      break;
    }
  }

  return n;
}

static size_t normalize_frame_if_needed(FieldSource src, const uint8_t* raw, size_t rawLen, uint8_t* normalized, size_t normalizedMax)
{
  if (!raw || !normalized) return 0;

  const size_t expect = (src == FIELD_F4) ? F4_VALID_LEN : F5_VALID_LEN;
  if (normalizedMax < expect) return 0;

  if (rawLen == expect) {
    memcpy(normalized, raw, expect);
    return expect;
  }

  if (src == FIELD_F4 && rawLen == 67) {
    memcpy(normalized, raw + 3, F4_VALID_LEN);
    normalized67CountF4++;
    return F4_VALID_LEN;
  }

  if (src == FIELD_F5 && rawLen == 68) {
    memcpy(normalized, raw + 3, F5_VALID_LEN);
    normalized68CountF5++;
    return F5_VALID_LEN;
  }

  if (src == FIELD_F5 && rawLen == 70) {
    memcpy(normalized, raw + 5, F5_VALID_LEN);
    return F5_VALID_LEN;
  }

  return rawLen;
}

static void determineRequiredCommands(bool &needF4, bool &needF5)
{
  needF4 = false;
  needF5 = false;

  for (int i = 0; i < kSlotCount; i++) {
    FieldSource src = getFieldSourceByID(idStore.getID(i));
    if (src == FIELD_F4) needF4 = true;
    else if (src == FIELD_F5) needF5 = true;
  }
}

static void update_readings_from_frames()
{
  static uint32_t last_ui_ms = 0;
  uint32_t now = millis();
  if (now - last_ui_ms < 80) return;
  last_ui_ms = now;

  lvgl_port_lock(-1);
  char buf[24];

  for (int i = 0; i < kSlotCount; i++) {
    const char* id = idStore.getID(i);

    if (!decode_selected_id_to_text(id, buf, sizeof(buf))) continue;

    if (tileValues[i]) {
      const char* cur = lv_label_get_text(tileValues[i]);
      if (!cur || strcmp(cur, buf) != 0) lv_label_set_text(tileValues[i], buf);
    }
    if (list_value_lbl[i]) {
      const char* cur = lv_label_get_text(list_value_lbl[i]);
      if (!cur || strcmp(cur, buf) != 0) lv_label_set_text(list_value_lbl[i], buf);
    }
  }

  if (selected_slot >= 0 && selected_slot < kSlotCount && layout_mode == LAYOUT_LIST_DETAIL) {
    update_detail_from_slot(selected_slot);
  }

  lv_obj_invalidate(lv_scr_act());
  lvgl_port_unlock();
}

static void aldl_init()
{
  pinMode(ALDL_TX_PIN, OUTPUT);
  pinMode(TX_OE_PIN, OUTPUT);
  digitalWrite(ALDL_TX_PIN, HIGH);
  digitalWrite(TX_OE_PIN, HIGH);

  pinMode(ALDL_EN_RX_PIN, OUTPUT);
  aldl_enable_rx(false);

  ALDLSerial.begin(ALDL_BAUD, SERIAL_8N1, ALDL_RX_PIN, -1);

  Serial.printf("ALDL init: bitbang TX=%d OE=%d | UART1 RX=%d EN_RX=%d BAUD=%d\n",
                ALDL_TX_PIN, TX_OE_PIN, ALDL_RX_PIN, ALDL_EN_RX_PIN, ALDL_BAUD);
}


static void poll_one_command(FieldSource src)
{
  aldl_enable_rx(false);
  while (ALDLSerial.available()) (void)ALDLSerial.read();

  if (src == FIELD_F4) sendCommandF4();
  else if (src == FIELD_F5) sendCommandF5();
  else return;

  aldl_enable_rx(true);
  uint8_t raw[FRAME_MAX_LEN] = {0};
  size_t rawLen = receive_raw_frame(raw, sizeof(raw));

  aldl_enable_rx(false);

  update_latest_raw_frame_buffers(raw, rawLen);

  uint8_t normalized[FRAME_MAX_LEN] = {0};
  size_t normLen = normalize_frame_if_needed(src, raw, rawLen, normalized, sizeof(normalized));

  if (!validate_len_for_source(src, normLen)) {
    append_frame_log((src == FIELD_F4) ? "F4_BAD" : "F5_BAD", raw, rawLen);
    rebuild_latest_json();
    return;
  }

  if (src == FIELD_F4) {
    memcpy(latestFrameF4, normalized, F4_VALID_LEN);
    latestFrameF4Len = F4_VALID_LEN;
    latestFrameF4Millis = millis();
    haveValidFrameF4 = true;
    goodFrameCountF4++;
    append_frame_log((rawLen == 67) ? "F4_GOOD64_FROM67" : "F4_GOOD64", raw, rawLen);
  } else {
    memcpy(latestFrameF5, normalized, F5_VALID_LEN);
    latestFrameF5Len = F5_VALID_LEN;
    latestFrameF5Millis = millis();
    haveValidFrameF5 = true;
    goodFrameCountF5++;
    append_frame_log(
      (rawLen == 68) ? "F5_GOOD65_FROM68" :
      (rawLen == 70) ? "F5_GOOD65_FROM70" :
                       "F5_GOOD65",
      raw, rawLen
    );
  }

  rebuild_latest_json();
  update_readings_from_frames();
}

static bool validate_len_for_source(FieldSource src, size_t len)
{
  const size_t expect = (src == FIELD_F4) ? F4_VALID_LEN : F5_VALID_LEN;
  if (len == expect) return true;

  if (len == 0) {
    if (src == FIELD_F4) timeoutCountF4++;
    else if (src == FIELD_F5) timeoutCountF5++;
  } else {
    if (src == FIELD_F4) badLengthCountF4++;
    else if (src == FIELD_F5) badLengthCountF5++;
  }

  if (src == FIELD_F4) lastRejectedLengthF4 = len;
  else if (src == FIELD_F5) lastRejectedLengthF5 = len;
  return false;
}

static void aldl_poll()
{
  static uint32_t last_poll = 0;
  uint32_t now = millis();
  if (now - last_poll < ALDL_POLL_MS) return;
  last_poll = now;

  bool needF4 = false, needF5 = false;
  determineRequiredCommands(needF4, needF5);

  if (needF4) poll_one_command(FIELD_F4);
  if (needF5) poll_one_command(FIELD_F5);
}
#endif

// ----------------- AHT sensor handling -----------------
#if ENABLE_AHT
static const char* aht_err_name(esp_err_t err)
{
  const char* name = esp_err_to_name(err);
  return name ? name : "UNKNOWN";
}

static esp_err_t i2c_probe_address(uint8_t address)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  if (!cmd) return ESP_ERR_NO_MEM;

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
  i2c_master_stop(cmd);
  esp_err_t err = i2c_master_cmd_begin(AHT_I2C_PORT, cmd, pdMS_TO_TICKS(AHT_I2C_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);
  return err;
}

static bool aht_i2c_begin()
{
  if (AHT_DEBUG_VERBOSE) {
    Serial.printf(
      "AHT I2C begin: configuring legacy port=%d SDA=%d SCL=%d freq=%u\n",
      (int)AHT_I2C_PORT,
      AHT_I2C_SDA_PIN,
      AHT_I2C_SCL_PIN,
      (unsigned)AHT_I2C_FREQ_HZ
    );
    Serial.flush();
  }

  i2c_config_t conf = {};
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = (gpio_num_t)AHT_I2C_SDA_PIN;
  conf.scl_io_num = (gpio_num_t)AHT_I2C_SCL_PIN;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = AHT_I2C_FREQ_HZ;

  esp_err_t err = i2c_param_config(AHT_I2C_PORT, &conf);
  if (AHT_DEBUG_VERBOSE) {
    Serial.printf("AHT I2C param_config result=%s (%d)\n", aht_err_name(err), (int)err);
  }
  if (err != ESP_OK) return false;

  err = i2c_driver_install(AHT_I2C_PORT, conf.mode, 0, 0, 0);
  if (AHT_DEBUG_VERBOSE) {
    Serial.printf("AHT I2C driver_install result=%s (%d)\n", aht_err_name(err), (int)err);
  }
  if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) return true;
  return false;
}

static void i2c_scan_aht_bus()
{
  if (!AHT_DEBUG_VERBOSE) return;
  Serial.println("I2C scan starting");
  for (uint8_t address = 0x03; address <= 0x77; address++) {
    esp_err_t err = i2c_probe_address(address);
    if (err == ESP_OK) {
      Serial.printf("I2C found 0x%02X\n", address);
    }
    delay(1);
  }
  Serial.println("I2C scan done");
}

static esp_err_t aht_i2c_write(const char* label, const uint8_t* data, size_t len)
{
  if (AHT_DEBUG_VERBOSE) {
    Serial.printf("AHT I2C begin: %s write addr=0x%02X len=%u timeout=%ums\n",
                  label, AHT_I2C_ADDR_DEFAULT, (unsigned)len, AHT_I2C_TIMEOUT_MS);
    Serial.flush();
  }
  esp_err_t err = i2c_master_write_to_device(
                    AHT_I2C_PORT,
                    AHT_I2C_ADDR_DEFAULT,
                    data,
                    len,
                    pdMS_TO_TICKS(AHT_I2C_TIMEOUT_MS)
                  );
  if (AHT_DEBUG_VERBOSE) {
    Serial.printf("AHT I2C end: %s write result=%s (%d)\n", label, aht_err_name(err), (int)err);
  }
  return err;
}

static esp_err_t aht_i2c_read(const char* label, uint8_t* data, size_t len)
{
  if (AHT_DEBUG_VERBOSE) {
    Serial.printf("AHT I2C begin: %s read addr=0x%02X len=%u timeout=%ums\n",
                  label, AHT_I2C_ADDR_DEFAULT, (unsigned)len, AHT_I2C_TIMEOUT_MS);
    Serial.flush();
  }
  esp_err_t err = i2c_master_read_from_device(
                    AHT_I2C_PORT,
                    AHT_I2C_ADDR_DEFAULT,
                    data,
                    len,
                    pdMS_TO_TICKS(AHT_I2C_TIMEOUT_MS)
                  );
  if (AHT_DEBUG_VERBOSE) {
    Serial.printf("AHT I2C end: %s read result=%s (%d)\n", label, aht_err_name(err), (int)err);
  }
  return err;
}

static bool aht_read_status(const char* label, uint8_t* status)
{
  if (!status) {
    if (AHT_DEBUG_VERBOSE) Serial.printf("AHT status read skipped: %s null buffer\n", label);
    return false;
  }
  esp_err_t err = aht_i2c_read(label, status, 1);
  if (err != ESP_OK) return false;
  if (AHT_DEBUG_VERBOSE) Serial.printf("AHT status: %s 0x%02X\n", label, *status);
  return true;
}

static void aht_print_status_bits(const char* label, uint8_t status)
{
  Serial.printf("AHT status bits: %s busy=%u calibrated=%u raw=0x%02X\n",
                label,
                (status & AHT_STATUS_BUSY) ? 1 : 0,
                (status & AHT_STATUS_CALIBRATED) ? 1 : 0,
                status);
}

static bool aht_wait_not_busy(const char* label, uint32_t timeout_ms)
{
  if (AHT_DEBUG_VERBOSE) {
    Serial.printf("AHT wait starting: %s timeout=%lums\n", label, (unsigned long)timeout_ms);
  }
  uint32_t start = millis();
  uint8_t status = AHT_STATUS_BUSY;

  do {
    if (!aht_read_status(label, &status)) {
      if (AHT_DEBUG_VERBOSE) Serial.printf("AHT wait failed: %s status read failed\n", label);
      return false;
    }
    if (AHT_DEBUG_VERBOSE) aht_print_status_bits(label, status);
    if ((status & AHT_STATUS_BUSY) == 0) {
      if (AHT_DEBUG_VERBOSE) {
        Serial.printf("AHT wait OK: %s ready after %lums\n",
                      label, (unsigned long)(millis() - start));
      }
      return true;
    }
    delay(10);
  } while ((millis() - start) < timeout_ms);

  if (AHT_DEBUG_VERBOSE) {
    Serial.printf("AHT wait failed: %s busy timeout after %lums status=0x%02X\n",
                  label, (unsigned long)timeout_ms, status);
  }
  return false;
}

static bool aht_read_sample(float* tempF, float* humidity)
{
  uint8_t trigger_cmd[3] = { AHT_CMD_TRIGGER, 0x33, 0x00 };
  if (AHT_DEBUG_VERBOSE) Serial.println("AHT measurement: trigger command 0xAC 0x33 0x00");
  if (aht_i2c_write("trigger measurement", trigger_cmd, sizeof(trigger_cmd)) != ESP_OK) {
    if (AHT_DEBUG_VERBOSE) Serial.println("AHT measurement failed: trigger write failed");
    return false;
  }

  if (AHT_DEBUG_VERBOSE) Serial.println("AHT measurement: trigger delay 80ms starting");
  delay(80);
  if (AHT_DEBUG_VERBOSE) Serial.println("AHT measurement: trigger delay complete");
  if (!aht_wait_not_busy("measurement wait", 120)) {
    if (AHT_DEBUG_VERBOSE) Serial.println("AHT measurement failed: sensor stayed busy or status read failed");
    return false;
  }

  uint8_t data[6] = {0};
  if (aht_i2c_read("read measurement", data, sizeof(data)) != ESP_OK) {
    if (AHT_DEBUG_VERBOSE) Serial.println("AHT measurement failed: read measurement failed");
    return false;
  }
  if (AHT_DEBUG_VERBOSE) {
    Serial.printf("AHT measurement raw: %02X %02X %02X %02X %02X %02X\n",
                  data[0], data[1], data[2], data[3], data[4], data[5]);
  }
  if (data[0] & AHT_STATUS_BUSY) {
    if (AHT_DEBUG_VERBOSE) {
      Serial.printf("AHT measurement failed: busy status after read 0x%02X\n", data[0]);
    }
    return false;
  }

  uint32_t rawHumidity = ((uint32_t)data[1] << 12) |
                         ((uint32_t)data[2] << 4) |
                         ((uint32_t)data[3] >> 4);
  uint32_t rawTemp = (((uint32_t)data[3] & 0x0F) << 16) |
                     ((uint32_t)data[4] << 8) |
                     (uint32_t)data[5];

  float humidityPct = ((float)rawHumidity * 100.0f) / 1048576.0f;
  float tempC = (((float)rawTemp * 200.0f) / 1048576.0f) - 50.0f;

  if (tempF) *tempF = (tempC * 9.0f / 5.0f) + 32.0f;
  if (humidity) *humidity = humidityPct;
  if (AHT_DEBUG_VERBOSE) {
    Serial.printf("AHT measurement parsed: tempF=%.1f humidity=%.1f\n",
                  (tempC * 9.0f / 5.0f) + 32.0f, humidityPct);
  }
  return true;
}

static void aht_not_found(const char* reason)
{
  Serial.print("AHT not found");
  if (reason && reason[0]) {
    Serial.print(": ");
    Serial.print(reason);
  }
  Serial.println();
}

static void migrate_legacy_aht_ids()
{
  bool changed = false;
  for (int i = 0; i < kSlotCount; i++) {
    const char* id = idStore.getID(i);
    if (id && (strcmp(id, AHT_OLD_OUTSIDE_TEMP_ID) == 0 ||
               strcmp(id, AHT_OLD_OUTSIDE_HUMIDITY_ID) == 0)) {
      idStore.setID(i, AHT_CABIN_ID);
      changed = true;
    }
  }
  if (changed) idStore.save();
}

static bool aht_is_selected()
{
  for (int i = 0; i < kSlotCount; i++) {
    const char* id = idStore.getID(i);
    if (id && strcmp(id, AHT_CABIN_ID) == 0) {
      return true;
    }
  }
  return false;
}

static void aht_init()
{
  if (AHT_DEBUG_VERBOSE) {
    Serial.println("AHT init starting");
    Serial.printf(
      "AHT probing legacy I2C port=%d addr=0x%02X SDA=%d SCL=%d\n",
      (int)AHT_I2C_PORT,
      AHT_I2C_ADDR_DEFAULT,
      AHT_I2C_SDA_PIN,
      AHT_I2C_SCL_PIN
    );
  }

  if (!aht_i2c_begin()) {
    aht_not_found("AHT I2C bus init failed");
    return;
  }

  i2c_scan_aht_bus();

  uint8_t status = 0;
  uint8_t reset_cmd = AHT_CMD_SOFTRESET;
  uint8_t calibrate_cmd[3] = { AHT_CMD_CALIBRATE, 0x08, 0x00 };

  ahtPresent = false;
  ahtReadFailureCount = 0;
  ahtReadFailed = false;
  ahtReadFailureReported = false;
  externalTempF = 0.0f;
  externalHumidity = 0.0f;

  if (!aht_read_status("initial status", &status)) {
    aht_not_found("status read failed");
    return;
  }
  if (AHT_DEBUG_VERBOSE) aht_print_status_bits("initial status", status);

  if (aht_i2c_write("soft reset", &reset_cmd, 1) != ESP_OK) {
    aht_not_found("soft reset write failed");
    return;
  }

  if (AHT_DEBUG_VERBOSE) Serial.println("AHT init: soft reset delay 20ms starting");
  delay(20);
  if (AHT_DEBUG_VERBOSE) Serial.println("AHT init: soft reset delay complete");
  if (!aht_read_status("post soft reset status", &status)) {
    aht_not_found("post soft reset status read failed");
    return;
  }
  if (AHT_DEBUG_VERBOSE) aht_print_status_bits("post soft reset status", status);

  if (!aht_wait_not_busy("soft reset wait", 250)) {
    aht_not_found("busy after soft reset");
    return;
  }

  if (!aht_read_status("post soft reset wait status", &status)) {
    aht_not_found("post soft reset wait status read failed");
    return;
  }
  if (AHT_DEBUG_VERBOSE) aht_print_status_bits("post soft reset wait status", status);

  if (status & AHT_STATUS_CALIBRATED) {
    if (AHT_DEBUG_VERBOSE) Serial.println("AHT init: calibration bit already set after reset");
  } else {
    if (AHT_DEBUG_VERBOSE) Serial.println("AHT init: calibration command 0xE1 0x08 0x00");
    esp_err_t cal_err = aht_i2c_write("calibrate", calibrate_cmd, sizeof(calibrate_cmd));
    if (AHT_DEBUG_VERBOSE) {
      Serial.printf("AHT init: calibration write final result=%s (%d)\n",
                    aht_err_name(cal_err), (int)cal_err);
    }
    if (cal_err != ESP_OK) {
      if (AHT_DEBUG_VERBOSE) Serial.println("AHT calibration failed: command write did not complete");
      aht_not_found("calibrate write failed");
      return;
    }

    if (!aht_wait_not_busy("calibration wait", 250)) {
      if (AHT_DEBUG_VERBOSE) Serial.println("AHT calibration failed: sensor stayed busy or status read failed");
      aht_not_found("calibration wait failed");
      return;
    }
  }

  if (!aht_read_status("calibration verify", &status)) {
    aht_not_found("calibration verify status read failed");
    return;
  }
  if (AHT_DEBUG_VERBOSE) aht_print_status_bits("calibration verify", status);
  if (status & AHT_STATUS_BUSY) {
    if (AHT_DEBUG_VERBOSE) Serial.printf("AHT calibration failed: still busy status=0x%02X\n", status);
    aht_not_found("calibration verify still busy");
    return;
  }
  if (!(status & AHT_STATUS_CALIBRATED)) {
    if (AHT_DEBUG_VERBOSE) Serial.printf("AHT calibration failed: calibrated bit not set status=0x%02X\n", status);
    aht_not_found("calibration bit not set");
    return;
  }

  if (AHT_DEBUG_VERBOSE) Serial.println("AHT init: validating first measurement before marking present");
  float tempF = 0.0f;
  float humidity = 0.0f;
  if (!aht_read_sample(&tempF, &humidity)) {
    aht_not_found("initial measurement read failed");
    return;
  }

  externalTempF = tempF;
  externalHumidity = humidity;
  ahtReadFailureCount = 0;
  ahtReadFailed = false;
  ahtReadFailureReported = false;
  ahtPresent = true;
  Serial.println("AHT found");
  if (AHT_DEBUG_VERBOSE) {
    Serial.printf("AHT temp=%.1f F humidity=%.1f %%\n", externalTempF, externalHumidity);
  }
}

static void aht_update()
{
  static uint32_t last_read_ms = 0;
  uint32_t now = millis();
  if (!ahtPresent || (now - last_read_ms) < 1000) return;
  last_read_ms = now;

  float tempF = 0.0f;
  float humidity = 0.0f;
  if (!aht_read_sample(&tempF, &humidity)) {
    ahtReadFailureCount++;
    ahtReadFailed = true;
    if (ahtReadFailureCount >= AHT_MAX_READ_FAILURES && !ahtReadFailureReported) {
      Serial.println("AHT read failed");
      ahtReadFailureReported = true;
    }
    rebuild_latest_json();
    update_readings_from_frames();
    return;
  }

  ahtReadFailureCount = 0;
  ahtReadFailed = false;
  ahtReadFailureReported = false;
  externalTempF = tempF;
  externalHumidity = humidity;
  if (AHT_DEBUG_VERBOSE) {
    Serial.printf("AHT temp=%.1f F humidity=%.1f %%\n", externalTempF, externalHumidity);
  }
  rebuild_latest_json();
  update_readings_from_frames();
}
#endif

// ----------------- Setup / Loop -----------------
void setup()
{
  Serial.begin(115200);
  delay(1200);

  layout_mode = LAYOUT_TILES;

  Serial.printf("Dash starting... layout=%s\n", layout_mode == LAYOUT_TILES ? "TILES" : "LIST+DETAIL");
  Serial.printf("Free heap after init: %lu\n", (unsigned long)ESP.getFreeHeap());

  using namespace esp_panel::board;
  Board *board = new Board();
  board->init();

  if (!board->begin()) {
    Serial.println("board->begin() FAILED");
    while (true) delay(1000);
  }

  auto bl = board->getBacklight();
  if (bl) bl->on();

#if ENABLE_AHT
  aht_init();
#endif

  lvgl_port_init(board->getLCD(), board->getTouch());
  lv_disp_set_rotation(lv_disp_get_default(), LV_DISP_ROT_270);  // 90 degrees clockwise from current 180 orientation
  idStore.begin();
#if ENABLE_AHT
  migrate_legacy_aht_ids();
#endif

  lvgl_port_lock(-1);
  build_ui();
  lvgl_port_unlock();

  rebuild_latest_json();

  #if ENABLE_WIFI_WEB
    wifi_init();
  #endif

  #if ENABLE_ALDL
    aldl_init();
  #endif

  Serial.printf("Free heap after init: %lu\n", (unsigned long)ESP.getFreeHeap());
  Serial.println("UI built. Tap items to cycle sensors.");
  Serial.println("Polling logic now follows the six selected cells:");
  Serial.println("- only F4 fields selected -> F4 only");
  Serial.println("- only F5 fields selected -> F5 only");
  Serial.println("- mixed F4/F5 fields selected -> poll both");
}

void loop()
{
  #if ENABLE_WIFI_WEB
    wifi_handle();
  #endif

  #if ENABLE_ALDL
    aldl_poll();
  #endif

#if ENABLE_AHT
  if (aht_is_selected()) {
    aht_update();
  }
#endif

  delay(5);
}
