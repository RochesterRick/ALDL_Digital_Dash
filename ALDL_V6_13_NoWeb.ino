// V6_11b_len67fix.ino — JC8048W550C (ESP32-8048S050C) DASH UI + ALDL (F4 fields)
// Uses ADS-matched btByteNumber values from ALDLCore.h with 1-based byte numbering.
//
// Applied ALDL rules:
// 1) Good F4 frames are exactly 64 bytes total.
// 2) Reject any frame that is not exactly 64 bytes.
// 3) 63-byte frames are bad/truncated and ignored.
// 4) If a raw receive is 67 bytes, treat it as 3 extra leading bytes + 64-byte frame.
// 5) Diagnostic raw receive is preserved before any normalization.
// 6) Poll timing uses ~40 ms response delay and ~60 ms receive window.
// 7) Last good 64-byte frame is preserved.
// 8) Flow is: flush -> send request -> wait -> receive -> normalize -> validate -> decode -> UI
//
// Byte numbering note:
// ALDLCore.h stores ADS btByteNumber values exactly as shown in the ADS file.
// ADS numbering is 1-based.
// Raw frame access converts with: frame[adsByteNumber - 1]
/*
===== REQUIRED ARDUINO SETTINGS =====
Board: ESP32S3 Dev Module
Flash Size: 16MB
PSRAM: OPI PSRAM
Partition: Huge APP
USB CDC On Boot: Enabled
CPU Freq: 240MHz

Board profile:
- JC8048W550C (ESP32-8048S050C)
- Backlight: GPIO2 HIGH
====================================
*/

#include <Arduino.h>
#include "esp_display_panel.hpp"
#include "lvgl_v8_port.h"

// Optional: custom BIG font (drop montserrat_64.c into this sketch folder)
#define USE_CUSTOM_FONT_64 0
#if USE_CUSTOM_FONT_64
  #include "montserrat_64.c"
  LV_FONT_DECLARE(lv_font_montserrat_64);
#endif

#include "ALDLCore.h"  // COMMAND1_FIELDS (F4 list) + PersistentIDs + findFieldByID

// ----------------- Layout toggle pin -----------------
#define MODE_PIN 20

// ----------------- ALDL pins (bit-bang TX + UART RX) -----------------
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
#define FRAME_VALID_LEN        64
#define FRAME_LOG_CAPACITY     50

// ----------------- WiFi / Web (SoftAP) -----------------
#define ENABLE_WIFI_WEB 0
#if ENABLE_WIFI_WEB
static const char* AP_SSID = "ALDL-Dash";
static const char* AP_PASS = "12345678";
static WebServer server(80);
#endif

// ----------------- Per-slot color theme -----------------
static const uint32_t TILE_ACCENT_HEX[6] = {
  0x00E5FF, 0xFFD400, 0x00E5FF, 0xFF2D95, 0x00E5FF, 0xB084FF
};
static const uint32_t TILE_VALUE_HEX[6] = {
  0x39FF14, 0xFFD400, 0x5DF2FF, 0xFF69B4, 0x5DF2FF, 0xB084FF
};

// ----------------- Globals -----------------
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

// Latest validated ALDL frame (exact 64 bytes only)
static uint8_t latestFrame[FRAME_VALID_LEN] = {0};
static size_t  latestFrameLen = 0;
static char    latestFrameHex[FRAME_VALID_LEN * 3 + 1] = {0};
static char    latestJson[1600] = "{\"status\":\"no data yet\"}";
static uint32_t latestFrameMillis = 0;
static bool    haveValidFrame = false;

// Last raw receive attempt (may be invalid length, kept for diagnostics)
static uint8_t latestRawFrame[FRAME_MAX_LEN] = {0};
static size_t  latestRawFrameLen = 0;
static char    latestRawFrameHex[FRAME_MAX_LEN * 3 + 1] = {0};

// Last good full raw 64-byte frame
static uint8_t lastGoodRaw64[FRAME_VALID_LEN] = {0};
static char    lastGoodRaw64Hex[FRAME_VALID_LEN * 3 + 1] = {0};
static uint32_t lastGoodFrameMillis = 0;

// Diagnostics
static uint32_t goodFrameCount = 0;
static uint32_t badLengthCount = 0;
static uint32_t bad63Count = 0;
static uint32_t timeoutCount = 0;
static size_t   lastRejectedLength = 0;
static uint32_t normalized67Count = 0;

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
static void update_latest_frame_buffers(const uint8_t* frame, size_t len);
static void update_last_good_frame_buffers(const uint8_t* frame, size_t len);
static void tile_event_cb(lv_event_t *e);
static void list_item_event_cb(lv_event_t *e);
static int  cmdf4_index_of_id(const char* id);
static void advance_saved_field(int slot_idx);

static lv_coord_t symbol_gap_px_for_font(const lv_font_t* vf);
static void set_row_gap_by_value_label(lv_obj_t* row, lv_obj_t* value_label);

static void apply_selection_style_tiles(int idx);
static void apply_selection_style_list(int idx);
static void update_slot_labels_from_id(int idx);
static void update_detail_from_slot(int idx);
static void handle_mode_button();

static void rebuild_latest_json();
static void append_frame_log(const char* tag, const uint8_t* frame, size_t len);
static const DataField* find_field_by_name_fragment(const char* needle);
static bool decode_field_to_text(const DataField* f, char* out, size_t outSize);
static bool decode_field_to_text_from_frame(const DataField* f, const uint8_t* frame, size_t len, char* out, size_t outSize);
static void update_readings_from_validated_frame(const uint8_t *frame, size_t len);
static bool validate_frame_len(size_t len);
static size_t receive_raw_frame(uint8_t* out, size_t maxLen);
static size_t normalize_frame_if_needed(const uint8_t* raw, size_t rawLen, uint8_t* normalized, size_t normalizedMax);

#if ENABLE_WIFI_WEB
static void wifi_init();
static void wifi_handle();
static void handleRoot();
static void handleData();
static void handleJson();
static void handleLog();
#endif

// ----------------- Helpers -----------------
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

static int cmdf4_index_of_id(const char* id)
{
  if (!id) return -1;
  for (size_t i = 0; i < COMMAND1_FIELD_COUNT; i++) {
    if (strcmp(id, COMMAND1_FIELDS[i].id) == 0) return (int)i;
  }
  return -1;
}

static void update_slot_labels_from_id(int idx)
{
  if (idx < 0 || idx >= kSlotCount) return;
  const char* id = idStore.getID(idx);
  const DataField* f = (const DataField*)findFieldByID(id, COMMAND1_FIELDS, COMMAND1_FIELD_COUNT);

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

  //if (valueRows[idx] && tileValues[idx]) set_row_gap_by_value_label(valueRows[idx], tileValues[idx]);
  //if (list_value_row[idx] && list_value_lbl[idx]) set_row_gap_by_value_label(list_value_row[idx], list_value_lbl[idx]);
}

static void update_detail_from_slot(int idx)
{
  if (!detail_name || !detail_value || !detail_symbol || !detail_value_row) return;

  const char* id = idStore.getID(idx);
  const DataField* f = (const DataField*)findFieldByID(id, COMMAND1_FIELDS, COMMAND1_FIELD_COUNT);

  const char* name = (f && f->name) ? f->name : (id ? id : "-");
  const char* sym  = (f && f->symbol && f->symbol[0]) ? f->symbol : "";

  lv_label_set_text(detail_name, name);

  const char* vtxt = (list_value_lbl[idx]) ? lv_label_get_text(list_value_lbl[idx]) : "--";
  lv_label_set_text(detail_value, vtxt ? vtxt : "--");

  lv_label_set_text(detail_symbol, sym);
  if (!sym || sym[0] == '\0') lv_obj_add_flag(detail_symbol, LV_OBJ_FLAG_HIDDEN);
  else                        lv_obj_clear_flag(detail_symbol, LV_OBJ_FLAG_HIDDEN);

  //set_row_gap_by_value_label(detail_value_row, detail_value);

  lv_obj_set_style_text_color(detail_name,   lv_color_hex(TILE_ACCENT_HEX[idx]), 0);
  lv_obj_set_style_text_color(detail_symbol, lv_color_hex(TILE_ACCENT_HEX[idx]), 0);
  lv_obj_set_style_text_color(detail_value,  lv_color_hex(TILE_VALUE_HEX[idx]),  0);
}

static void advance_saved_field(int slot_idx)
{
  const char* cur_id = idStore.getID(slot_idx);
  int cur = cmdf4_index_of_id(cur_id);

  int next = (cur < 0) ? 0 : (cur + 1);
  if (next >= (int)COMMAND1_FIELD_COUNT) next = 0;

  const DataField &f = COMMAND1_FIELDS[next];
  idStore.setID(slot_idx, f.id);
  idStore.save();

  update_slot_labels_from_id(slot_idx);

  if (tileValues[slot_idx]) lv_label_set_text(tileValues[slot_idx], "--");
  if (list_value_lbl[slot_idx]) lv_label_set_text(list_value_lbl[slot_idx], "--");

  if (valueRows[slot_idx]) {
    lv_obj_update_layout(valueRows[slot_idx]);
    lv_obj_align(valueRows[slot_idx], LV_ALIGN_CENTER, 0, 14);
  }

  if (haveValidFrame) {
    update_readings_from_validated_frame(latestFrame, latestFrameLen);
  }

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
static void update_latest_frame_buffers(const uint8_t* frame, size_t len)
{
  if (!frame || len != FRAME_VALID_LEN) return;

  memcpy(latestFrame, frame, FRAME_VALID_LEN);
  latestFrameLen = FRAME_VALID_LEN;
  latestFrameMillis = millis();
  haveValidFrame = true;

  size_t pos = 0;
  latestFrameHex[0] = '\0';
  for (size_t i = 0; i < FRAME_VALID_LEN; i++) {
    int written = snprintf(&latestFrameHex[pos], sizeof(latestFrameHex) - pos,
                           (i + 1 < FRAME_VALID_LEN) ? "%02X " : "%02X", frame[i]);
    if (written <= 0) break;
    pos += (size_t)written;
    if (pos >= sizeof(latestFrameHex) - 1) break;
  }
}

static void update_last_good_frame_buffers(const uint8_t* frame, size_t len)
{
  if (!frame || len != FRAME_VALID_LEN) return;

  memcpy(lastGoodRaw64, frame, FRAME_VALID_LEN);
  lastGoodFrameMillis = millis();

  size_t pos = 0;
  lastGoodRaw64Hex[0] = '\0';
  for (size_t i = 0; i < FRAME_VALID_LEN; i++) {
    int written = snprintf(&lastGoodRaw64Hex[pos], sizeof(lastGoodRaw64Hex) - pos,
                           (i + 1 < FRAME_VALID_LEN) ? "%02X " : "%02X", frame[i]);
    if (written <= 0) break;
    pos += (size_t)written;
    if (pos >= sizeof(lastGoodRaw64Hex) - 1) break;
  }
}

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
  float bpw = atof(bpwBuf);   // milliseconds
  float rpm = atof(rpmBuf);

  if (mph < 1.0f || bpw <= 0.0f || rpm <= 0.0f) {
    snprintf(out, outSize, "0.0");
    return true;
  }

  // Simple starting estimate for TBI/TPI style GM ECM data.
  // This constant will likely need tuning to your truck.
  const float K = 11250.0f;

  float mpg = mph / (bpw * rpm / K);
  mpg *= 0.80f;

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

  // ADS btByteNumber is 1-based position, convert to 0-based array index
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

static bool decode_field_to_text(const DataField* f, char* out, size_t outSize)
{
  if (!haveValidFrame) {
    if (out && outSize) strncpy(out, "--", outSize);
    if (out && outSize) out[outSize - 1] = '\0';
    return false;
  }
  return decode_field_to_text_from_frame(f, latestFrame, latestFrameLen, out, outSize);
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

  decode_field_to_text(rpmF, rpmBuf, sizeof(rpmBuf));
  decode_field_to_text(tpsF, tpsBuf, sizeof(tpsBuf));
  decode_field_to_text(battF, battBuf, sizeof(battBuf));
  decode_field_to_text(tempF, tempBuf, sizeof(tempBuf));

  size_t pos = 0;
  int n = snprintf(latestJson + pos, sizeof(latestJson) - pos,
                 "{\"ms\":%lu,\"selected_slot\":%d,\"valid\":%s,"
                 "\"anchors\":{\"RPM\":\"%s\",\"TPS\":\"%s\",\"Batt\":\"%s\",\"Temp\":\"%s\"},\"slots\":[",
                 (unsigned long)latestFrameMillis,
                 selected_slot,
                 haveValidFrame ? "true" : "false",
                 rpmBuf, tpsBuf, battBuf, tempBuf);
  if (n < 0) return;
  pos += (size_t)n;
  if (pos >= sizeof(latestJson)) {
    latestJson[sizeof(latestJson) - 1] = '\0';
    return;
  }

  for (int i = 0; i < kSlotCount; i++) {
    const char* id = idStore.getID(i);
    const DataField* f = (const DataField*)findFieldByID(id, COMMAND1_FIELDS, COMMAND1_FIELD_COUNT);
    const char* name = (f && f->name) ? f->name : "-";
    const char* symbol = (f && f->symbol) ? f->symbol : "";

    char valueBuf[24] = "--";
    decode_field_to_text(f, valueBuf, sizeof(valueBuf));

    n = snprintf(latestJson + pos, sizeof(latestJson) - pos,
                 "%s{\"slot\":%d,\"name\":\"%s\",\"value\":\"%s\",\"symbol\":\"%s\"}",
                 (i == 0) ? "" : ",",
                 i, name, valueBuf, symbol);
    if (n < 0) return;
    pos += (size_t)n;
    if (pos >= sizeof(latestJson)) {
      latestJson[sizeof(latestJson) - 1] = '\0';
      return;
    }
  }

  n = snprintf(latestJson + pos, sizeof(latestJson) - pos,
               "],\"raw\":\"%s\",\"raw_len\":%u,"
               "\"last_rx_raw\":\"%s\",\"last_rx_len\":%u,"
               "\"good_frames\":%lu,\"bad_len\":%lu,\"bad63\":%lu,\"timeouts\":%lu,"
               "\"normalized67\":%lu,\"last_rejected_len\":%u,\"log_count\":%u}",
               haveValidFrame ? lastGoodRaw64Hex : "",
               haveValidFrame ? (unsigned)FRAME_VALID_LEN : 0,
               latestRawFrameHex,
               (unsigned)latestRawFrameLen,
               (unsigned long)goodFrameCount,
               (unsigned long)badLengthCount,
               (unsigned long)bad63Count,
               (unsigned long)timeoutCount,
               (unsigned long)normalized67Count,
               (unsigned)lastRejectedLength,
               (unsigned)frameLogCount);
  if (n < 0) return;
  if ((size_t)n >= sizeof(latestJson) - pos) latestJson[sizeof(latestJson) - 1] = '\0';
}

#if ENABLE_WIFI_WEB
// ----------------- Web server -----------------
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
  <div class="card"><div class="small">Latest validated raw 64-byte F4 frame</div><div id="raw" class="raw">waiting...</div></div>
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
          <div class="row"><div class="name">Stats</div><div class="val">good=${j.good_frames||0} badLen=${j.bad_len||0} bad63=${j.bad63||0} norm67=${j.normalized67||0}</div></div>`;

        let html = '';
        for (const s of (j.slots || [])) {
          html += `<div class="row"><div class="name">${s.name}</div><div class="val">${s.value} ${s.symbol || ''}</div></div>`;
        }
        document.getElementById('slots').innerHTML = html || 'no slots';
        document.getElementById('raw').textContent = `[len=${j.raw_len || 0}] ` + (j.raw || 'no validated frame');
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
  server.send(200, "text/plain", haveValidFrame ? lastGoodRaw64Hex : "no validated 64-byte frame yet");
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

  if (haveValidFrame) {
    update_readings_from_validated_frame(latestFrame, latestFrameLen);
  }
}

static void build_ui_tiles()
{
  lv_obj_t *scr = lv_scr_act();
  static lv_coord_t col_dsc[] = { LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST };
  static lv_coord_t row_dsc[] = { LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST };

  lv_obj_set_grid_dsc_array(scr, col_dsc, row_dsc);
  lv_obj_set_layout(scr, LV_LAYOUT_GRID);
  lv_obj_set_style_pad_row(scr, 16, 0);
  lv_obj_set_style_pad_column(scr, 16, 0);
  lv_obj_set_style_pad_all(scr, 16, 0);

  for (int i = 0; i < kSlotCount; i++) {
    lv_obj_t *tile = lv_obj_create(scr);
    tiles[i] = tile;
    valueRows[i] = nullptr;  // no shared value/symbol row in tile layout now

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

    int r = (i / 3);
    int c = (i % 3);
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
    //set_row_gap_by_value_label(list_value_row[i], list_value_lbl[i]);
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

// ----------------- Mode switch handling -----------------
static void handle_mode_button()
{
  bool raw_pressed = (digitalRead(MODE_PIN) == LOW);
  static bool last_raw = false;
  static bool stable = false;
  static uint32_t last_change_ms = 0;
  uint32_t now = millis();

  if (raw_pressed != last_raw) {
    last_raw = raw_pressed;
    last_change_ms = now;
  }

  if ((now - last_change_ms) >= 30) {
    if (stable != last_raw) {
      stable = last_raw;
      if (stable) {
        layout_mode = (layout_mode == LAYOUT_TILES) ? LAYOUT_LIST_DETAIL : LAYOUT_TILES;
        lvgl_port_lock(-1);
        build_ui();
        lvgl_port_unlock();
        Serial.printf("Layout toggled -> %s\n", layout_mode == LAYOUT_TILES ? "TILES" : "LIST+DETAIL");
      }
    }
  }
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
  // Keep the currently used request bytes from your working sketch
  bitBangSendByte(0xF4);
  bitBangSendByte(0x57);
  bitBangSendByte(0x01);
  bitBangSendByte(0x00);
  bitBangSendByte(0xB4);
}

static bool validate_frame_len(size_t len)
{
  if (len == FRAME_VALID_LEN) return true;

  if (len == 63) bad63Count++;
  else if (len == 0) timeoutCount++;
  else badLengthCount++;

  lastRejectedLength = len;
  return false;
}

static size_t receive_raw_frame(uint8_t* out, size_t maxLen)
{
  if (!out || maxLen == 0) return 0;

  memset(out, 0, maxLen);

  // confirmed theory: response starts roughly 40 ms after request
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


static size_t normalize_frame_if_needed(const uint8_t* raw, size_t rawLen, uint8_t* normalized, size_t normalizedMax)
{
  if (!raw || !normalized || normalizedMax < FRAME_VALID_LEN) return 0;

  if (rawLen == FRAME_VALID_LEN) {
    memcpy(normalized, raw, FRAME_VALID_LEN);
    return FRAME_VALID_LEN;
  }

  // Current truck testing suggests some receives come in as 67 bytes:
  // 3 extra leading bytes followed by the real 64-byte frame.
  if (rawLen == 67) {
    memcpy(normalized, raw + 3, FRAME_VALID_LEN);
    normalized67Count++;
    Serial.printf("ALDL len67 normalized -> 64 by skipping first 3 bytes: %02X %02X %02X\n",
                  raw[0], raw[1], raw[2]);
    return FRAME_VALID_LEN;
  }

  return rawLen;
}

static void update_readings_from_validated_frame(const uint8_t *frame, size_t len)
{
  if (!frame || len != FRAME_VALID_LEN) return;

  static uint32_t last_ui_ms = 0;
  uint32_t now = millis();
  if (now - last_ui_ms < 80) return;
  last_ui_ms = now;

  lvgl_port_lock(-1);
  char buf[24];

  for (int i = 0; i < kSlotCount; i++) {
    const char* id = idStore.getID(i);
    const DataField* f = (const DataField*)findFieldByID(id, COMMAND1_FIELDS, COMMAND1_FIELD_COUNT);
    if (!f) continue;

    bool usedOverride = false;

    if (!usedOverride) {
      if (!decode_field_to_text_from_frame(f, frame, len, buf, sizeof(buf))) continue;
    }

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
  Serial.printf("ALDL theory: valid=%d bytes, response delay=%d ms, rx window=%d ms\n",
                FRAME_VALID_LEN, ALDL_RESPONSE_DELAY_MS, ALDL_RX_WINDOW_MS);
}

static void aldl_poll()
{
  static uint32_t last_poll = 0;
  uint32_t now = millis();
  if (now - last_poll < ALDL_POLL_MS) return;
  last_poll = now;

  // 1) Flush old UART bytes before sending a new request
  aldl_enable_rx(false);
  while (ALDLSerial.available()) (void)ALDLSerial.read();

  // 2) Send F4 request
  sendCommandF4();

  // 3) Switch to receive and capture raw wire frame
  aldl_enable_rx(true);
  uint8_t raw[FRAME_MAX_LEN] = {0};
  size_t n = receive_raw_frame(raw, sizeof(raw));
  aldl_enable_rx(false);

  update_latest_raw_frame_buffers(raw, n);

  // 4) Normalize known-good raw variations before validation
  uint8_t normalized[FRAME_VALID_LEN] = {0};
  size_t normalizedLen = normalize_frame_if_needed(raw, n, normalized, sizeof(normalized));

  // 5) Accept only exact 64-byte normalized frames
  if (!validate_frame_len(normalizedLen)) {
    append_frame_log((normalizedLen == 63) ? "BAD63" : ((normalizedLen == 0) ? "TIMEOUT" : "BADLEN"), raw, n);
    rebuild_latest_json();
    Serial.printf("ALDL rejected rawLen=%u normLen=%u (good=%lu badLen=%lu bad63=%lu timeout=%lu norm67=%lu)\n",
                  (unsigned)n,
                  (unsigned)normalizedLen,
                  (unsigned long)goodFrameCount,
                  (unsigned long)badLengthCount,
                  (unsigned long)bad63Count,
                  (unsigned long)timeoutCount,
                  (unsigned long)normalized67Count);
    return;
  }

  // 6) Keep last good full normalized 64-byte frame
  goodFrameCount++;
  update_last_good_frame_buffers(normalized, FRAME_VALID_LEN);
  update_latest_frame_buffers(normalized, FRAME_VALID_LEN);
  append_frame_log((n == 67) ? "GOOD64_FROM67" : "GOOD64", raw, n);

  // 7) Decode fields from validated frame
  rebuild_latest_json();

  // 8) Update UI
  update_readings_from_validated_frame(normalized, FRAME_VALID_LEN);

  Serial.printf("ALDL good64  t=%lu  rawLen=%u normLen=%u\n",
              (unsigned long)latestFrameMillis,
              (unsigned)n,
              (unsigned)FRAME_VALID_LEN);
  }
#endif

// ----------------- Setup / Loop -----------------
void setup()
{
  Serial.begin(115200);
  delay(200);

  pinMode(MODE_PIN, INPUT_PULLUP);
  layout_mode = LAYOUT_TILES;

  Serial.printf("Dash starting... MODE_PIN=%d -> %s\n", MODE_PIN, layout_mode == LAYOUT_TILES ? "TILES" : "LIST+DETAIL");
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

  lvgl_port_init(board->getLCD(), board->getTouch());
  lv_disp_set_rotation(lv_disp_get_default(), LV_DISP_ROT_180);
  idStore.begin();

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
  Serial.println("UI built. Tap items to cycle sensors. Press MODE_PIN to toggle layouts.");
  Serial.println("Important: this sketch accepts exact 64-byte frames, and also normalizes 67-byte receives by skipping the first 3 bytes.");
  Serial.println("Any field offsets inherited from old USB/header-based ADX assumptions should be rechecked.");
  #if ENABLE_WIFI_WEB
    Serial.println("Web AP: SSID=ALDL-Dash PASS=12345678 URL=http://192.168.4.1/");
  #endif
}

void loop()
{
  handle_mode_button();

  #if ENABLE_WIFI_WEB
    wifi_handle();
  #endif

  #if ENABLE_ALDL
    aldl_poll();
  #endif

  delay(5);
}
