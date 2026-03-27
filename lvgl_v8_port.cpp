#include "lvgl_v8_port.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

static SemaphoreHandle_t lvgl_mux = nullptr;
static esp_lcd_touch_handle_t s_touch = nullptr;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf1 = nullptr;
static lv_disp_drv_t disp_drv;
static lv_indev_drv_t indev_drv;

static void my_flush_cb(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  esp_lcd_panel_handle_t panel = (esp_lcd_panel_handle_t)disp->user_data;
  esp_lcd_panel_draw_bitmap(panel, area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_p);
  lv_disp_flush_ready(disp);
}

static void my_touchpad_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
  if (!s_touch) {
    data->state = LV_INDEV_STATE_REL;
    return;
  }

  uint16_t x[1], y[1];
  uint8_t cnt = 0;
  esp_lcd_touch_read_data(s_touch);
  bool touched = esp_lcd_touch_get_coordinates(s_touch, x, y, NULL, &cnt, 1);

  if (touched && cnt > 0) {
    data->state = LV_INDEV_STATE_PR;
    data->point.x = x[0];
    data->point.y = y[0];
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}

static void lvgl_tick_task(void *arg)
{
  (void)arg;
  while (true) {
    lv_tick_inc(2);
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

static void lvgl_task(void *arg)
{
  (void)arg;
  while (true) {
    if (lvgl_mux && xSemaphoreTake(lvgl_mux, portMAX_DELAY) == pdTRUE) {
      lv_timer_handler();
      xSemaphoreGive(lvgl_mux);
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

extern "C" void lvgl_port_init(esp_lcd_panel_handle_t panel_handle, esp_lcd_touch_handle_t touch_handle)
{
  s_touch = touch_handle;
  lv_init();

  lvgl_mux = xSemaphoreCreateMutex();

  const int screenWidth = 800;
  const int screenHeight = 480;
  const int bufLines = 40;

  buf1 = (lv_color_t *)heap_caps_malloc(screenWidth * bufLines * sizeof(lv_color_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!buf1) {
    Serial.println("LVGL buffer alloc failed");
    while (true) delay(1000);
  }

  lv_disp_draw_buf_init(&draw_buf, buf1, NULL, screenWidth * bufLines);

  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_flush_cb;
  disp_drv.draw_buf = &draw_buf;
  disp_drv.user_data = panel_handle;
  lv_disp_drv_register(&disp_drv);

  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  xTaskCreatePinnedToCore(lvgl_tick_task, "lv_tick", 2048, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(lvgl_task, "lv_task", 4096, NULL, 1, NULL, 1);
}

extern "C" void lvgl_port_lock(int timeout_ms)
{
  if (!lvgl_mux) return;
  TickType_t t = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
  xSemaphoreTake(lvgl_mux, t);
}

extern "C" void lvgl_port_unlock(void)
{
  if (!lvgl_mux) return;
  xSemaphoreGive(lvgl_mux);
}
