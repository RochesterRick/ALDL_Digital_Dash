#ifndef LV_CONF_H
#define LV_CONF_H
#warning "USING SKETCH lv_conf.h"

/*====================
 * COLOR / MEMORY
 *====================*/
#define LV_COLOR_DEPTH 16
#define LV_MEM_CUSTOM 0
#define LV_USE_LOG 0

/*====================
 * FONTS (ENABLE THESE)
 *====================*/
#define LV_FONT_MONTSERRAT_22  1
#define LV_FONT_MONTSERRAT_28  1
#define LV_FONT_MONTSERRAT_40  1
#define LV_FONT_MONTSERRAT_48  1
#define LV_FONT_MONTSERRAT_56  1
#define LV_FONT_MONTSERRAT_64  1

/*====================
 * CORE DRAW
 *====================*/
#define LV_USE_DRAW_SW 1
#define LV_USE_TICK_CUSTOM 0

/*====================
 * DISABLE DESKTOP / GPU STUFF
 *====================*/
#define LV_USE_DRAW_VG_LITE 0
#define LV_USE_LINUX_DRM 0
#define LV_USE_LINUX_FBDEV 0
#define LV_USE_EVDEV 0

#endif
