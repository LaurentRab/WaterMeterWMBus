#pragma once

// ============================================================
//  Shim ESPHome -> standalone PlatformIO
//  Replaces esphome/core/log.h with no-op stubs to avoid
//  pulling in Arduino.h and its macro pollution (HZ, STATUS, etc.)
//  Source: wmbusmeters (GPL-3.0) via SzczepanLeon/esphome-components
// ============================================================

#define esph_log_d(tag, ...) ((void)0)
#define esph_log_v(tag, ...) ((void)0)
#define esph_log_i(tag, ...) ((void)0)
#define esph_log_w(tag, ...) ((void)0)
#define esph_log_e(tag, ...) ((void)0)

#define ESP_LOGD(tag, ...) ((void)0)
#define ESP_LOGV(tag, ...) ((void)0)
#define ESP_LOGI(tag, ...) ((void)0)
#define ESP_LOGW(tag, ...) ((void)0)
#define ESP_LOGE(tag, ...) ((void)0)
#define ESP_LOGCONFIG(tag, ...) ((void)0)
