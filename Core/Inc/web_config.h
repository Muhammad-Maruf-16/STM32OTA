#ifndef WEB_CONFIG_H
#define WEB_CONFIG_H

#include <stdint.h>

// Path file HTML di LittleFS
#define CONFIG_HTML_FILE    "/config.html"

// =============================================================================
// FUNGSI PUBLIK
// =============================================================================

/**
 * @brief  Inisialisasi web config:
 *         - Cek apakah /config.html sudah ada di LittleFS
 *         - Kalau belum, tulis dari ROM (HTML_CONFIG_PAGE[])
 */
void WebConfig_Init(void);

/**
 * @brief  Handle request HTTP untuk /config.
 *         - GET  /config  → streaming HTML dari LittleFS
 *         - POST /config  → parse form, simpan ke /mqtt.cfg, redirect
 *
 * @param  rx_buf   Buffer yang sudah dibaca dari W5500 RX
 * @param  rx_len   Panjang data di buffer
 */
void WebConfig_Handle(uint8_t *rx_buf, uint16_t rx_len);

#endif /* WEB_CONFIG_H */
