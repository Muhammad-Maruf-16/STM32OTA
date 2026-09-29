#ifndef MQTT_CONFIG_H
#define MQTT_CONFIG_H

#include <stdint.h>

// =============================================================================
// PATH FILE DI LITTLEFS
// =============================================================================
#define MQTT_CFG_FILE    "/mqtt.cfg"

// =============================================================================
// STRUCT CONFIG MQTT (mirip Tasmota)
// =============================================================================
// Total size dijaga kecil agar muat di stack & buffer 256 byte
typedef struct {
    char     host     [64];   // MQTT Broker IP / hostname
    uint16_t port;            // Default: 1883
    char     user     [32];   // MQTT Username
    char     password [32];   // MQTT Password
    char     topic    [32];   // Base topic (misal: "tasmota/cmnd/stm32")
    char     client_id[32];   // Client ID (misal: "STM32_01")
    uint8_t  enabled;         // 1 = aktif, 0 = nonaktif
    uint8_t  _pad[3];         // padding supaya struct aligned 4 byte
    uint32_t magic;           // 0xABCD1234 = config valid
} MqttConfig_t;

#define MQTT_CONFIG_MAGIC   0xABCD1234U

// =============================================================================
// DEFAULT CONFIG (jika belum pernah disimpan)
// =============================================================================
#define MQTT_DEFAULT_HOST       "192.168.1.1"
#define MQTT_DEFAULT_PORT       1883
#define MQTT_DEFAULT_USER       ""
#define MQTT_DEFAULT_PASSWORD   ""
#define MQTT_DEFAULT_TOPIC      "stm32/cmnd"
#define MQTT_DEFAULT_CLIENT_ID  "STM32_W5500"
#define MQTT_DEFAULT_ENABLED    1

// =============================================================================
// FUNGSI PUBLIK
// =============================================================================

/**
 * @brief  Load config MQTT dari LittleFS.
 *         Jika file belum ada / magic tidak cocok → isi default.
 * @param  cfg  Pointer ke struct output
 * @retval 0 sukses load dari flash, 1 pakai default (file belum ada)
 */
int  MqttConfig_Load(MqttConfig_t *cfg);

/**
 * @brief  Simpan config MQTT ke LittleFS.
 * @param  cfg  Pointer ke struct yang mau disimpan
 * @retval 0 sukses, <0 error LittleFS
 */
int  MqttConfig_Save(const MqttConfig_t *cfg);

/**
 * @brief  Isi struct dengan nilai default.
 * @param  cfg  Pointer ke struct output
 */
void MqttConfig_SetDefault(MqttConfig_t *cfg);

#endif /* MQTT_CONFIG_H */
