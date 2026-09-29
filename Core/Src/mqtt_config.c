#include "mqtt_config.h"
#include "lfs_config.h"
#include <string.h>

// =============================================================================
// FUNGSI INTERNAL
// =============================================================================

static int lfs_read_file(const char *path, void *buf, lfs_size_t size)
{
    lfs_file_t file;

    int err = lfs_file_open(&littlefs, &file, path, LFS_O_RDONLY);
    if (err < 0) return err;

    lfs_ssize_t read = lfs_file_read(&littlefs, &file, buf, size);
    lfs_file_close(&littlefs, &file);

    return (read >= 0) ? LFS_ERR_OK : (int)read;
}

static int lfs_write_file(const char *path, const void *buf, lfs_size_t size)
{
    lfs_file_t file;

    int err = lfs_file_open(&littlefs, &file, path,
                            LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
    if (err < 0) return err;

    lfs_ssize_t written = lfs_file_write(&littlefs, &file, buf, size);
    lfs_file_close(&littlefs, &file);

    return (written >= 0) ? LFS_ERR_OK : (int)written;
}

// =============================================================================
// FUNGSI PUBLIK
// =============================================================================

void MqttConfig_SetDefault(MqttConfig_t *cfg)
{
    memset(cfg, 0, sizeof(MqttConfig_t));

    strncpy(cfg->host,      MQTT_DEFAULT_HOST,      sizeof(cfg->host)      - 1);
    strncpy(cfg->user,      MQTT_DEFAULT_USER,      sizeof(cfg->user)      - 1);
    strncpy(cfg->password,  MQTT_DEFAULT_PASSWORD,  sizeof(cfg->password)  - 1);
    strncpy(cfg->topic,     MQTT_DEFAULT_TOPIC,     sizeof(cfg->topic)     - 1);
    strncpy(cfg->client_id, MQTT_DEFAULT_CLIENT_ID, sizeof(cfg->client_id) - 1);

    cfg->port    = MQTT_DEFAULT_PORT;
    cfg->enabled = MQTT_DEFAULT_ENABLED;
    cfg->magic   = MQTT_CONFIG_MAGIC;
}

int MqttConfig_Load(MqttConfig_t *cfg)
{
    MqttConfig_t tmp;
    memset(&tmp, 0, sizeof(tmp));

    int err = lfs_read_file(MQTT_CFG_FILE, &tmp, sizeof(MqttConfig_t));

    if (err == LFS_ERR_OK && tmp.magic == MQTT_CONFIG_MAGIC) {
        memcpy(cfg, &tmp, sizeof(MqttConfig_t));
        return 0;
    }

    // File belum ada / corrupt → pakai default
    MqttConfig_SetDefault(cfg);
    return 1;
}

int MqttConfig_Save(const MqttConfig_t *cfg)
{
    return lfs_write_file(MQTT_CFG_FILE, cfg, sizeof(MqttConfig_t));
}
