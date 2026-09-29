#include "lfs_config.h"
#include "main.h"

// =============================================================================
// FUNGSI FLASH — sudah ada di main.c, tinggal extern
// =============================================================================
extern void Flash_ReadBytes    (uint32_t addr, uint8_t *buf, uint32_t len);
extern void ExtFlash_WritePage (uint32_t addr, const uint8_t *data, uint16_t len);
extern void ExtFlash_EraseSector(uint32_t addr);
extern void ExtFlash_WaitBusy  (void);

// =============================================================================
// GLOBAL LFS INSTANCE — di-extern oleh mqtt_config.c dan web_config.c
// =============================================================================
lfs_t littlefs;

// =============================================================================
// BUFFER STATIS (tidak pakai malloc)
// =============================================================================
static uint8_t s_read_buf     [LFS_CACHE_SIZE];
static uint8_t s_prog_buf     [LFS_CACHE_SIZE];
static uint8_t s_lookahead_buf[LFS_LOOKAHEAD_SIZE];

// =============================================================================
// CALLBACK — dipanggil LittleFS secara internal
// =============================================================================

static int lfs_cb_read(const struct lfs_config *c, lfs_block_t block,
                       lfs_off_t off, void *buffer, lfs_size_t size)
{
    (void)c;
    if (block >= LFS_BLOCK_COUNT) return LFS_ERR_INVAL;

    uint32_t addr = LFS_FLASH_OFFSET + (block * LFS_BLOCK_SIZE) + off;
    Flash_ReadBytes(addr, (uint8_t *)buffer, size);
    return LFS_ERR_OK;
}

static int lfs_cb_prog(const struct lfs_config *c, lfs_block_t block,
                       lfs_off_t off, const void *buffer, lfs_size_t size)
{
    (void)c;
    if (block >= LFS_BLOCK_COUNT) return LFS_ERR_INVAL;

    uint32_t       addr      = LFS_FLASH_OFFSET + (block * LFS_BLOCK_SIZE) + off;
    const uint8_t *data      = (const uint8_t *)buffer;
    uint32_t       remaining = size;

    // Tulis per page 256 byte (limitasi hardware W25Qxx)
    while (remaining > 0) {
        uint16_t page_offset = (uint16_t)(addr & 0xFF);
        uint16_t space       = 256 - page_offset;
        uint16_t write_len   = (remaining > space) ? space : (uint16_t)remaining;

        ExtFlash_WritePage(addr, data, write_len);

        addr      += write_len;
        data      += write_len;
        remaining -= write_len;
    }
    return LFS_ERR_OK;
}

static int lfs_cb_erase(const struct lfs_config *c, lfs_block_t block)
{
    (void)c;
    if (block >= LFS_BLOCK_COUNT) return LFS_ERR_INVAL;

    uint32_t addr = LFS_FLASH_OFFSET + (block * LFS_BLOCK_SIZE);
    ExtFlash_EraseSector(addr);
    return LFS_ERR_OK;
}

static int lfs_cb_sync(const struct lfs_config *c)
{
    (void)c;
    // SPI flash langsung commit saat write, tidak perlu apa-apa
    ExtFlash_WaitBusy();
    return LFS_ERR_OK;
}

// =============================================================================
// STRUCT CONFIG LFS
// =============================================================================
static const struct lfs_config s_lfs_cfg = {
    .read  = lfs_cb_read,
    .prog  = lfs_cb_prog,
    .erase = lfs_cb_erase,
    .sync  = lfs_cb_sync,

    .read_size      = LFS_READ_SIZE,
    .prog_size      = LFS_PROG_SIZE,
    .block_size     = LFS_BLOCK_SIZE,
    .block_count    = LFS_BLOCK_COUNT,
    .cache_size     = LFS_CACHE_SIZE,
    .lookahead_size = LFS_LOOKAHEAD_SIZE,
    .block_cycles   = LFS_BLOCK_CYCLES,

    .read_buffer      = s_read_buf,
    .prog_buffer      = s_prog_buf,
    .lookahead_buffer = s_lookahead_buf,
};

// =============================================================================
// INIT
// =============================================================================
int LfsConfig_Init(void)
{
    int err = lfs_mount(&littlefs, &s_lfs_cfg);

    if (err != LFS_ERR_OK) {
        // Flash belum pernah diformat → format dulu lalu mount lagi
        err = lfs_format(&littlefs, &s_lfs_cfg);
        if (err != LFS_ERR_OK) return err;

        err = lfs_mount(&littlefs, &s_lfs_cfg);
    }

    return err;
}
