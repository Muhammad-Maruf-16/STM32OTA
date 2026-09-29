#ifndef LFS_CONFIG_H
#define LFS_CONFIG_H

#include "lfs.h"

// =============================================================================
// AREA LITTLEFS DI SPI FLASH
// 0x00000000 - 0x0002FFFF  → manual (OTA, backup) — tidak disentuh LittleFS
// 0x00030000 - 0x007FFFFF  → LittleFS area
// =============================================================================
#define LFS_FLASH_OFFSET    0x00030000U   // Byte pertama LittleFS di flash fisik
#define LFS_BLOCK_SIZE      4096U         // 4KB per sektor (sesuai W25Qxx)
#define LFS_BLOCK_COUNT     1952U         // (8MB - 192KB) / 4KB
#define LFS_PROG_SIZE       256U          // Page size W25Qxx
#define LFS_READ_SIZE       16U           // Minimal read (bisa lebih kecil dari page)
#define LFS_CACHE_SIZE      256U          // Sama dengan prog_size
#define LFS_LOOKAHEAD_SIZE  16U           // 16 byte = bisa track 128 blok
#define LFS_BLOCK_CYCLES    500           // Wear leveling

// =============================================================================
// GLOBAL — bisa di-extern dari file manapun
// =============================================================================
extern lfs_t littlefs;

// =============================================================================
// FUNGSI PUBLIK
// =============================================================================

/**
 * @brief  Init LittleFS: mount, atau format dulu kalau flash masih kosong.
 * @retval 0 sukses, <0 error LittleFS
 */
int LfsConfig_Init(void);

#endif /* LFS_CONFIG_H */
