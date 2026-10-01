/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "dhcp.h"
#include "mqtt_config.h"
#include "web_config.h"
#include "mqtt_client.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define OTA_MODE_BACKUP
//#define MAIN_DEBUG
#define DEBUG_MQTT
#define APP_START_ADDR        0x08001000U
#define MAX_FIRMWARE_SIZE     (28 * 1024)

#define FLASH_BACKUP_META     0x00010000U
#define FLASH_BACKUP_BIN      0x00011000U

#define FLASH_STAGING_META    0x00020000U
#define FLASH_STAGING_BIN     0x00021000U

#define W5500_COMMON_REG_OP   0x00
#define W5500_S0_REG_OP       0x08
#define W5500_S0_TX_OP        0x10
#define W5500_S0_RX_OP        0x18

#define Sn_MR                 0x0000
#define Sn_CR                 0x0001
#define Sn_IR                 0x0002
#define Sn_SR                 0x0003
#define Sn_PORT               0x0004
#define Sn_TX_FSR             0x0020
#define Sn_TX_RD              0x0022
#define Sn_TX_WR              0x0024
#define Sn_RX_RSR             0x0026
#define Sn_RX_RD              0x0028

#define SOCK_CLOSED           0x00
#define SOCK_INIT             0x13
#define SOCK_LISTEN           0x14
#define SOCK_ESTABLISHED      0x17
#define SOCK_CLOSE_WAIT       0x1C
#define SOCK_FIN_WAIT         0x18

#define CMD_OPEN              0x01
#define CMD_LISTEN            0x02
#define CMD_DISCON            0x08
#define CMD_CLOSE             0x10
#define CMD_SEND              0x20
#define CMD_RECV              0x40

#define W5500_S0_BUF_MASK     0x1FFF
#define CHUNK_BUFFER_SIZE     (256)
static uint8_t ram_buffer[CHUNK_BUFFER_SIZE];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint32_t g_ms_tick = 0;
uint8_t g_phy_reg = 0;
#ifdef MAIN_DEBUG
uint8_t g_dbg_s0_test = 0;
uint8_t g_dbg_s1_test = 0;
uint8_t g_dbg_test2   = 0;
uint8_t g_dbg_test3   = 0;
#endif

#ifdef DEBUG_MQTT
uint8_t g_dbg_s2_tx_size = 0;
uint8_t g_dbg_s2_sr     = 0;
uint8_t g_dbg_s2_fsr_hi = 0;
uint8_t g_dbg_s2_fsr_lo = 0;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_CRC_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ==============================================================================
// 1. DRIVER SPI1 - FLASH EKSTERNAL
// ==============================================================================
static inline uint8_t SPI1_Transfer(uint8_t data)
{
  while (!LL_SPI_IsActiveFlag_TXE(SPI1));
  LL_SPI_TransmitData8(SPI1, data);
  while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
  return LL_SPI_ReceiveData8(SPI1);
}

void ExtFlash_WaitBusy(void)
{
  uint8_t status;
  do {
    LL_GPIO_ResetOutputPin(FLASH_CS_GPIO_Port, FLASH_CS_Pin);
    SPI1_Transfer(0x05);
    status = SPI1_Transfer(0xFF);
    LL_GPIO_SetOutputPin(FLASH_CS_GPIO_Port, FLASH_CS_Pin);
  } while (status & 0x01);
}

void ExtFlash_WriteEnable(void)
{
  LL_GPIO_ResetOutputPin(FLASH_CS_GPIO_Port, FLASH_CS_Pin);
  SPI1_Transfer(0x06);
  LL_GPIO_SetOutputPin(FLASH_CS_GPIO_Port, FLASH_CS_Pin);
}

void ExtFlash_EraseSector(uint32_t addr)
{
  ExtFlash_WriteEnable();
  LL_GPIO_ResetOutputPin(FLASH_CS_GPIO_Port, FLASH_CS_Pin);
  SPI1_Transfer(0x20);
  SPI1_Transfer((uint8_t)(addr >> 16));
  SPI1_Transfer((uint8_t)(addr >> 8));
  SPI1_Transfer((uint8_t)addr);
  LL_GPIO_SetOutputPin(FLASH_CS_GPIO_Port, FLASH_CS_Pin);
  ExtFlash_WaitBusy();
}

void ExtFlash_WritePage(uint32_t addr, const uint8_t *data, uint16_t len)
{
  ExtFlash_WriteEnable();
  LL_GPIO_ResetOutputPin(FLASH_CS_GPIO_Port, FLASH_CS_Pin);
  SPI1_Transfer(0x02);
  SPI1_Transfer((uint8_t)(addr >> 16));
  SPI1_Transfer((uint8_t)(addr >> 8));
  SPI1_Transfer((uint8_t)addr);
  for (uint16_t i = 0; i < len; i++) SPI1_Transfer(data[i]);
  LL_GPIO_SetOutputPin(FLASH_CS_GPIO_Port, FLASH_CS_Pin);
  ExtFlash_WaitBusy();
}

void Flash_ReadBytes(uint32_t addr, uint8_t *buf, uint32_t len)
{
  LL_GPIO_ResetOutputPin(FLASH_CS_GPIO_Port, FLASH_CS_Pin);
  SPI1_Transfer(0x03);
  SPI1_Transfer((uint8_t)(addr >> 16));
  SPI1_Transfer((uint8_t)(addr >> 8));
  SPI1_Transfer((uint8_t)addr);
  for (uint32_t i = 0; i < len; i++) buf[i] = SPI1_Transfer(0xFF);
  LL_GPIO_SetOutputPin(FLASH_CS_GPIO_Port, FLASH_CS_Pin);
}

uint32_t Hitung_CRC32_LL(const uint32_t *pData32, uint32_t totalWord)
{
  CRC->CR = 1;
  while (totalWord--) CRC->DR = *pData32++;
  return CRC->DR;
}

// ==============================================================================
// 2. FUNGSI SELF-BACKUP
// ==============================================================================
uint8_t Backup_CurrentFirmwareToExtFlash(uint32_t bin_size)
{
  if (bin_size == 0 || bin_size > MAX_FIRMWARE_SIZE) return 0;

  uint32_t total_words = (bin_size + 3) / 4;
  uint32_t rom_crc = Hitung_CRC32_LL((const uint32_t *)APP_START_ADDR, total_words);

  ExtFlash_EraseSector(FLASH_BACKUP_META);
  uint32_t sectors = (bin_size + 4095) / 4096;
  for (uint32_t s = 0; s < sectors; s++)
    ExtFlash_EraseSector(FLASH_BACKUP_BIN + (s * 4096));

  uint32_t bytes_left = bin_size;
  uint32_t cur_rom    = APP_START_ADDR;
  uint32_t cur_flash  = FLASH_BACKUP_BIN;
  while (bytes_left > 0) {
    uint16_t chunk = (bytes_left > 256) ? 256 : bytes_left;
    ExtFlash_WritePage(cur_flash, (const uint8_t *)cur_rom, chunk);
    cur_rom   += chunk;
    cur_flash += chunk;
    bytes_left -= chunk;
  }

  CRC->CR = 1;
  uint32_t words_left = total_words;
  uint32_t read_addr  = FLASH_BACKUP_BIN;
  while (words_left > 0) {
    uint32_t c_words = (words_left > 16) ? 16 : words_left;
    uint8_t temp[64];
    Flash_ReadBytes(read_addr, temp, c_words * 4);
    uint32_t *p = (uint32_t *)temp;
    for (uint32_t w = 0; w < c_words; w++) CRC->DR = p[w];
    read_addr  += (c_words * 4);
    words_left -= c_words;
  }

  if (CRC->DR == rom_crc) {
    uint8_t meta[10] = {0};
    meta[0] = (uint8_t)(bin_size);
    meta[1] = (uint8_t)(bin_size >> 8);
    meta[2] = (uint8_t)(bin_size >> 16);
    meta[3] = (uint8_t)(bin_size >> 24);
    meta[4] = (uint8_t)(rom_crc);
    meta[5] = (uint8_t)(rom_crc >> 8);
    meta[6] = (uint8_t)(rom_crc >> 16);
    meta[7] = (uint8_t)(rom_crc >> 24);
    meta[8] = 0x00;
    ExtFlash_WritePage(FLASH_BACKUP_META, meta, 10);
    return 1;
  }
  return 0;
}

// ==============================================================================
// 3. DRIVER SPI2 - W5500 ETHERNET BARE-METAL
// ==============================================================================
static inline uint8_t SPI2_Transfer(uint8_t data)
{
  uint32_t t = 200000;
  while (!LL_SPI_IsActiveFlag_TXE(SPI2))
    if (--t == 0) return 0xFF;
  LL_SPI_TransmitData8(SPI2, data);
  t = 200000;
  while (!LL_SPI_IsActiveFlag_RXNE(SPI2))
    if (--t == 0) return 0xFF;
  return LL_SPI_ReceiveData8(SPI2);
}

void W5500_WriteReg(uint16_t addr, uint8_t block, uint8_t data)
{
  LL_GPIO_ResetOutputPin(ETH_CS_GPIO_Port, ETH_CS_Pin);
  SPI2_Transfer((uint8_t)(addr >> 8));
  SPI2_Transfer((uint8_t)addr);
  SPI2_Transfer(block | 0x04);
  SPI2_Transfer(data);
  LL_GPIO_SetOutputPin(ETH_CS_GPIO_Port, ETH_CS_Pin);
}

uint8_t W5500_ReadReg(uint16_t addr, uint8_t block)
{
  LL_GPIO_ResetOutputPin(ETH_CS_GPIO_Port, ETH_CS_Pin);
  SPI2_Transfer((uint8_t)(addr >> 8));
  SPI2_Transfer((uint8_t)addr);
  SPI2_Transfer(block | 0x00);
  uint8_t val = SPI2_Transfer(0xFF);
  LL_GPIO_SetOutputPin(ETH_CS_GPIO_Port, ETH_CS_Pin);
  return val;
}

void W5500_WriteBuf(uint16_t addr, uint8_t block, const uint8_t *buf, uint16_t len)
{
  LL_GPIO_ResetOutputPin(ETH_CS_GPIO_Port, ETH_CS_Pin);
  SPI2_Transfer((uint8_t)(addr >> 8));
  SPI2_Transfer((uint8_t)addr);
  SPI2_Transfer(block | 0x04);
  for (uint16_t i = 0; i < len; i++) SPI2_Transfer(buf[i]);
  LL_GPIO_SetOutputPin(ETH_CS_GPIO_Port, ETH_CS_Pin);
}

void W5500_ReadBuf(uint16_t addr, uint8_t block, uint8_t *buf, uint16_t len)
{
  LL_GPIO_ResetOutputPin(ETH_CS_GPIO_Port, ETH_CS_Pin);
  SPI2_Transfer((uint8_t)(addr >> 8));
  SPI2_Transfer((uint8_t)addr);
  SPI2_Transfer(block | 0x00);
  for (uint16_t i = 0; i < len; i++) buf[i] = SPI2_Transfer(0xFF);
  LL_GPIO_SetOutputPin(ETH_CS_GPIO_Port, ETH_CS_Pin);
}

void W5500_InitNetwork(void)
{
  LL_GPIO_ResetOutputPin(ETH_RST_GPIO_Port, ETH_RST_Pin);
  for (volatile int i = 0; i < 72000; i++);
  LL_GPIO_SetOutputPin(ETH_RST_GPIO_Port, ETH_RST_Pin);
  for (volatile int i = 0; i < 720000; i++);

  // Set MAC — IP/subnet/gateway diurus DHCP
  uint8_t mac[] = {0x00, 0x08, 0xDC, 0x11, 0x22, 0x33};
  uint8_t zero[] = {0, 0, 0, 0};
  W5500_WriteBuf(0x0009, W5500_COMMON_REG_OP, mac,  6);
  W5500_WriteBuf(0x0001, W5500_COMMON_REG_OP, zero, 4);  // GW
  W5500_WriteBuf(0x0005, W5500_COMMON_REG_OP, zero, 4);  // Subnet
  W5500_WriteBuf(0x000F, W5500_COMMON_REG_OP, zero, 4);  // IP

  // Alokasi buffer: Socket 0 = 8KB, Socket 1 = 2KB
  W5500_WriteReg(0x001E, W5500_S0_REG_OP, 8);  // Socket 0 TX
  W5500_WriteReg(0x001F, W5500_S0_REG_OP, 8);  // Socket 0 RX
  W5500_WriteReg(0x001E, 0x28,            2);  // Socket 1 TX
  W5500_WriteReg(0x001F, 0x28,            2);  // Socket 1 RX
}

static void W5500_WaitCR(uint8_t block, uint32_t timeout_ms)
{
  uint32_t t = g_ms_tick;
  while (W5500_ReadReg(Sn_CR, block))
    if ((g_ms_tick - t) > timeout_ms) break;
}

static void W5500_WaitSR(uint8_t block, uint8_t expected, uint32_t timeout_ms)
{
  uint32_t t = g_ms_tick;
  while (W5500_ReadReg(Sn_SR, block) != expected)
    if ((g_ms_tick - t) > timeout_ms) break;
}

void W5500_SocketOpenListen(uint16_t port)
{
  W5500_WriteReg(Sn_CR,      W5500_S0_REG_OP, CMD_CLOSE);
  W5500_WaitCR(W5500_S0_REG_OP, 10);
  W5500_WriteReg(Sn_MR,      W5500_S0_REG_OP, 0x01);
  W5500_WriteReg(Sn_PORT,    W5500_S0_REG_OP, (uint8_t)(port >> 8));
  W5500_WriteReg(Sn_PORT + 1,W5500_S0_REG_OP, (uint8_t)port);
  W5500_WriteReg(Sn_CR,      W5500_S0_REG_OP, CMD_OPEN);
  W5500_WaitCR(W5500_S0_REG_OP, 10);
  W5500_WaitSR(W5500_S0_REG_OP, SOCK_INIT, 50);
  W5500_WriteReg(Sn_CR,      W5500_S0_REG_OP, CMD_LISTEN);
  W5500_WaitCR(W5500_S0_REG_OP, 10);
  W5500_WaitSR(W5500_S0_REG_OP, SOCK_LISTEN, 50);
}

void W5500_SendTCP(const uint8_t *data, uint16_t len)
{
  uint16_t ptr = ((uint16_t)W5500_ReadReg(Sn_TX_WR,     W5500_S0_REG_OP) << 8) |
                             W5500_ReadReg(Sn_TX_WR + 1, W5500_S0_REG_OP);
  W5500_WriteBuf(ptr, W5500_S0_TX_OP, data, len);
  ptr += len;
  W5500_WriteReg(Sn_TX_WR,     W5500_S0_REG_OP, (uint8_t)(ptr >> 8));
  W5500_WriteReg(Sn_TX_WR + 1, W5500_S0_REG_OP, (uint8_t)ptr);
  W5500_WriteReg(Sn_CR,        W5500_S0_REG_OP, CMD_SEND);
  W5500_WaitCR(W5500_S0_REG_OP, 10);
}

// ==============================================================================
// 4. WEB OTA HANDLER
// ==============================================================================
const char HTML_PAGE[] =
"<!DOCTYPE html><html><body style='font-family:sans-serif;text-align:center;padding:20px;'>"
"<h2>STM32 OTA Web Uploader</h2>"
"<input type='file' id='f'><br><br>"
"<button onclick='upload()'>Upload & Update</button>"
"<h4 id='st'>Pilih file .bin</h4>"
"<script>"
"function crc32(b){let v=new DataView(b.buffer,b.byteOffset,b.byteLength),c=0xFFFFFFFF,p=0x04C11DB7,w=Math.floor(b.length/4);"
"for(let i=0;i<w;i++){c^=v.getUint32(i*4,true);for(let k=0;k<32;k++)c=(c&0x80000000)?((c<<1)^p)>>>0:(c<<1)>>>0;}"
"return '0x'+(c>>>0).toString(16).toUpperCase();}"
"async function upload(){"
"let file=document.getElementById('f').files[0]; if(!file)return;"
"let arr=new Uint8Array(await file.arrayBuffer());"
"let hexCrc=crc32(arr);"
"document.getElementById('st').innerText='Mengunggah... (CRC: '+hexCrc+')';"
"try{"
"  let res=await fetch('/ota?sz='+arr.length+'&crc='+hexCrc,{method:'POST',body:arr});"
"  let txt=await res.text();"
"  document.getElementById('st').innerText=txt;"
"}catch(e){"
"  document.getElementById('st').innerText='Update Gagal! Coba lagi.';}"
"}"
"</script></body></html>";

void Process_Web_OTA(void)
{
  uint8_t sr = W5500_ReadReg(Sn_SR, W5500_S0_REG_OP);

  if (sr == SOCK_CLOSED) {
    W5500_SocketOpenListen(80);
    return;
  }
  if (sr == SOCK_CLOSE_WAIT || sr == SOCK_FIN_WAIT) {
    W5500_WriteReg(Sn_CR, W5500_S0_REG_OP, CMD_DISCON);
    while (W5500_ReadReg(Sn_CR, W5500_S0_REG_OP));
    return;
  }
  if (sr != SOCK_ESTABLISHED) return;

  uint16_t rx_len = ((uint16_t)W5500_ReadReg(Sn_RX_RSR,     W5500_S0_REG_OP) << 8) |
                               W5500_ReadReg(Sn_RX_RSR + 1, W5500_S0_REG_OP);
  if (rx_len == 0) return;

  uint16_t rx_rd = ((uint16_t)W5500_ReadReg(Sn_RX_RD,     W5500_S0_REG_OP) << 8) |
                               W5500_ReadReg(Sn_RX_RD + 1, W5500_S0_REG_OP);

  uint16_t fetch = (rx_len > (CHUNK_BUFFER_SIZE - 1)) ? (CHUNK_BUFFER_SIZE - 1) : rx_len;
  uint16_t s_off = rx_rd & W5500_S0_BUF_MASK;

  if ((s_off + fetch) > (W5500_S0_BUF_MASK + 1)) {
    uint16_t part1 = (W5500_S0_BUF_MASK + 1) - s_off;
    W5500_ReadBuf(s_off,  W5500_S0_RX_OP, ram_buffer,         part1);
    W5500_ReadBuf(0x0000, W5500_S0_RX_OP, ram_buffer + part1, fetch - part1);
  } else {
    W5500_ReadBuf(s_off, W5500_S0_RX_OP, ram_buffer, fetch);
  }
  ram_buffer[fetch] = '\0';

//	// ============================================================
//	// /config — Halaman konfigurasi MQTT
//	// ============================================================
//	if (strstr((char *)ram_buffer, "/config") != NULL)
//	{
//		WebConfig_Handle(ram_buffer, fetch);
//
//		rx_rd += rx_len;
//		W5500_WriteReg(Sn_RX_RD,     W5500_S0_REG_OP, (uint8_t)(rx_rd >> 8));
//		W5500_WriteReg(Sn_RX_RD + 1, W5500_S0_REG_OP, (uint8_t)rx_rd);
//		W5500_WriteReg(Sn_CR, W5500_S0_REG_OP, CMD_RECV);
//		while (W5500_ReadReg(Sn_CR, W5500_S0_REG_OP));
//			W5500_WriteReg(Sn_CR, W5500_S0_REG_OP, CMD_DISCON);
//		while (W5500_ReadReg(Sn_CR, W5500_S0_REG_OP));
//			return;
//	}
  // ============================================================
  	// /config — Halaman konfigurasi MQTT
  	// ============================================================
  	if (strstr((char *)ram_buffer, "/config") != NULL)
  	{
  		// Cek apakah request berupa POST (Form submit)
  		if (strstr((char *)ram_buffer, "POST") != NULL)
  		{
  			uint8_t tail[4] = {0};
  			char *pBody = NULL;

  			// Stream per chunk 256 byte sampai delimiter "\r\n\r\n" ditemukan
  			while (pBody == NULL) {
  				uint8_t overlap[4 + CHUNK_BUFFER_SIZE];
  				memcpy(overlap, tail, 4);
  				memcpy(overlap + 4, ram_buffer, fetch);
  				overlap[4 + fetch] = '\0';

  				char *found = strstr((char *)overlap, "\r\n\r\n");
  				if (found) {
  					int pos_in_overlap = (found - (char *)overlap);
  					int pos_in_buf     = pos_in_overlap - 4;
  					int body_start     = pos_in_buf + 4;
  					if (body_start < 0)     body_start = 0;
  					if (body_start > fetch) body_start = fetch;
  					pBody = (char *)ram_buffer + body_start;

  					// Tandai penerimaan chunk ini ke W5500
  					rx_rd += fetch;
  					W5500_WriteReg(Sn_RX_RD,     W5500_S0_REG_OP, (uint8_t)(rx_rd >> 8));
  					W5500_WriteReg(Sn_RX_RD + 1, W5500_S0_REG_OP, (uint8_t)rx_rd);
  					W5500_WriteReg(Sn_CR,        W5500_S0_REG_OP, CMD_RECV);
  					while (W5500_ReadReg(Sn_CR, W5500_S0_REG_OP));
  				} else {
  					memcpy(tail, ram_buffer + fetch - 4, 4);
  					rx_rd += fetch;
  					W5500_WriteReg(Sn_RX_RD,     W5500_S0_REG_OP, (uint8_t)(rx_rd >> 8));
  					W5500_WriteReg(Sn_RX_RD + 1, W5500_S0_REG_OP, (uint8_t)rx_rd);
  					W5500_WriteReg(Sn_CR,        W5500_S0_REG_OP, CMD_RECV);
  					while (W5500_ReadReg(Sn_CR, W5500_S0_REG_OP));

  					// Tarik chunk berikutnya
  					rx_len = ((uint16_t)W5500_ReadReg(Sn_RX_RSR,     W5500_S0_REG_OP) << 8) |
  					                     W5500_ReadReg(Sn_RX_RSR + 1, W5500_S0_REG_OP);
  					if (rx_len == 0) continue;
  					fetch = (rx_len > (CHUNK_BUFFER_SIZE - 1)) ? (CHUNK_BUFFER_SIZE - 1) : rx_len;
  					s_off = rx_rd & W5500_S0_BUF_MASK;
  					W5500_ReadBuf(s_off, W5500_S0_RX_OP, ram_buffer, fetch);
  					ram_buffer[fetch] = '\0';
  				}
  			}

  			// Pindahkan sisa body yang terpotong ke awal ram_buffer
  			uint16_t body_in_chunk = fetch - ((uint8_t *)pBody - ram_buffer);
  			memmove(ram_buffer, pBody, body_in_chunk);
  			ram_buffer[body_in_chunk] = '\0';

  			// Jika body masih tersisa di buffer hardware W5500, ambil sisa byte-nya
  			rx_len = ((uint16_t)W5500_ReadReg(Sn_RX_RSR,     W5500_S0_REG_OP) << 8) |
  			                     W5500_ReadReg(Sn_RX_RSR + 1, W5500_S0_REG_OP);
  			if (rx_len > 0) {
  				uint16_t grab = rx_len;
  				if (body_in_chunk + grab > CHUNK_BUFFER_SIZE - 1) {
  					grab = (CHUNK_BUFFER_SIZE - 1) - body_in_chunk;
  				}
  				s_off = rx_rd & W5500_S0_BUF_MASK;
  				W5500_ReadBuf(s_off, W5500_S0_RX_OP, ram_buffer + body_in_chunk, grab);
  				body_in_chunk += grab;
  				ram_buffer[body_in_chunk] = '\0';

  				rx_rd += rx_len;
  				W5500_WriteReg(Sn_RX_RD,     W5500_S0_REG_OP, (uint8_t)(rx_rd >> 8));
  				W5500_WriteReg(Sn_RX_RD + 1, W5500_S0_REG_OP, (uint8_t)rx_rd);
  				W5500_WriteReg(Sn_CR,        W5500_S0_REG_OP, CMD_RECV);
  				while (W5500_ReadReg(Sn_CR, W5500_S0_REG_OP));
  			}

  			// Sisipkan prefix "POST /config\r\n\r\n" agar lolos pengecekan strstr() di web_config.c
  			const char prefix_tag[] = "POST /config\r\n\r\n";
  			uint8_t tag_len = strlen(prefix_tag);
  			if (body_in_chunk + tag_len < CHUNK_BUFFER_SIZE) {
  				memmove(ram_buffer + tag_len, ram_buffer, body_in_chunk + 1);
  				memcpy(ram_buffer, prefix_tag, tag_len);
  				body_in_chunk += tag_len;
  			}

  			// Proses penyimpanan dan kirim response HTTP
  			WebConfig_Handle(ram_buffer, body_in_chunk);

  			// Berikan jeda sejenak agar frame TCP ACK / HTTP 200 OK selesai terkirim sebelum DISCON
  			for (volatile int d = 0; d < 20000; d++);

  			W5500_WriteReg(Sn_CR, W5500_S0_REG_OP, CMD_DISCON);
  			while (W5500_ReadReg(Sn_CR, W5500_S0_REG_OP));
  			return;
  		}

  		// Handle GET /config dan GET /config/data
  		WebConfig_Handle(ram_buffer, fetch);

  		rx_rd += rx_len;
  		W5500_WriteReg(Sn_RX_RD,     W5500_S0_REG_OP, (uint8_t)(rx_rd >> 8));
  		W5500_WriteReg(Sn_RX_RD + 1, W5500_S0_REG_OP, (uint8_t)rx_rd);
  		W5500_WriteReg(Sn_CR,        W5500_S0_REG_OP, CMD_RECV);
  		while (W5500_ReadReg(Sn_CR, W5500_S0_REG_OP));

  		for (volatile int d = 0; d < 20000; d++);

  		W5500_WriteReg(Sn_CR,        W5500_S0_REG_OP, CMD_DISCON);
  		while (W5500_ReadReg(Sn_CR, W5500_S0_REG_OP));
  		return;
  	}




  // GET / — Kirim halaman web
  if (strstr((char *)ram_buffer, "GET /") != NULL) {
    char header[128];
    sprintf(header,
      "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: %d\r\nConnection: close\r\n\r\n",
      strlen(HTML_PAGE));
    W5500_SendTCP((uint8_t *)header, strlen(header));
    W5500_SendTCP((const uint8_t *)HTML_PAGE, strlen(HTML_PAGE));
    rx_rd += rx_len;
    W5500_WriteReg(Sn_RX_RD,     W5500_S0_REG_OP, (uint8_t)(rx_rd >> 8));
    W5500_WriteReg(Sn_RX_RD + 1, W5500_S0_REG_OP, (uint8_t)rx_rd);
    W5500_WriteReg(Sn_CR, W5500_S0_REG_OP, CMD_RECV);
    while (W5500_ReadReg(Sn_CR, W5500_S0_REG_OP));
    W5500_WriteReg(Sn_CR, W5500_S0_REG_OP, CMD_DISCON);
    while (W5500_ReadReg(Sn_CR, W5500_S0_REG_OP));
    return;
  }

  // POST /ota — Terima file binary
  if (strstr((char *)ram_buffer, "POST /ota") == NULL) return;

  char *pSize = strstr((char *)ram_buffer, "?sz=");
  char *pCrc  = strstr((char *)ram_buffer, "&crc=");
  if (!pSize || !pCrc) return;

  uint32_t expected_size = strtoul(pSize + 4, NULL, 10);
  uint32_t expected_crc  = strtoul(pCrc  + 5, NULL, 0);

  uint8_t rx_already_updated = 0;
  uint8_t tail[4] = {0};
  char *pBody = NULL;

  while (pBody == NULL) {
    uint8_t overlap[4 + CHUNK_BUFFER_SIZE];
    memcpy(overlap, tail, 4);
    memcpy(overlap + 4, ram_buffer, fetch);
    overlap[4 + fetch] = '\0';

    char *found = strstr((char *)overlap, "\r\n\r\n");
    if (found) {
      int pos_in_overlap = (found - (char *)overlap);
      int pos_in_buf     = pos_in_overlap - 4;
      int body_start     = pos_in_buf + 4;
      if (body_start < 0)     body_start = 0;
      if (body_start > fetch) body_start = fetch;
      pBody = (char *)ram_buffer + body_start;

      rx_rd += fetch;
      W5500_WriteReg(Sn_RX_RD,     W5500_S0_REG_OP, (uint8_t)(rx_rd >> 8));
      W5500_WriteReg(Sn_RX_RD + 1, W5500_S0_REG_OP, (uint8_t)rx_rd);
      W5500_WriteReg(Sn_CR, W5500_S0_REG_OP, CMD_RECV);
      while (W5500_ReadReg(Sn_CR, W5500_S0_REG_OP));
      rx_already_updated = 1;
    } else {
      memcpy(tail, ram_buffer + fetch - 4, 4);
      rx_rd += fetch;
      W5500_WriteReg(Sn_RX_RD,     W5500_S0_REG_OP, (uint8_t)(rx_rd >> 8));
      W5500_WriteReg(Sn_RX_RD + 1, W5500_S0_REG_OP, (uint8_t)rx_rd);
      W5500_WriteReg(Sn_CR, W5500_S0_REG_OP, CMD_RECV);
      while (W5500_ReadReg(Sn_CR, W5500_S0_REG_OP));
      rx_len = ((uint16_t)W5500_ReadReg(Sn_RX_RSR,     W5500_S0_REG_OP) << 8) |
                           W5500_ReadReg(Sn_RX_RSR + 1, W5500_S0_REG_OP);
      if (rx_len == 0) continue;
      fetch = (rx_len > (CHUNK_BUFFER_SIZE - 1)) ? (CHUNK_BUFFER_SIZE - 1) : rx_len;
      s_off = rx_rd & W5500_S0_BUF_MASK;
      W5500_ReadBuf(s_off, W5500_S0_RX_OP, ram_buffer, fetch);
      ram_buffer[fetch] = '\0';
    }
  }

  if (expected_size == 0 || expected_size > MAX_FIRMWARE_SIZE) {
    const char err[] = "HTTP/1.1 400 Bad\r\nConnection: close\r\n\r\nFile > 28KB!";
    W5500_SendTCP((uint8_t *)err, strlen(err));
    W5500_WriteReg(Sn_CR, W5500_S0_REG_OP, CMD_DISCON);
    while (W5500_ReadReg(Sn_CR, W5500_S0_REG_OP));
    return;
  }

#ifdef OTA_MODE_BACKUP
  uint32_t target_meta = FLASH_BACKUP_META;
  uint32_t target_bin  = FLASH_BACKUP_BIN;
#else
  uint32_t target_meta = FLASH_STAGING_META;
  uint32_t target_bin  = FLASH_STAGING_BIN;
#endif

  ExtFlash_EraseSector(target_meta);
  uint32_t sectors = (expected_size + 4095) / 4096;
  for (uint32_t s = 0; s < sectors; s++)
    ExtFlash_EraseSector(target_bin + (s * 4096));

  uint16_t header_len     = (uint8_t *)pBody - ram_buffer;
  uint16_t body_in_packet = fetch - header_len;
  uint32_t bytes_received = 0;

  if (body_in_packet > 0) {
    uint16_t written = 0;
    while (written < body_in_packet) {
      uint32_t cur_flash   = target_bin + written;
      uint16_t page_offset = cur_flash & 0xFF;
      uint16_t space_left  = 256 - page_offset;
      uint16_t page_len    = (body_in_packet - written > space_left)
                             ? space_left : (body_in_packet - written);
      ExtFlash_WritePage(cur_flash, (uint8_t *)pBody + written, page_len);
      written += page_len;
    }
    bytes_received += body_in_packet;
  }

  if (!rx_already_updated) {
    rx_rd += fetch;
    W5500_WriteReg(Sn_RX_RD,     W5500_S0_REG_OP, (uint8_t)(rx_rd >> 8));
    W5500_WriteReg(Sn_RX_RD + 1, W5500_S0_REG_OP, (uint8_t)rx_rd);
    W5500_WriteReg(Sn_CR, W5500_S0_REG_OP, CMD_RECV);
    while (W5500_ReadReg(Sn_CR, W5500_S0_REG_OP));
  }

  while (bytes_received < expected_size) {
    rx_len = ((uint16_t)W5500_ReadReg(Sn_RX_RSR,     W5500_S0_REG_OP) << 8) |
                         W5500_ReadReg(Sn_RX_RSR + 1, W5500_S0_REG_OP);
    if (rx_len == 0) continue;

    rx_rd = ((uint16_t)W5500_ReadReg(Sn_RX_RD,     W5500_S0_REG_OP) << 8) |
                        W5500_ReadReg(Sn_RX_RD + 1, W5500_S0_REG_OP);

    uint16_t grab = (rx_len > CHUNK_BUFFER_SIZE) ? CHUNK_BUFFER_SIZE : rx_len;
    if (bytes_received + grab > expected_size) grab = expected_size - bytes_received;

    s_off = rx_rd & W5500_S0_BUF_MASK;
    if ((s_off + grab) > (W5500_S0_BUF_MASK + 1)) {
      uint16_t p1 = (W5500_S0_BUF_MASK + 1) - s_off;
      W5500_ReadBuf(s_off,  W5500_S0_RX_OP, ram_buffer,      p1);
      W5500_ReadBuf(0x0000, W5500_S0_RX_OP, ram_buffer + p1, grab - p1);
    } else {
      W5500_ReadBuf(s_off, W5500_S0_RX_OP, ram_buffer, grab);
    }

    uint16_t written = 0;
    while (written < grab) {
      uint32_t cur_flash   = target_bin + bytes_received + written;
      uint16_t page_offset = cur_flash & 0xFF;
      uint16_t space_left  = 256 - page_offset;
      uint16_t page_len    = (grab - written > space_left)
                             ? space_left : (grab - written);
      ExtFlash_WritePage(cur_flash, &ram_buffer[written], page_len);
      written += page_len;
    }

    bytes_received += grab;
    rx_rd          += grab;
    W5500_WriteReg(Sn_RX_RD,     W5500_S0_REG_OP, (uint8_t)(rx_rd >> 8));
    W5500_WriteReg(Sn_RX_RD + 1, W5500_S0_REG_OP, (uint8_t)rx_rd);
    W5500_WriteReg(Sn_CR,        W5500_S0_REG_OP, CMD_RECV);
    while (W5500_ReadReg(Sn_CR, W5500_S0_REG_OP));
  }

  CRC->CR = 1;
  uint32_t words  = (expected_size + 3) / 4;
  uint32_t r_addr = target_bin;
  while (words > 0) {
    uint32_t c_w = (words > 16) ? 16 : words;
    uint8_t t[64];
    Flash_ReadBytes(r_addr, t, c_w * 4);
    uint32_t *p = (uint32_t *)t;
    for (uint32_t w = 0; w < c_w; w++) CRC->DR = p[w];
    r_addr += (c_w * 4);
    words  -= c_w;
  }

  if (CRC->DR == expected_crc) {
    uint8_t meta[10] = {0};
    meta[0] = (uint8_t)(expected_size);
    meta[1] = (uint8_t)(expected_size >> 8);
    meta[2] = (uint8_t)(expected_size >> 16);
    meta[3] = (uint8_t)(expected_size >> 24);
    meta[4] = (uint8_t)(expected_crc);
    meta[5] = (uint8_t)(expected_crc >> 8);
    meta[6] = (uint8_t)(expected_crc >> 16);
    meta[7] = (uint8_t)(expected_crc >> 24);

#ifdef OTA_MODE_BACKUP
    meta[8] = 0x00;
    ExtFlash_WritePage(target_meta, meta, 10);
    const char ok_body[] = "Backup Saved!";
    char ok_msg[80];
    sprintf(ok_msg,
      "HTTP/1.1 200 OK\r\nContent-Length: %d\r\nConnection: close\r\n\r\n%s",
      strlen(ok_body), ok_body);
    W5500_SendTCP((uint8_t *)ok_msg, strlen(ok_msg));
    W5500_WriteReg(Sn_CR, W5500_S0_REG_OP, CMD_DISCON);
    while (W5500_ReadReg(Sn_CR, W5500_S0_REG_OP));
#else
    meta[8] = 0xA5;
    ExtFlash_WritePage(target_meta, meta, 10);
    const char ok_body[] = "Update Terkirim! Firmware tersimpan di flash. Device Rebooting...";
    char ok_msg[128];
    sprintf(ok_msg,
      "HTTP/1.1 200 OK\r\nContent-Length: %d\r\nConnection: close\r\n\r\n%s",
      strlen(ok_body), ok_body);
    W5500_SendTCP((uint8_t *)ok_msg, strlen(ok_msg));
    for (volatile int d = 0; d < 7200000; d++);
    NVIC_SystemReset();
#endif

  } else {
    const char fail_msg[] = "HTTP/1.1 500 Error\r\nConnection: close\r\n\r\nCRC Mismatch!";
    W5500_SendTCP((uint8_t *)fail_msg, strlen(fail_msg));
    W5500_WriteReg(Sn_CR, W5500_S0_REG_OP, CMD_DISCON);
    while (W5500_ReadReg(Sn_CR, W5500_S0_REG_OP));
  }
}

uint32_t g_html_ota_size    = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  SCB->VTOR = APP_START_ADDR;
  __enable_irq();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  */
  LL_GPIO_AF_Remap_SWJ_NOJTAG();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // Enable SysTick interrupt (LL_Init1msTick tidak set TICKINT otomatis)
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_CRC_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  LL_SPI_Enable(SPI1);
  if (LL_SPI_IsActiveFlag_RXNE(SPI1)) (void)LL_SPI_ReceiveData8(SPI1);
  if (SPI1->SR & SPI_SR_OVR) { (void)SPI1->DR; (void)SPI1->SR; }
  LL_SPI_Enable(SPI2);

  W5500_InitNetwork();
  DHCP_Init();
  WebConfig_Init();
  MQTT_Init();

  g_dbg_s2_sr     = W5500_ReadReg(0x0003, 0x48);
  g_dbg_s2_fsr_hi = W5500_ReadReg(0x0020, 0x48);
  g_dbg_s2_fsr_lo = W5500_ReadReg(0x0021, 0x48);

  g_phy_reg = W5500_ReadReg(0x002E, 0x00);

#ifdef MAIN_DEBUG
  // Test write/read Socket 0
  W5500_WriteReg(0x0004, W5500_S0_REG_OP, 0xAB);
  g_dbg_s0_test = W5500_ReadReg(0x0004, W5500_S0_REG_OP);

  // Test write/read Socket 1
  W5500_WriteReg(0x0004, 0x28, 0xCD);
  g_dbg_s1_test = W5500_ReadReg(0x0004, 0x28);

  // Test write/read DHAR Socket 1
  W5500_WriteReg(0x0006, 0x28, 0xFF);
  W5500_WriteReg(0x0007, 0x28, 0xFF);
  g_dbg_test2 = W5500_ReadReg(0x0006, 0x28);
  g_dbg_test3 = W5500_ReadReg(0x0007, 0x28);
#endif

#ifdef DEBUG_MQTT
  W5500_WriteReg(0x001E, 0x48, 2);
  W5500_WriteReg(0x001F, 0x48, 2);
  g_dbg_s2_tx_size = W5500_ReadReg(0x001E, 0x48);
#endif

  uint32_t led_tick = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    DHCP_Process();

    if (DHCP_IsBound())
    {
      Process_Web_OTA();
      static MqttConfig_t s_mqtt_cfg;
      static uint8_t s_cfg_loaded = 0;
      if (!s_cfg_loaded)
      {
    	  MqttConfig_Load(&s_mqtt_cfg);
          s_cfg_loaded = 1;
      }
      MQTT_Process(&s_mqtt_cfg);
    }

    LL_IWDG_ReloadCounter(IWDG);


    g_html_ota_size    = sizeof(HTML_PAGE);

    if (++led_tick > 100000)
    {
      LL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
      led_tick = 0;
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
  }
  LL_RCC_LSI_Enable();

   /* Wait till LSI is ready */
  while(LL_RCC_LSI_IsReady() != 1)
  {
  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }
  LL_Init1msTick(72000000);
  LL_SetSystemCoreClock(72000000);
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* Peripheral clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  LL_IWDG_Enable(IWDG);
  LL_IWDG_EnableWriteAccess(IWDG);
  LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_256);
  LL_IWDG_SetReloadCounter(IWDG, 780);
  while (LL_IWDG_IsReady(IWDG) != 1)
  {
  }
  LL_IWDG_ReloadCounter(IWDG);
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**SPI1 GPIO Configuration
  PA5   ------> SPI1_SCK
  PA6   ------> SPI1_MISO
  PA7   ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = FLASH_SCK_Pin|FLASH_MOSI_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = FLASH_MISO_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(FLASH_MISO_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**SPI2 GPIO Configuration
  PB13   ------> SPI2_SCK
  PB14   ------> SPI2_MISO
  PB15   ------> SPI2_MOSI
  */
  GPIO_InitStruct.Pin = ETH_SCK_Pin|ETH_MOSI_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ETH_MISO_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(ETH_MISO_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI2, &SPI_InitStruct);
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_SetOutputPin(GPIOA, ETH_RST_Pin|FLASH_CS_Pin);

  /**/
  LL_GPIO_SetOutputPin(ETH_CS_GPIO_Port, ETH_CS_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);

  /**/
  GPIO_InitStruct.Pin = ETH_RST_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(ETH_RST_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = FLASH_CS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(FLASH_CS_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED_BUILTIN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(LED_BUILTIN_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = ETH_CS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(ETH_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
