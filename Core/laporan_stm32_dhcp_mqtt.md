# Laporan Debug & Implementasi STM32F103 + W5500
## DHCP Client + Web Config MQTT + MQTT Client Bare-Metal
**Tanggal:** 30 September 2026

---

## Ringkasan Proyek

**Hardware:**
- MCU: STM32F103C8T6 (72MHz, HSE+PLL)
- Ethernet: W5500 via SPI2 (PB13/14/15)
- SPI Flash: W25Q 8MB via SPI1 (PA5/6/7)
- Bootloader di `0x08000000`, App di `0x08001000`

**Yang sudah ada sebelumnya:**
- Web OTA via HTTP POST `/ota` dengan IP static `172.155.0.200`
- Streaming firmware ke SPI Flash, verifikasi CRC32

**Target sesi ini:**
1. DHCP client — IP dinamis dari router
2. Web Config MQTT di route `/config` mirip Tasmota
3. MQTT Client bare-metal publish/subscribe

---

## BAGIAN 1: DHCP CLIENT

### Masalah 1 — SysTick tidak jalan (ROOT CAUSE UTAMA)

**Gejala:** `g_ms_tick` selalu 0 meski program jalan. Semua timeout tidak bekerja.

**Penyebab:** `LL_Init1msTick()` di library LL STM32 **tidak** men-set bit `TICKINT` di register `SysTick->CTRL`. Berbeda dengan HAL yang otomatis enable interrupt.

```c
// Isi LL_Init1msTick() — tidak ada TICKINT:
SysTick->LOAD = (HCLKFrequency / 1000U) - 1U;
SysTick->VAL  = 0UL;
SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
// ← TICKINT tidak di-set!
```

**Fix:** Tambah satu baris di `main.c` setelah `SystemClock_Config()`:
```c
/* USER CODE BEGIN SysInit */
SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
/* USER CODE END SysInit */
```

Dan tambah `g_ms_tick++` di `stm32f1xx_it.c`:
```c
/* External variables */
extern volatile uint32_t g_ms_tick;

void SysTick_Handler(void)
{
    g_ms_tick++;
}
```

**Dampak fix:** Semua timeout (`wait_cr_clear`, `wait_sr`, loop IR flag) langsung bekerja.

---

### Masalah 2 — Buffer Socket 1 tidak teralokasi

**Gejala:** DHCP DISCOVER tidak keluar dari W5500.

**Penyebab:** Register buffer size Socket 1 ditulis ke **Common Register** (`0x00`) bukan ke **Socket 1 Register** (`0x28`).

```c
// SALAH:
W5500_WriteReg(0x001E, W5500_COMMON_REG_OP, 8);  // block 0x00 = Common

// BENAR:
W5500_WriteReg(0x001E, W5500_S0_REG_OP, 8);  // block 0x08 = Socket 0
W5500_WriteReg(0x001F, W5500_S0_REG_OP, 8);
W5500_WriteReg(0x001E, 0x28, 2);              // block 0x28 = Socket 1
W5500_WriteReg(0x001F, 0x28, 2);
```

**Referensi block selector W5500:**
```
Socket N REG = (N*4+1) << 3
Socket 0: REG=0x08, TX=0x10, RX=0x18
Socket 1: REG=0x28, TX=0x30, RX=0x38
Socket 2: REG=0x48, TX=0x50, RX=0x58
```

---

### Masalah 3 — Destination MAC tidak di-set (ARP timeout)

**Gejala:** `g_dbg_ir_after_send = 0x10` (TIMEOUT flag) — W5500 gagal kirim UDP broadcast.

**Penyebab:** W5500 mencoba ARP untuk resolve MAC `255.255.255.255` — ini tidak mungkin berhasil. Harus set destination MAC manual ke `FF:FF:FF:FF:FF:FF`.

**Fix:** Set DHAR (Destination Hardware Address Register) di `S1_OpenUDP()` sebelum CMD_OPEN:
```c
for (int i = 0; i < 6; i++)
    W5500_WriteReg(0x0006 + i, S1_REG_OP, 0xFF);  // DHAR broadcast
uint8_t bcast[4] = {255, 255, 255, 255};
W5500_WriteBuf(Sn_DIPR, S1_REG_OP, bcast, 4);
```

---

### Masalah 4 — TX buffer pointer pakai mask

**Gejala:** Data tertulis ke alamat salah di TX buffer.

**Penyebab:** `ptr & S1_BUF_MASK` — W5500 handle wrap-around buffer secara internal, tidak perlu mask dari software.

```c
// SALAH:
W5500_WriteBuf(ptr & S1_BUF_MASK, S1_TX_OP, data, len);

// BENAR:
W5500_WriteBuf(ptr, S1_TX_OP, data, len);
```

---

### Masalah 5 — CMD_SEND timeout terlalu pendek

**Gejala:** `g_dbg_ir_after_send = 0` — CMD_SEND belum selesai saat dicek.

**Penyebab:** `wait_cr_clear(10ms)` tidak cukup. Perlu tunggu IR flag SENDOK (bit 0) atau TIMEOUT (bit 4).

**Fix:**
```c
W5500_WriteReg(Sn_CR, S1_REG_OP, CMD_SEND);
wait_cr_clear(W5500_CMD_TIMEOUT_MS);

uint32_t t = g_ms_tick;
uint8_t ir;
do {
    ir = W5500_ReadReg(Sn_IR, S1_REG_OP);
} while (!(ir & 0x11) && (g_ms_tick - t) < 500);

W5500_WriteReg(Sn_IR, S1_REG_OP, ir);  // clear flag
```

---

### Hasil DHCP

Setelah semua fix:
```
g_dhcp_state = 5 (BOUND) ✅
g_dhcp_ip    = 172.155.0.39 ✅
g_dhcp_lease = 1800 (30 menit) ✅
g_ms_tick    = bertambah tiap ms ✅
```

tcpdump konfirmasi:
```
_gateway > 255.255.255.255: DHCP ACK
  Client-IP 172.155.0.39
  Subnet-Mask 255.255.252.0
  Lease-Time 1800
```

---

## BAGIAN 2: WEB CONFIG MQTT

### Implementasi

File baru: `mqtt_config.h/c` dan `web_config.h/c`

**Penyimpanan config:** SPI Flash address `0x00030000` (4KB sector), tanpa LittleFS karena flash overflow.

**Struct MqttConfig_t:**
```c
typedef struct {
    char     host       [64];  // Broker IP
    uint16_t port;             // Default: 1883
    char     client_id  [32];  // MQTT Client ID
    char     user       [32];  // Username
    char     password   [32];  // Password
    char     topic      [32];  // %topic%
    char     full_topic [64];  // Template, default: "%prefix%/%topic%/"
    uint16_t tele_period;      // Interval telemetri (detik)
    uint8_t  enabled;          // 1=aktif
    uint8_t  _pad[1];
    uint32_t magic;            // 0xABCD1234 = valid
} MqttConfig_t;
```

**Helper MqttConfig_BuildTopic():** Replace `%prefix%` dan `%topic%` dari full_topic template:
```c
// full_topic = "%prefix%/%topic%/"
// prefix     = "tele", topic = "stm32"
// hasil      = "tele/stm32/"
```

**Halaman /config:** Dark theme mirip Tasmota, field lengkap, preview topic realtime di browser via JavaScript.

### Masalah — Form tidak load parameter lama

**Penyebab:** Race condition — JavaScript `load()` dipanggil sebelum DOM siap.

**Fix:**
```javascript
// Ganti: load();
// Jadi:
document.addEventListener('DOMContentLoaded', load);
```

### Masalah — Error build tanda kutip

**Penyebab:** `"DOMContentLoaded"` di dalam string C tidak di-escape.

**Fix:** Ganti double quote dengan single quote di JavaScript:
```c
"document.addEventListener('DOMContentLoaded',load);"
```

---

## BAGIAN 3: MQTT CLIENT BARE-METAL

### Arsitektur

- Socket 2 W5500 (block `0x48/0x50/0x58`), TCP, 2KB buffer
- State machine: `IDLE → TCP_CONN → MQTT_CONN → CONNECTED → RECONNECT`
- QoS 0 untuk semua publish/subscribe
- Format payload persis Tasmota Sonoff single relay

### Topic Structure (default `%prefix%/%topic%/`)

```
tele/stm32/LWT     → "Online" / "Offline" (LWT)
tele/stm32/INFO1   → info module, versi, fallback topic
tele/stm32/INFO2   → IP, hostname
tele/stm32/INFO3   → restart reason, boot count
tele/stm32/STATE   → status device periodik (uptime, POWER, Eth)
tele/stm32/SENSOR  → data sensor periodik (ENERGY dummy)
stat/stm32/RESULT  → {"POWER":"ON"} — reply command
stat/stm32/POWER   → ON/OFF — reply command
cmnd/stm32/POWER   → subscribe terima perintah ON/OFF/TOGGLE
cmnd/stm32/TelePeriod → subscribe ubah interval telemetri
```

### Masalah 1 — MQTT CONNECT packet duplikat/berantakan

**Penyebab:** Fungsi `mqtt_send_connect()` dibangun dua kali (ada sisa kode lama).

**Fix:** Bersihkan jadi satu pass yang benar dengan flag yang tepat:
```c
uint8_t flags = 0x02;           // Clean Session
flags |= 0x04;                  // Will Flag
flags |= 0x20;                  // Will Retain
if (cfg->user[0])     flags |= 0x80;
if (cfg->password[0]) flags |= 0x40;
```

### Masalah 2 — sscanf tidak aman di embedded

**Penyebab:** `sscanf` untuk parse IP string butuh banyak stack.

**Fix:** Parser IP manual:
```c
uint8_t ip[4] = {0};
const char *p = cfg->host;
for (int idx = 0; idx < 4; idx++) {
    uint16_t val = 0;
    while (*p >= '0' && *p <= '9') {
        val = val * 10 + (*p - '0');
        p++;
    }
    ip[idx] = (uint8_t)val;
    if (*p == '.') p++;
}
```

### Masalah 3 — TX buffer overflow saat startup

**Gejala:** `g_mqtt_pub_count = 0` meski `CONNECTED`. Semua publish di CONNACK handler gagal.

**Penyebab:** Saat CONNACK diterima, langsung kirim 6 packet sekaligus (Subscribe + LWT + INFO1 + INFO2 + INFO3 + STATE + SENSOR) — overflow TX buffer 2KB.

**Fix:** Startup sequence — kirim satu packet per loop iteration:
```c
if (s_startup_step < 6) {
    switch (s_startup_step) {
        case 0: mqtt_subscribe(cmnd_wildcard); break;
        case 1: MQTT_Publish(s_t_lwt, "Online", 6); break;
        case 2: publish_info(cfg); break;
        case 3: publish_state(cfg); break;
        case 4: publish_sensor(); break;
        case 5: s_last_pub = g_ms_tick; break;
    }
    s_startup_step++;
    break;
}
```

### Masalah 4 — Parse topic incoming PUBLISH salah offset

**Penyebab:** MQTT remaining length bisa 1 atau 2 byte. Kode lama selalu asumsi 1 byte, jadi offset topic_len salah.

**Fix:**
```c
uint8_t  rl0      = s_buf[1];
uint8_t  hdr_size = (rl0 & 0x80) ? 3 : 2;
uint16_t topic_len = ((uint16_t)s_buf[hdr_size] << 8) | s_buf[hdr_size + 1];
char     *topic_in = (char *)&s_buf[hdr_size + 2];
uint8_t  *pay      = &s_buf[hdr_size + 2 + topic_len];
```

### Masalah 5 — s2_send() tidak cek socket status

**Penyebab:** `s2_send()` langsung tulis TX buffer tanpa cek apakah socket established dan buffer cukup.

**Fix:**
```c
static void s2_send(const uint8_t *data, uint16_t len)
{
    uint8_t sr = W5500_ReadReg(Sn_SR, S2_REG_OP);
    if (sr != SOCK_ESTABLISHED) return;

    uint32_t t = g_ms_tick;
    uint16_t fsr;
    do {
        fsr = ((uint16_t)W5500_ReadReg(Sn_TX_FSR, S2_REG_OP) << 8) |
                          W5500_ReadReg(Sn_TX_FSR + 1, S2_REG_OP);
        if ((g_ms_tick - t) > 200) return;
    } while (fsr < len);

    // ... tulis data dan CMD_SEND
}
```

---

## Hasil Akhir

### Live Expression saat running:
```
g_dhcp_ip        = 172.155.0.39
g_dhcp_state     = 5 (BOUND)
g_mqtt_state     = MQTT_STATE_CONNECTED
g_mqtt_conn_count = 1
g_mqtt_pub_count  = 12+
g_ms_tick        = bertambah (SysTick jalan)
```

### MQTT Explorer menunjukkan:
```
▼ stm32otacobacoba
  ▼ tele
      LWT    = Online
      INFO1  = {"Info1":{"Module":"STM32F103+W5500",...}}
      INFO2  = {"Info2":{"Hostname":"cobacobastm32","IPAddress":"172.155.0.39"}}
      INFO3  = {"Info3":{"RestartReason":"Power on","BootCount":1}}
      STATE  = {"Time":"...","Uptime":"...","POWER":"OFF","Eth":{"IP":"172.155.0.39","MAC":"00:08:DC:11:22:33",...}}
      SENSOR = {"Time":"...","Switch1":"OFF","ENERGY":{...}}
  ▼ stat
      RESULT = {"POWER":"ON"}   ← muncul setelah publish cmnd/POWER=ON
      POWER  = ON
```

### Kontrol via MQTT Explorer:
```
Publish ke: stm32otacobacoba/cmnd/POWER
Payload:    ON / OFF / TOGGLE
→ STM32 terima, balas via stat/RESULT dan stat/POWER ✅
```

---

## File yang Dibuat

| File | Fungsi |
|------|--------|
| `dhcp.h / dhcp.c` | DHCP client bare-metal via Socket 1 |
| `mqtt_config.h / mqtt_config.c` | Struct config MQTT + baca/tulis SPI Flash |
| `web_config.h / web_config.c` | Handler HTTP `/config`, HTML Tasmota style |
| `mqtt_client.h / mqtt_client.c` | MQTT client bare-metal via Socket 2 |

## Socket Map W5500

| Socket | Fungsi | Block REG | TX | RX | Buffer |
|--------|--------|-----------|----|----|--------|
| 0 | TCP Web OTA (port 80) | 0x08 | 0x10 | 0x18 | 8KB |
| 1 | UDP DHCP (port 68) | 0x28 | 0x30 | 0x38 | 2KB |
| 2 | TCP MQTT (port 1883) | 0x48 | 0x50 | 0x58 | 2KB |

## Jaringan

| Parameter | Nilai |
|-----------|-------|
| Router/Gateway | 172.155.0.1 |
| Subnet | 172.155.0.0/22 (255.255.252.0) |
| IP static lama | 172.155.0.200 |
| IP DHCP didapat | 172.155.0.39 |
| MAC W5500 | 00:08:DC:11:22:33 |
| MQTT Broker | 172.155.0.202:1883 |

