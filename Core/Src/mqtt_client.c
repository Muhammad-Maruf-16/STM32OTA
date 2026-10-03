#include "mqtt_client.h"
#include "mqtt_config.h"
#include "main.h"
#include "can_bus.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


// =============================================================================
// FORWARD DECLARATION — fungsi W5500 dari main.c
// =============================================================================
extern uint8_t W5500_ReadReg (uint16_t addr, uint8_t block);
extern void    W5500_WriteReg(uint16_t addr, uint8_t block, uint8_t data);
extern void    W5500_WriteBuf(uint16_t addr, uint8_t block, const uint8_t *buf, uint16_t len);
extern void    W5500_ReadBuf (uint16_t addr, uint8_t block, uint8_t *buf, uint16_t len);
extern volatile uint32_t g_ms_tick;

extern CanBoardBData_t g_can_b_data;
extern uint32_t        g_can_tx_count;
extern uint32_t        g_can_rx_count;
extern CAN_HandleTypeDef hcan;

// IP yang didapat DHCP
extern uint8_t g_dhcp_ip[4];

// =============================================================================
// REGISTER W5500 SOCKET 2
// =============================================================================
#define S2_REG_OP    0x48
#define S2_TX_OP     0x50
#define S2_RX_OP     0x58

#define Sn_MR        0x0000
#define Sn_CR        0x0001
#define Sn_IR        0x0002
#define Sn_SR        0x0003
#define Sn_PORT      0x0004
#define Sn_DIPR      0x000C
#define Sn_DPORT     0x0010
#define Sn_TX_FSR    0x0020
#define Sn_TX_WR     0x0024
#define Sn_RX_RSR    0x0026
#define Sn_RX_RD     0x0028

#define SOCK_CLOSED      0x00
#define SOCK_INIT        0x13
#define SOCK_TCP_SYNSENT 0x15
#define SOCK_ESTABLISHED 0x17
#define SOCK_CLOSE_WAIT  0x1C
#define SOCK_FIN_WAIT    0x18

#define CMD_OPEN         0x01
#define CMD_CONNECT      0x04
#define CMD_DISCON       0x08
#define CMD_CLOSE        0x10
#define CMD_SEND         0x20
#define CMD_RECV         0x40

// =============================================================================
// MQTT PACKET TYPE
// =============================================================================
#define MQTT_CONNECT     0x10
#define MQTT_CONNACK     0x20
#define MQTT_PUBLISH     0x30
#define MQTT_PUBACK      0x40
#define MQTT_SUBSCRIBE   0x82
#define MQTT_SUBACK      0x90
#define MQTT_PINGREQ     0xC0
#define MQTT_PINGRESP    0xD0
#define MQTT_DISCONNECT  0xE0

// =============================================================================
// VARIABEL GLOBAL
// =============================================================================
//MqttState_t     g_mqtt_state     = MQTT_STATE_IDLE;
//uint8_t         g_mqtt_conn_count = 0;
//uint32_t        g_mqtt_pub_count  = 0;

MqttState_t     g_mqtt_state;
uint8_t         g_mqtt_conn_count;
uint32_t        g_mqtt_pub_count;

MqttSensorData_t g_mqtt_sensor   = {25.0f, 3.30f};

// =============================================================================
// VARIABEL INTERNAL
// =============================================================================
static uint8_t  s_buf[MQTT_BUF_SIZE];
static uint16_t s_pkt_id   = 1;
static uint32_t s_tick_ref = 0;     // referensi waktu untuk timeout/interval
static uint32_t s_last_ping = 0;    // waktu terakhir PINGREQ
static uint32_t s_uptime_sec = 0;   // uptime counter
static uint32_t s_last_uptime = 0;  // untuk hitung uptime
static uint32_t s_pub_interval = 0; // interval publish dari config (ms)
static uint32_t s_last_pub = 0;     // waktu terakhir publish tele

// topic cache
static char s_t_lwt [96];
static char s_t_state[96];
static char s_t_sensor[96];
static char s_t_result[96];
static char s_t_power[96];
static char s_t_cmnd[96];

// Flag untuk publish startup (dikirim satu per satu di state machine)
static uint8_t s_startup_step = 0;

// Status command queue — untuk handle cmnd/Status=0 (kirim semua STATUS)
static int8_t  s_status_cmd   = -1;  // -1=idle, 0=kirim semua, 1-11=kirim status N
static uint8_t s_status_step  = 0;   // step untuk STATUS0 (kirim 1 per loop)

// =============================================================================
// SHARED STATIC BUFFER — hindari stack overflow
// Semua fungsi pakai buffer yang sama secara bergantian (tidak reentrant)
// =============================================================================
static char     s_json[400];   // buffer JSON publish
static char     s_time[24];    // buffer timestamp
static char     s_uptime[16];  // buffer uptime string

// =============================================================================
// HELPER: Tunggu CR clear Socket 2
// =============================================================================
static void s2_wait_cr(uint32_t timeout_ms)
{
    uint32_t t = g_ms_tick;
    while (W5500_ReadReg(Sn_CR, S2_REG_OP)) {
        if ((g_ms_tick - t) > timeout_ms) break;
    }
}

// =============================================================================
// HELPER: Kirim data via TCP Socket 2
// =============================================================================
static void s2_send(const uint8_t *data, uint16_t len)
{
    // Cek socket established
    uint8_t sr = W5500_ReadReg(Sn_SR, S2_REG_OP);
    if (sr != SOCK_ESTABLISHED) return;

    // Cek TX Free Size cukup
    uint32_t t = g_ms_tick;
    uint16_t fsr;
    do {
        fsr = ((uint16_t)W5500_ReadReg(Sn_TX_FSR,     S2_REG_OP) << 8) |
                          W5500_ReadReg(Sn_TX_FSR + 1, S2_REG_OP);
        if ((g_ms_tick - t) > 200) return;  // timeout 200ms
    } while (fsr < len);

    uint16_t ptr = ((uint16_t)W5500_ReadReg(Sn_TX_WR,     S2_REG_OP) << 8) |
                               W5500_ReadReg(Sn_TX_WR + 1, S2_REG_OP);
    W5500_WriteBuf(ptr, S2_TX_OP, data, len);
    ptr += len;
    W5500_WriteReg(Sn_TX_WR,     S2_REG_OP, (uint8_t)(ptr >> 8));
    W5500_WriteReg(Sn_TX_WR + 1, S2_REG_OP, (uint8_t)(ptr));
    W5500_WriteReg(Sn_CR,        S2_REG_OP, CMD_SEND);
    s2_wait_cr(100);
}

// =============================================================================
// HELPER: Encode MQTT remaining length (variable length encoding)
// =============================================================================
static uint8_t encode_remaining(uint8_t *buf, uint32_t len)
{
    uint8_t i = 0;
    do {
        buf[i] = len & 0x7F;
        len >>= 7;
        if (len > 0) buf[i] |= 0x80;
        i++;
    } while (len > 0);
    return i;
}

// =============================================================================
// HELPER: Tulis 2-byte length + string (MQTT string format)
// =============================================================================
static uint16_t write_str(uint8_t *buf, const char *str)
{
    uint16_t len = (uint16_t)strlen(str);
    buf[0] = (uint8_t)(len >> 8);
    buf[1] = (uint8_t)(len);
    memcpy(&buf[2], str, len);
    return 2 + len;
}

// =============================================================================
// HELPER: Format waktu uptime "0T00:05:23"
// =============================================================================
static void format_uptime(char *out, uint32_t sec)
{
    uint32_t days  = sec / 86400;
    uint32_t hours = (sec % 86400) / 3600;
    uint32_t mins  = (sec % 3600)  / 60;
    uint32_t secs  = sec % 60;
    snprintf(out, 16, "%luT%02lu:%02lu:%02lu",
             (unsigned long)days,
             (unsigned long)hours,
             (unsigned long)mins,
             (unsigned long)secs);
}

// =============================================================================
// HELPER: Format timestamp dummy "2026-09-30T13:35:00"
// (tanpa RTC — pakai uptime saja sebagai placeholder)
// =============================================================================
static void format_time(char *out)
{
    // Placeholder — nanti bisa diganti dengan RTC
    uint32_t s   = s_uptime_sec;
    uint32_t hh  = (s / 3600) % 24;
    uint32_t mm  = (s / 60) % 60;
    uint32_t ss  = s % 60;
    snprintf(out, 24, "2026-09-30T%02lu:%02lu:%02lu",
             (unsigned long)hh,
             (unsigned long)mm,
             (unsigned long)ss);
}

// =============================================================================
// MQTT CONNECT packet
// =============================================================================
static void mqtt_send_connect(const MqttConfig_t *cfg)
{
    // Gunakan bagian kedua s_buf sebagai payload buffer (s_buf = 512 bytes)
    uint8_t  *payload = &s_buf[100];  // offset 100, max 300 bytes
    uint16_t pi = 0;

    // --- Variable header ---
    // Protocol Name "MQTT"
    payload[pi++] = 0x00;
    payload[pi++] = 0x04;
    payload[pi++] = 'M';
    payload[pi++] = 'Q';
    payload[pi++] = 'T';
    payload[pi++] = 'T';

    // Protocol Level 3.1.1
    payload[pi++] = 0x04;

    // Connect Flags
    uint8_t flags = 0x02;           // Clean Session
    flags |= 0x04;                  // Will Flag
    flags |= 0x20;                  // Will Retain, Will QoS=0
    if (cfg->user[0])     flags |= 0x80;  // Username
    if (cfg->password[0]) flags |= 0x40;  // Password
    payload[pi++] = flags;

    // Keepalive
    payload[pi++] = (uint8_t)(MQTT_KEEPALIVE_SEC >> 8);
    payload[pi++] = (uint8_t)(MQTT_KEEPALIVE_SEC);

    // --- Payload ---
    // Client ID
    pi += write_str(&payload[pi], cfg->client_id);

    // Will Topic + Will Message (LWT)
    pi += write_str(&payload[pi], s_t_lwt);
    pi += write_str(&payload[pi], "Offline");

    // Username (jika ada)
    if (cfg->user[0])
        pi += write_str(&payload[pi], cfg->user);

    // Password (jika ada)
    if (cfg->password[0])
        pi += write_str(&payload[pi], cfg->password);

    // --- Fixed header ---
    uint8_t rem[4];
    uint8_t rem_len = encode_remaining(rem, pi);

    uint16_t total = 0;
    s_buf[total++] = MQTT_CONNECT;
    memcpy(&s_buf[total], rem, rem_len); total += rem_len;
    memcpy(&s_buf[total], payload, pi);  total += pi;

    s2_send(s_buf, total);
}

// =============================================================================
// MQTT PUBLISH (QoS 0)
// =============================================================================
void MQTT_Publish(const char *topic, const uint8_t *payload, uint16_t plen)
{
    uint16_t topic_len = (uint16_t)strlen(topic);
    uint32_t rem = 2 + topic_len + plen;  // 2 = topic length field

    uint8_t rem_enc[4];
    uint8_t rem_len = encode_remaining(rem_enc, rem);

    uint16_t i = 0;
    s_buf[i++] = MQTT_PUBLISH;  // QoS 0, no retain
    memcpy(&s_buf[i], rem_enc, rem_len); i += rem_len;
    s_buf[i++] = (uint8_t)(topic_len >> 8);
    s_buf[i++] = (uint8_t)(topic_len);
    memcpy(&s_buf[i], topic, topic_len); i += topic_len;
    memcpy(&s_buf[i], payload, plen);    i += plen;

    s2_send(s_buf, i);
    g_mqtt_pub_count++;
}

// =============================================================================
// MQTT SUBSCRIBE (QoS 0)
// =============================================================================
static void mqtt_subscribe(const char *topic)
{
    uint16_t topic_len = (uint16_t)strlen(topic);
    uint32_t rem = 2 + 2 + topic_len + 1;  // pkt_id + topic_len + topic + QoS

    uint8_t rem_enc[4];
    uint8_t rem_len = encode_remaining(rem_enc, rem);

    uint16_t i = 0;
    s_buf[i++] = MQTT_SUBSCRIBE;
    memcpy(&s_buf[i], rem_enc, rem_len); i += rem_len;
    s_buf[i++] = (uint8_t)(s_pkt_id >> 8);
    s_buf[i++] = (uint8_t)(s_pkt_id++);
    s_buf[i++] = (uint8_t)(topic_len >> 8);
    s_buf[i++] = (uint8_t)(topic_len);
    memcpy(&s_buf[i], topic, topic_len); i += topic_len;
    s_buf[i++] = 0x00;  // QoS 0

    s2_send(s_buf, i);
}

// =============================================================================
// MQTT PINGREQ
// =============================================================================
static void mqtt_pingreq(void)
{
    s_buf[0] = MQTT_PINGREQ;
    s_buf[1] = 0x00;
    s2_send(s_buf, 2);
}

// =============================================================================
// PUBLISH tele/STATE — mirip Tasmota Sonoff single relay
// =============================================================================
static void publish_state(const MqttConfig_t *cfg)
{
    // Pakai static buffer — hindari stack overflow
    format_uptime(s_uptime, s_uptime_sec);
    format_time(s_time);

    char ip_str[16];
    snprintf(ip_str, sizeof(ip_str), "%d.%d.%d.%d",
             g_dhcp_ip[0], g_dhcp_ip[1], g_dhcp_ip[2], g_dhcp_ip[3]);

    snprintf(s_json, sizeof(s_json),
        "{"
        "\"Time\":\"%s\","
        "\"Uptime\":\"%s\","
        "\"UptimeSec\":%lu,"
        "\"Heap\":0,"
        "\"SleepMode\":\"None\","
        "\"Sleep\":0,"
        "\"LoadAvg\":100,"
        "\"MqttCount\":%d,"
        "\"POWER\":\"%s\","
        "\"Eth\":{"
            "\"IP\":\"%s\","
            "\"MAC\":\"00:08:DC:11:22:33\","
            "\"LinkCount\":1"
        "},"
        "\"Hostname\":\"%s\","
        "\"IPAddress\":\"%s\""
        "}",
        s_time,
        s_uptime,
        (unsigned long)s_uptime_sec,
        g_mqtt_conn_count,
        g_can_b_data.relay_state ? "ON" : "OFF",
        ip_str,
        cfg->client_id,
        ip_str);

    MQTT_Publish(s_t_state, (uint8_t *)s_json, (uint16_t)strlen(s_json));
}

// =============================================================================
// PUBLISH tele/SENSOR — data sensor (dummy, forward variabel g_mqtt_sensor)
// =============================================================================
static void publish_sensor(void)
{
    format_time(s_time);

    snprintf(s_json, sizeof(s_json),
        "{"
        "\"Time\":\"%s\","
        "\"Switch1\":\"%s\","
        "\"ENERGY\":{"
            "\"Voltage\":%d.%d,"
            "\"Current\":0.000,"
            "\"Power\":0"
        "},"
        "\"Temperature\":%d,"
        "\"Uptime\":%d,"
        "\"Counter\":%d"
        "}",
        s_time,
        g_can_b_data.relay_state ? "ON" : "OFF",

g_can_b_data.voltage / 10,
g_can_b_data.voltage % 10,
        g_can_b_data.temperature,
        g_can_b_data.uptime,
        g_can_b_data.counter);

    MQTT_Publish(s_t_sensor, (uint8_t *)s_json, (uint16_t)strlen(s_json));
}

// =============================================================================
// PUBLISH tele/INFO1, INFO2, INFO3 — dikirim sekali saat connect
// =============================================================================
static char s_info_topic[96];

static void publish_info(const MqttConfig_t *cfg)
{
    char ip_str[16];
    snprintf(ip_str, sizeof(ip_str), "%d.%d.%d.%d",
             g_dhcp_ip[0], g_dhcp_ip[1], g_dhcp_ip[2], g_dhcp_ip[3]);

    // INFO1 — reuse s_json sbg JSON, s_info_topic sbg topic
    MqttConfig_BuildTopic(cfg, "tele", s_info_topic, sizeof(s_info_topic));
    // Simpan tele base ke s_time sementara
    strncpy(s_time, s_info_topic, sizeof(s_time)-1);
    snprintf(s_info_topic, sizeof(s_info_topic), "%sINFO1", s_time);
    snprintf(s_json, sizeof(s_json),
        "{\"Info1\":{\"Module\":\"STM32F103+W5500\",\"Version\":\"1.0.0\","
        "\"FallbackTopic\":\"cmnd/%s_fb/\",\"GroupTopic\":\"cmnd/stm32s/\"}}",
        cfg->topic);
    MQTT_Publish(s_info_topic, (uint8_t *)s_json, (uint16_t)strlen(s_json));

    // INFO2
    snprintf(s_info_topic, sizeof(s_info_topic), "%sINFO2", s_time);
    snprintf(s_json, sizeof(s_json),
        "{\"Info2\":{\"WebServerMode\":\"Admin\",\"Hostname\":\"%s\","
        "\"IPAddress\":\"%s\"}}",
        cfg->client_id, ip_str);
    MQTT_Publish(s_info_topic, (uint8_t *)s_json, (uint16_t)strlen(s_json));

    // INFO3
    snprintf(s_info_topic, sizeof(s_info_topic), "%sINFO3", s_time);
    snprintf(s_json, sizeof(s_json),
        "{\"Info3\":{\"RestartReason\":\"Power on\",\"BootCount\":1}}");
    MQTT_Publish(s_info_topic, (uint8_t *)s_json, (uint16_t)strlen(s_json));
}


// =============================================================================
// PUBLISH STATUS — persis Tasmota, disesuaikan STM32+W5500+CAN
// =============================================================================
static void build_stat_topic(const char *suffix, char *out, uint16_t len)
{
    // Bangun stat/topic/SUFFIXnya
    MqttConfig_BuildTopic((const MqttConfig_t *)0, "stat", out, len);
    // Tidak bisa pakai cfg di sini — pakai s_t_result sebagai base
    // s_t_result = "stat/topic/RESULT" → ambil base sampai "RESULT"
    uint16_t base_len = (uint16_t)(strrchr(s_t_result, '/') - s_t_result + 1);
    memcpy(out, s_t_result, base_len);
    out[base_len] = '\0';
    strncat(out, suffix, len - base_len - 1);
}

static void publish_status_n(const MqttConfig_t *cfg, uint8_t n)
{
    // Build topic stat/topic/STATUSn
    char ip_str[16];
    snprintf(ip_str, sizeof(ip_str), "%d.%d.%d.%d",
             g_dhcp_ip[0], g_dhcp_ip[1], g_dhcp_ip[2], g_dhcp_ip[3]);

    char gw_str[16];
    snprintf(gw_str, sizeof(gw_str), "%d.%d.%d.%d",
             g_dhcp_ip[0], g_dhcp_ip[1], g_dhcp_ip[2], 1);

    // Build stat topic
    uint16_t base_len = (uint16_t)(strrchr(s_t_result, '/') - s_t_result + 1);
    memcpy(s_info_topic, s_t_result, base_len);
    s_info_topic[base_len] = '\0';

    if (n == 0) {
        strncat(s_info_topic, "STATUS", sizeof(s_info_topic)-base_len-1);
    } else {
        char num[10];
        snprintf(num, sizeof(num), "STATUS%d", n);
        strncat(s_info_topic, num, sizeof(s_info_topic)-base_len-1);
    }

    switch (n) {
        case 0:  // STATUS — ringkasan
            snprintf(s_json, sizeof(s_json),
                "{\"Status\":{\"Module\":1,\"DeviceName\":\"%s\","
                "\"FriendlyName\":[\"  %s\"],\"Topic\":\"%s\","
                "\"OtaUrl\":\"http://%s/ota\","
                "\"Power\":\"%s\",\"PowerOnState\":1,"
                "\"TelePeriod\":%d}}",
                cfg->client_id, cfg->topic, cfg->topic,
                ip_str,
                g_can_b_data.relay_state ? "1" : "0",
                cfg->tele_period);
            break;

        case 1:  // STATUS1 — parameter
            format_uptime(s_uptime, s_uptime_sec);
            snprintf(s_json, sizeof(s_json),
                "{\"StatusPRM\":{\"OtaUrl\":\"http://%s/ota\","
                "\"RestartReason\":\"Power on\","
                "\"Uptime\":\"%s\",\"UptimeSec\":%lu,"
                "\"Sleep\":0,\"BootCount\":1,"
                "\"TelePeriod\":%d}}",
                ip_str, s_uptime, (unsigned long)s_uptime_sec,
                cfg->tele_period);
            break;

        case 2:  // STATUS2 — firmware
            snprintf(s_json, sizeof(s_json),
                "{\"StatusFWR\":{\"Version\":\"1.0.0(stm32-w5500)\","
                "\"BuildDateTime\":\"2026-09-30T00:00:00\","
                "\"CpuFrequency\":72,\"Hardware\":\"STM32F103C8T6\","
                "\"Core\":\"LL Driver\"}}");
            break;

        case 3:  // STATUS3 — log/serial/CAN
            snprintf(s_json, sizeof(s_json),
                "{\"StatusLOG\":{\"TelePeriod\":%d,"
                "\"CANSpeed\":500,\"RS485Baud\":115200,"
                "\"SerialConfig\":\"8N1\"}}",
                cfg->tele_period);
            break;

        case 4:  // STATUS4 — memori
            snprintf(s_json, sizeof(s_json),
                "{\"StatusMEM\":{\"ProgramSize\":34,"
                "\"FlashSize\":8192,\"FlashChipId\":\"W25Q64\","
                "\"FlashFrequency\":36,\"FlashMode\":\"SPI\","
                "\"RAM\":20,\"RAMFree\":16}}");
            break;

        case 5:  // STATUS5 — network
            snprintf(s_json, sizeof(s_json),
                "{\"StatusNET\":{\"Hostname\":\"%s\","
                "\"IPAddress\":\"%s\",\"Gateway\":\"%s\","
                "\"Subnetmask\":\"%d.%d.%d.%d\","
                "\"DNSServer\":\"%s\","
                "\"Mac\":\"00:08:DC:11:22:33\"}}",
                cfg->client_id, ip_str, gw_str,
                g_dhcp_ip[0], g_dhcp_ip[1], g_dhcp_ip[2], 0,
                gw_str);
            break;

        case 6:  // STATUS6 — MQTT
            snprintf(s_json, sizeof(s_json),
                "{\"StatusMQT\":{\"MqttHost\":\"%s\","
                "\"MqttPort\":%d,\"MqttClient\":\"%s\","
                "\"MqttUser\":\"%s\",\"MqttCount\":%d,"
                "\"KEEPALIVE\":%d,\"MqttTLS\":0}}",
                cfg->host, cfg->port, cfg->client_id,
                cfg->user, g_mqtt_conn_count,
                MQTT_KEEPALIVE_SEC);
            break;

        case 7:  // STATUS7 — time (tanpa RTC, pakai uptime)
            format_uptime(s_uptime, s_uptime_sec);
            format_time(s_time);
            snprintf(s_json, sizeof(s_json),
                "{\"StatusTIM\":{\"Local\":\"%s\","
                "\"Uptime\":\"%s\",\"UptimeSec\":%lu,"
                "\"Timezone\":\"+07:00\"}}",
                s_time, s_uptime, (unsigned long)s_uptime_sec);
            break;

        case 8:  // STATUS8 — sensor (ENERGY dummy)
            format_time(s_time);
            snprintf(s_json, sizeof(s_json),
                   "{\"StatusSNS\":{\"Time\":\"%s\","
                   "\"Switch1\":\"%s\","
                   "\"ENERGY\":{\"Voltage\":%d.%d,"
                   "\"Current\":0.000,\"Power\":0,\"Total\":0.000},"
                   "\"Temperature\":%d,"
                   "\"Uptime\":%d,"
                   "\"Counter\":%d}}",
                   s_time,
                   g_can_b_data.relay_state ? "ON" : "OFF",
                	g_can_b_data.voltage / 10,
					g_can_b_data.voltage % 10,
                   g_can_b_data.temperature,
                   g_can_b_data.uptime,
                   g_can_b_data.counter);
            break;

        case 9:  // STATUS9 — power state
        	 snprintf(s_json, sizeof(s_json),
        	        "{\"StatusPWR\":{\"POWER\":\"%s\","
        	        "\"PowerOnState\":1,\"LedState\":1}}",
        	        g_can_b_data.relay_state ? "ON" : "OFF");
            break;

        case 10:  // STATUS10 — CAN Bus info
        	snprintf(s_json, sizeof(s_json),
        	        "{\"StatusCAN\":{\"CANSpeed\":500,"
        	        "\"CANStatus\":\"Ready\","
        	        "\"CANTxCount\":%lu,\"CANRxCount\":%lu}}",
        	        (unsigned long)g_can_tx_count,
        	        (unsigned long)g_can_rx_count);
            break;

        case 11:  // STATUS11 — ringkasan STATE
            format_uptime(s_uptime, s_uptime_sec);
            format_time(s_time);
            snprintf(s_json, sizeof(s_json),
                "{\"StatusSTS\":{\"Time\":\"%s\","
                "\"Uptime\":\"%s\",\"UptimeSec\":%lu,"
                "\"MqttCount\":%d,\"POWER\":\"%s\","
                "\"Eth\":{\"IP\":\"%s\","
                "\"MAC\":\"00:08:DC:11:22:33\","
                "\"LinkCount\":1}}}",
                s_time, s_uptime, (unsigned long)s_uptime_sec,
                g_mqtt_conn_count,
                g_can_b_data.relay_state ? "ON" : "OFF",
                ip_str);
            break;

        default:
            return;
    }

    MQTT_Publish(s_info_topic, (uint8_t *)s_json, (uint16_t)strlen(s_json));
}

// Proses antrian status command — dipanggil tiap loop saat CONNECTED
// STATUS0 = kirim STATUS, STATUS1...STATUS11 satu per satu
static void process_status_queue(const MqttConfig_t *cfg)
{
    if (s_status_cmd < 0) return;

    if (s_status_cmd == 0) {
        // Kirim satu per satu tiap loop: STATUS, STATUS1, ..., STATUS11
        publish_status_n(cfg, s_status_step);
        s_status_step++;
        if (s_status_step > 11) {
            s_status_cmd  = -1;
            s_status_step = 0;
        }
    } else {
        // Kirim STATUS tertentu langsung
        publish_status_n(cfg, (uint8_t)s_status_cmd);
        s_status_cmd = -1;
    }
}

// =============================================================================
// HANDLE incoming MQTT packet (subscribe callback)
// =============================================================================
static void mqtt_handle_incoming(const MqttConfig_t *cfg)
{
    #define R16(reg) (((uint16_t)W5500_ReadReg(reg, S2_REG_OP) << 8) | W5500_ReadReg((reg)+1, S2_REG_OP))
    uint16_t len = R16(Sn_RX_RSR);
    if (!len) return;

    uint16_t rd = R16(Sn_RX_RD), grab = (len > MQTT_BUF_SIZE) ? MQTT_BUF_SIZE : len;
    W5500_ReadBuf(rd, S2_RX_OP, s_buf, grab);
    rd += grab;
    W5500_WriteReg(Sn_RX_RD, S2_REG_OP, rd >> 8);
    W5500_WriteReg(Sn_RX_RD + 1, S2_REG_OP, (uint8_t)rd);
    W5500_WriteReg(Sn_CR, S2_REG_OP, CMD_RECV);
    s2_wait_cr(10);

    uint8_t type = s_buf[0] & 0xF0;
    if (type == MQTT_CONNACK) {
        if (!s_buf[3]) { g_mqtt_conn_count++; g_mqtt_state = MQTT_STATE_CONNECTED; s_startup_step = 0; s_last_ping = s_last_pub = g_ms_tick; }
        else { g_mqtt_state = MQTT_STATE_RECONNECT; s_tick_ref = g_ms_tick; }
        return;
    }
    if (type != MQTT_PUBLISH) return;

    uint8_t  hsz = (s_buf[1] & 0x80) ? 3 : 2;
    uint16_t tlen = ((uint16_t)s_buf[hsz] << 8) | s_buf[hsz + 1];
    if (hsz + 2 + tlen >= grab || tlen >= sizeof(s_info_topic)) return;

    memcpy(s_info_topic, &s_buf[hsz + 2], tlen);
    s_info_topic[tlen] = '\0';

    char *pay = (char *)&s_buf[hsz + 2 + tlen];
    s_buf[grab] = '\0'; // terminasi string payload

    size_t clen = strlen(s_t_cmnd);
    if (strncasecmp(s_info_topic, s_t_cmnd, clen)) return;
    char *cmd = s_info_topic + clen;

    // Dispatch Command
    if (!strcasecmp(cmd, "POWER")) {
        if (!strcasecmp(pay, "ON")) g_can_b_data.relay_state = 1;
        else if (!strcasecmp(pay, "OFF")) g_can_b_data.relay_state = 0;
        else if (!strcasecmp(pay, "TOGGLE")) g_can_b_data.relay_state ^= 1;
        CAN_Bus_SendCmd(&hcan, g_can_b_data.relay_state ? RELAY_CMD_ON : RELAY_CMD_OFF);
        const char *st = g_can_b_data.relay_state ? "ON" : "OFF";
        snprintf(s_json, sizeof(s_json), "{\"POWER\":\"%s\"}", st);
        MQTT_Publish(s_t_result, (uint8_t *)s_json, strlen(s_json));
        MQTT_Publish(s_t_power,  (uint8_t *)st, strlen(st));
    }
    else if (!strcasecmp(cmd, "TelePeriod")) s_pub_interval = (uint32_t)atoi(pay) * 1000UL;
    else if (!strcasecmp(cmd, "Status"))     { s_status_cmd = (int8_t)atoi(pay); s_status_step = 0; }
    else if (!strcasecmp(cmd, "Restart") && pay[0] == '1') {
        MQTT_Publish(s_t_result, (uint8_t *)"{\"Restart\":\"Restarting\"}", 24);
        volatile uint32_t d = 720000; while (d--);
        extern void NVIC_SystemReset(void); NVIC_SystemReset();
    }
    else if (!strcasecmp(cmd, "CAN")) {
        char *pi = strstr(pay, "\"id\":\""), *pm = strstr(pay, "\"msg\":\"");
        if (pi && pm) {
            uint8_t d[8], dlc = 0, *p = (uint8_t *)pm + 7;
            #define N(c) (((c)|0x20) > '9' ? ((c)|0x20)-'a'+10 : (c)-'0')
            while (*p != '"' && *(p+1) != '"' && dlc < 8) { d[dlc++] = (N(*p) << 4) | N(*(p+1)); p += 2; }
            #undef N
            CAN_TxHeaderTypeDef hdr = { .StdId = strtoul(pi + 6, NULL, 16), .DLC = dlc };
            uint32_t box;
            if (dlc && HAL_CAN_AddTxMessage(&hcan, &hdr, d, &box) == HAL_OK) {
                snprintf(s_json, sizeof(s_json), "{\"CAN_OK\":{\"id\":\"%lX\",\"dlc\":%u}}", hdr.StdId, dlc);
                MQTT_Publish(s_t_result, (uint8_t *)s_json, strlen(s_json));
            }
        }
    }
    #undef R16
}
// =============================================================================
// FUNGSI PUBLIK: Init
// =============================================================================
void MQTT_Init(void)
{
    // Alokasi buffer Socket 2
    W5500_WriteReg(0x001E, S2_REG_OP, 2);  // Socket 2 TX 2KB
    W5500_WriteReg(0x001F, S2_REG_OP, 2);  // Socket 2 RX 2KB

    g_mqtt_state    = MQTT_STATE_IDLE;
    g_mqtt_conn_count = 0;
    g_mqtt_pub_count  = 0;
    s_uptime_sec    = 0;
    s_last_uptime   = g_ms_tick;
}

// =============================================================================
// FUNGSI PUBLIK: Process (dipanggil setiap loop)
// =============================================================================
void MQTT_Process(const MqttConfig_t *cfg)
{
    if (!cfg->enabled) {
        g_mqtt_state = MQTT_STATE_DISABLED;
        return;
    }

    // Update uptime
    if ((g_ms_tick - s_last_uptime) >= 1000) {
        s_uptime_sec++;
        s_last_uptime += 1000;
    }

    // Interval publish dari config
    if (s_pub_interval == 0) {
        s_pub_interval = (uint32_t)cfg->tele_period * 1000UL;
    }

    uint8_t sr = W5500_ReadReg(Sn_SR, S2_REG_OP);

    switch (g_mqtt_state) {

        // =====================================================================
        case MQTT_STATE_IDLE:
        // =====================================================================
        {
            // Bangun topic cache dari config — reuse s_json sebagai temp buffer
            MqttConfig_BuildTopic(cfg, "tele", s_json, 80);
            snprintf(s_t_lwt,    sizeof(s_t_lwt),    "%sLWT",    s_json);
            snprintf(s_t_state,  sizeof(s_t_state),  "%sSTATE",  s_json);
            snprintf(s_t_sensor, sizeof(s_t_sensor), "%sSENSOR", s_json);
            MqttConfig_BuildTopic(cfg, "stat", s_json, 80);
            snprintf(s_t_result, sizeof(s_t_result), "%sRESULT", s_json);
            snprintf(s_t_power,  sizeof(s_t_power),  "%sPOWER",  s_json);
            MqttConfig_BuildTopic(cfg, "cmnd", s_t_cmnd, sizeof(s_t_cmnd));

            // Tutup socket dulu kalau perlu
            if (sr != SOCK_CLOSED) {
                W5500_WriteReg(Sn_CR, S2_REG_OP, CMD_CLOSE);
                s2_wait_cr(50);
                break;
            }

            // Buka socket TCP
            W5500_WriteReg(Sn_MR,      S2_REG_OP, 0x01);   // TCP mode
            W5500_WriteReg(Sn_PORT,    S2_REG_OP, 0x00);
            W5500_WriteReg(Sn_PORT+1,  S2_REG_OP, 0xC8);   // source port 200
            W5500_WriteReg(Sn_CR,      S2_REG_OP, CMD_OPEN);
            s2_wait_cr(50);

            // Parse IP string ke bytes manual (hindari sscanf di embedded)
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
            W5500_WriteBuf(Sn_DIPR,    S2_REG_OP, ip, 4);
            W5500_WriteReg(Sn_DPORT,   S2_REG_OP, (uint8_t)(cfg->port >> 8));
            W5500_WriteReg(Sn_DPORT+1, S2_REG_OP, (uint8_t)(cfg->port));

            // Connect
            W5500_WriteReg(Sn_CR, S2_REG_OP, CMD_CONNECT);
            s2_wait_cr(50);

            g_mqtt_state = MQTT_STATE_TCP_CONN;
            s_tick_ref   = g_ms_tick;
            break;
        }

        // =====================================================================
        case MQTT_STATE_TCP_CONN:
        // =====================================================================
            if (sr == SOCK_ESTABLISHED) {
                // TCP tersambung → kirim MQTT CONNECT
                mqtt_send_connect(cfg);
                g_mqtt_state = MQTT_STATE_MQTT_CONN;
                s_tick_ref   = g_ms_tick;
            } else if ((g_ms_tick - s_tick_ref) > MQTT_CONN_TIMEOUT_MS) {
                // Timeout → coba lagi
                W5500_WriteReg(Sn_CR, S2_REG_OP, CMD_CLOSE);
                s2_wait_cr(50);
                g_mqtt_state = MQTT_STATE_RECONNECT;
                s_tick_ref   = g_ms_tick;
            }
            break;

        // =====================================================================
        case MQTT_STATE_MQTT_CONN:
        // =====================================================================
            mqtt_handle_incoming(cfg);  // tunggu CONNACK
            if (g_mqtt_state == MQTT_STATE_CONNECTED) break;  // berhasil
            if ((g_ms_tick - s_tick_ref) > MQTT_CONN_TIMEOUT_MS) {
                W5500_WriteReg(Sn_CR, S2_REG_OP, CMD_CLOSE);
                s2_wait_cr(50);
                g_mqtt_state = MQTT_STATE_RECONNECT;
                s_tick_ref   = g_ms_tick;
            }
            break;

        // =====================================================================
        case MQTT_STATE_CONNECTED:
        // =====================================================================
            // Cek koneksi TCP masih hidup
            if (sr != SOCK_ESTABLISHED) {
                g_mqtt_state = MQTT_STATE_RECONNECT;
                s_tick_ref   = g_ms_tick;
                break;
            }

            // Startup sequence — kirim satu per satu tiap loop
            // agar tidak overflow TX buffer 2KB
            if (s_startup_step < 6) {
                switch (s_startup_step) {
                    case 0:
                        // Subscribe cmnd/# — reuse s_json sbg temp
                        snprintf(s_json, sizeof(s_json), "%s#", s_t_cmnd);
                        mqtt_subscribe(s_json);
                        break;
                    case 1:
                        MQTT_Publish(s_t_lwt, (uint8_t *)"Online", 6);
                        break;
                    case 2:
                        publish_info(cfg);
                        break;
                    case 3:
                        publish_state(cfg);
                        break;
                    case 4:
                        publish_sensor();
                        break;
                    case 5:
                        s_last_pub = g_ms_tick;
                        break;
                    default: break;
                }
                s_startup_step++;
                break;
            }

            // Baca incoming (command dari broker)
            mqtt_handle_incoming(cfg);

            // Proses antrian Status command (STATUS0 kirim satu per loop)
            process_status_queue(cfg);

            // PINGREQ setiap KEEPALIVE/2 detik
            if ((g_ms_tick - s_last_ping) >= (MQTT_KEEPALIVE_SEC * 500UL)) {
                mqtt_pingreq();
                s_last_ping = g_ms_tick;
            }

            // Publish tele STATE + SENSOR setiap tele_period
            if ((g_ms_tick - s_last_pub) >= s_pub_interval) {
                publish_state(cfg);
                publish_sensor();
                s_last_pub = g_ms_tick;
            }
            break;

        // =====================================================================
        case MQTT_STATE_RECONNECT:
        // =====================================================================
            // Tunggu 5 detik sebelum reconnect
            if ((g_ms_tick - s_tick_ref) > 5000) {
                g_mqtt_state = MQTT_STATE_IDLE;
            }
            break;

        case MQTT_STATE_DISABLED:
        default:
            break;
    }
}

// =============================================================================
// FUNGSI PUBLIK: IsConnected
// =============================================================================
uint8_t MQTT_IsConnected(void)
{
    return (g_mqtt_state == MQTT_STATE_CONNECTED) ? 1 : 0;
}
