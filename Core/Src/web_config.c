#include "web_config.h"
#include "mqtt_config.h"
#include "lfs_config.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// =============================================================================
// FORWARD DECLARATION — fungsi W5500 dari main.c
// =============================================================================
extern void    W5500_SendTCP (const uint8_t *data, uint16_t len);
extern void    W5500_ReadBuf (uint16_t addr, uint8_t block, uint8_t *buf, uint16_t len);
extern uint8_t W5500_ReadReg (uint16_t addr, uint8_t block);
extern void    W5500_WriteReg(uint16_t addr, uint8_t block, uint8_t data);

// =============================================================================
// HTML CONFIG PAGE — hardcode di ROM, ditulis ke LittleFS saat pertama boot
// =============================================================================
static const char HTML_CONFIG_PAGE[] =
"<!DOCTYPE html>"
"<html lang='id'>"
"<head>"
"<meta charset='UTF-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>STM32 MQTT Config</title>"
"<style>"
"*{box-sizing:border-box;margin:0;padding:0}"
"body{font-family:Arial,sans-serif;background:#1a1a2e;color:#eee;"
"min-height:100vh;display:flex;align-items:center;justify-content:center}"
".card{background:#16213e;border-radius:8px;padding:24px;width:100%;"
"max-width:420px;box-shadow:0 4px 24px #0008}"
"h2{color:#0f9b8e;margin-bottom:6px;font-size:1.2em;letter-spacing:1px}"
".sub{color:#888;font-size:.8em;margin-bottom:20px}"
"label{display:block;font-size:.8em;color:#aaa;margin:12px 0 4px}"
"input{width:100%;padding:8px 10px;border:1px solid #0f9b8e44;border-radius:4px;"
"background:#0d1b2a;color:#eee;font-size:.9em;outline:none}"
"input:focus{border-color:#0f9b8e}"
".toggle{display:flex;align-items:center;gap:8px;margin-top:14px}"
".toggle input{width:auto}"
"button{width:100%;margin-top:18px;padding:10px;background:#0f9b8e;"
"border:none;border-radius:4px;color:#fff;font-size:1em;cursor:pointer}"
"button:hover{background:#0d8077}"
"#msg{margin-top:12px;font-size:.85em;text-align:center;min-height:18px}"
".ok{color:#4caf50}.err{color:#f44336}"
"</style></head>"
"<body><div class='card'>"
"<h2>&#9881; MQTT Configuration</h2>"
"<p class='sub'>STM32 W5500 &mdash; Tasmota Style</p>"
"<label>MQTT Host</label>"
"<input type='text' id='host' maxlength='63' placeholder='192.168.1.100'>"
"<label>MQTT Port</label>"
"<input type='number' id='port' min='1' max='65535' placeholder='1883'>"
"<label>Username</label>"
"<input type='text' id='user' maxlength='31' placeholder='(kosong jika tidak ada)'>"
"<label>Password</label>"
"<input type='password' id='pass' maxlength='31'>"
"<label>Topic</label>"
"<input type='text' id='topic' maxlength='31' placeholder='stm32/cmnd'>"
"<label>Client ID</label>"
"<input type='text' id='cid' maxlength='31' placeholder='STM32_W5500'>"
"<div class='toggle'>"
"<input type='checkbox' id='en'>"
"<label for='en' style='margin:0'>MQTT Enabled</label>"
"</div>"
"<button onclick='save()'>Simpan</button>"
"<div id='msg'></div>"
"</div>"
"<script>"
"async function load(){"
"try{"
"let r=await fetch('/config/data');"
"let d=await r.json();"
"document.getElementById('host').value=d.host||'';"
"document.getElementById('port').value=d.port||1883;"
"document.getElementById('user').value=d.user||'';"
"document.getElementById('pass').value=d.password||'';"
"document.getElementById('topic').value=d.topic||'';"
"document.getElementById('cid').value=d.client_id||'';"
"document.getElementById('en').checked=d.enabled==1;"
"}catch(e){}}"
"async function save(){"
"let m=document.getElementById('msg');"
"m.className='';m.innerText='Menyimpan...';"
"let fd=new URLSearchParams();"
"fd.append('host',document.getElementById('host').value);"
"fd.append('port',document.getElementById('port').value);"
"fd.append('user',document.getElementById('user').value);"
"fd.append('password',document.getElementById('pass').value);"
"fd.append('topic',document.getElementById('topic').value);"
"fd.append('client_id',document.getElementById('cid').value);"
"fd.append('enabled',document.getElementById('en').checked?'1':'0');"
"try{"
"let r=await fetch('/config',{method:'POST',body:fd.toString(),"
"headers:{'Content-Type':'application/x-www-form-urlencoded'}});"
"let t=await r.text();"
"if(r.ok){m.className='ok';m.innerText='Tersimpan! '+t;}"
"else{m.className='err';m.innerText='Gagal: '+t;}"
"}catch(e){m.className='err';m.innerText='Error: '+e;}}"
"load();"
"</script></body></html>";

// =============================================================================
// BUFFER STREAMING
// =============================================================================
#define STREAM_CHUNK  256
static uint8_t s_stream_buf[STREAM_CHUNK];

// =============================================================================
// HELPER: URL decode
// =============================================================================
static void url_decode(char *dst, const char *src, size_t max_len)
{
    size_t i = 0;
    while (*src && i < max_len - 1) {
        if (*src == '%' && src[1] && src[2]) {
            char hex[3] = { src[1], src[2], 0 };
            dst[i++] = (char)strtol(hex, NULL, 16);
            src += 3;
        } else if (*src == '+') {
            dst[i++] = ' ';
            src++;
        } else {
            dst[i++] = *src++;
        }
    }
    dst[i] = '\0';
}

// =============================================================================
// HELPER: Ambil nilai field dari body URL-encoded
// =============================================================================
static void parse_field(const char *body, const char *key,
                        char *out, size_t out_len)
{
    char search[48];
    snprintf(search, sizeof(search), "%s=", key);

    const char *p = strstr(body, search);
    if (!p) { out[0] = '\0'; return; }

    p += strlen(search);
    const char *end = strchr(p, '&');
    size_t len = end ? (size_t)(end - p) : strlen(p);
    if (len >= out_len) len = out_len - 1;

    char tmp[200];
    memcpy(tmp, p, len);
    tmp[len] = '\0';
    url_decode(out, tmp, out_len);
}

// =============================================================================
// HELPER: Kirim HTTP response teks
// =============================================================================
static void send_response(uint16_t code, const char *code_str, const char *body)
{
    char hdr[128];
    snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 %d %s\r\n"
        "Content-Type: text/plain\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n\r\n",
        code, code_str, (int)strlen(body));
    W5500_SendTCP((uint8_t *)hdr,  (uint16_t)strlen(hdr));
    W5500_SendTCP((uint8_t *)body, (uint16_t)strlen(body));
}

// =============================================================================
// HELPER: Kirim JSON config saat ini
// =============================================================================
static void send_config_json(void)
{
    MqttConfig_t cfg;
    MqttConfig_Load(&cfg);

    char json[300];
    snprintf(json, sizeof(json),
        "{\"host\":\"%s\",\"port\":%d,\"user\":\"%s\","
        "\"password\":\"%s\",\"topic\":\"%s\","
        "\"client_id\":\"%s\",\"enabled\":%d}",
        cfg.host, cfg.port, cfg.user,
        cfg.password, cfg.topic,
        cfg.client_id, cfg.enabled);

    char hdr[128];
    snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n\r\n",
        (int)strlen(json));
    W5500_SendTCP((uint8_t *)hdr,  (uint16_t)strlen(hdr));
    W5500_SendTCP((uint8_t *)json, (uint16_t)strlen(json));
}

// =============================================================================
// HELPER: Streaming file dari LittleFS ke TCP
// =============================================================================
static void stream_file_from_lfs(const char *path)
{
    lfs_file_t file;

    int err = lfs_file_open(&littlefs, &file, path, LFS_O_RDONLY);
    if (err < 0) {
        send_response(404, "Not Found", "File tidak ditemukan");
        return;
    }

    lfs_soff_t file_size = lfs_file_size(&littlefs, &file);
    if (file_size < 0) file_size = 0;

    char hdr[128];
    snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html\r\n"
        "Content-Length: %ld\r\n"
        "Connection: close\r\n\r\n",
        (long)file_size);
    W5500_SendTCP((uint8_t *)hdr, (uint16_t)strlen(hdr));

    // Streaming chunk per chunk
    lfs_ssize_t n;
    while ((n = lfs_file_read(&littlefs, &file, s_stream_buf, STREAM_CHUNK)) > 0) {
        W5500_SendTCP(s_stream_buf, (uint16_t)n);
    }

    lfs_file_close(&littlefs, &file);
}

// =============================================================================
// HELPER: Tulis HTML ke LittleFS (pertama kali boot)
// =============================================================================
static void write_html_to_lfs(void)
{
    lfs_file_t file;

    int err = lfs_file_open(&littlefs, &file, CONFIG_HTML_FILE,
                            LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
    if (err < 0) return;

    const char *ptr       = HTML_CONFIG_PAGE;
    uint32_t    remaining = (uint32_t)strlen(HTML_CONFIG_PAGE);

    while (remaining > 0) {
        uint32_t    chunk   = (remaining > STREAM_CHUNK) ? STREAM_CHUNK : remaining;
        lfs_ssize_t written = lfs_file_write(&littlefs, &file, ptr, chunk);
        if (written < 0) break;
        ptr       += written;
        remaining -= (uint32_t)written;
    }

    lfs_file_close(&littlefs, &file);
}

// =============================================================================
// FUNGSI PUBLIK
// =============================================================================

void WebConfig_Init(void)
{
    lfs_file_t file;

    // Cek apakah /config.html sudah ada di LittleFS
    int err = lfs_file_open(&littlefs, &file, CONFIG_HTML_FILE, LFS_O_RDONLY);
    if (err == LFS_ERR_OK) {
        lfs_file_close(&littlefs, &file);
        return; // Sudah ada, tidak perlu tulis ulang
    }

    // Belum ada → tulis dari ROM ke LittleFS
    write_html_to_lfs();
}

void WebConfig_Handle(uint8_t *rx_buf, uint16_t rx_len)
{
    (void)rx_len;
    char *req = (char *)rx_buf;

    // =========================================================
    // GET /config/data → JSON config saat ini (dipanggil JS)
    // =========================================================
    if (strstr(req, "GET /config/data") != NULL) {
        send_config_json();
        return;
    }

    // =========================================================
    // GET /config → streaming HTML dari LittleFS
    // =========================================================
    if (strstr(req, "GET /config") != NULL) {
        stream_file_from_lfs(CONFIG_HTML_FILE);
        return;
    }

    // =========================================================
    // POST /config → parse form, simpan config MQTT
    // =========================================================
    if (strstr(req, "POST /config") != NULL) {
        char *body = strstr(req, "\r\n\r\n");
        if (!body) {
            send_response(400, "Bad Request", "Body tidak ditemukan");
            return;
        }
        body += 4;

        MqttConfig_t cfg;
        memset(&cfg, 0, sizeof(cfg));

        parse_field(body, "host",      cfg.host,      sizeof(cfg.host));
        parse_field(body, "user",      cfg.user,      sizeof(cfg.user));
        parse_field(body, "password",  cfg.password,  sizeof(cfg.password));
        parse_field(body, "topic",     cfg.topic,     sizeof(cfg.topic));
        parse_field(body, "client_id", cfg.client_id, sizeof(cfg.client_id));

        char port_str[8] = {0};
        parse_field(body, "port", port_str, sizeof(port_str));
        cfg.port = (uint16_t)atoi(port_str);
        if (cfg.port == 0) cfg.port = 1883;

        char en_str[4] = {0};
        parse_field(body, "enabled", en_str, sizeof(en_str));
        cfg.enabled = (en_str[0] == '1') ? 1 : 0;

        cfg.magic = MQTT_CONFIG_MAGIC;

        int err = MqttConfig_Save(&cfg);
        if (err == LFS_ERR_OK) {
            send_response(200, "OK", "Config tersimpan!");
        } else {
            send_response(500, "Error", "Gagal simpan ke flash");
        }
        return;
    }
}
