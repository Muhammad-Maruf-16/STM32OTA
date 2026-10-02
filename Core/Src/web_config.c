#include "web_config.h"
#include "mqtt_config.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// =============================================================================
// FORWARD DECLARATION
// =============================================================================
extern void W5500_SendTCP(const uint8_t *data, uint16_t len);
uint8_t g_config_updated = 0;

// =============================================================================
// HTML — persis Tasmota MQTT Config page
// Dark theme, layout 2-kolom untuk field pendek (port, teleperiod)
// =============================================================================
static const char HTML_CONFIG_PAGE[] =
"<!DOCTYPE html>"
"<html>"
"<head>"
"<meta charset='UTF-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>STM32 - Configuration MQTT</title>"
"<style>"
"*{box-sizing:border-box}"
"body{font-family:'Segoe UI',Arial,sans-serif;background:#1e1e1e;color:#ddd;"
"margin:0;padding:0;font-size:14px}"
"/* Header bar mirip Tasmota */"
".header{background:#1fa3ec;padding:10px 16px;display:flex;"
"align-items:center;justify-content:space-between}"
".header h3{margin:0;color:#fff;font-size:16px;font-weight:600}"
".header .sub{color:#cde;font-size:11px}"
"/* Navigation tabs */"
".nav{background:#282828;padding:0 12px;display:flex;gap:2px;flex-wrap:wrap}"
".nav a{color:#aaa;text-decoration:none;padding:8px 12px;font-size:12px;"
"border-bottom:2px solid transparent;display:block}"
".nav a.active{color:#1fa3ec;border-bottom-color:#1fa3ec}"
".nav a:hover{color:#fff}"
"/* Main content */"
".main{max-width:480px;margin:0 auto;padding:16px}"
"/* Section title */"
".sect{font-size:11px;font-weight:700;color:#1fa3ec;letter-spacing:1px;"
"text-transform:uppercase;margin:18px 0 8px;border-bottom:1px solid #333;padding-bottom:4px}"
"/* Field row */"
".field{margin-bottom:10px}"
".field label{display:block;font-size:12px;color:#999;margin-bottom:3px}"
".field input[type=text],.field input[type=password],.field input[type=number]{"
"width:100%;padding:6px 8px;background:#2d2d2d;border:1px solid #444;"
"border-radius:3px;color:#ddd;font-size:13px;outline:none}"
".field input:focus{border-color:#1fa3ec;background:#333}"
"/* 2-kolom */"
".row2{display:flex;gap:10px}"
".row2 .field{flex:1}"
"/* Topic preview box */"
".preview{background:#1a1a1a;border:1px solid #333;border-radius:3px;"
"padding:8px 10px;font-size:11px;font-family:monospace;color:#888;margin-top:6px}"
".preview span{color:#1fa3ec}"
"/* Checkbox toggle */"
".toggle{display:flex;align-items:center;gap:8px;padding:4px 0}"
".toggle input{width:16px;height:16px;cursor:pointer;accent-color:#1fa3ec}"
".toggle label{font-size:13px;color:#ccc;cursor:pointer;margin:0}"
"/* Buttons */"
".btns{margin-top:20px;display:flex;gap:8px}"
".btn{flex:1;padding:9px;border:none;border-radius:3px;font-size:13px;"
"cursor:pointer;font-weight:600}"
".btn-save{background:#1fa3ec;color:#fff}"
".btn-save:hover{background:#1b8fd4}"
".btn-reset{background:#e74c3c;color:#fff}"
".btn-reset:hover{background:#c0392b}"
"/* Status message */"
"#msg{margin-top:10px;padding:8px 10px;border-radius:3px;font-size:12px;"
"display:none;text-align:center}"
".msg-ok{background:#1e3d1e;color:#4caf50;border:1px solid #2e5c2e}"
".msg-err{background:#3d1e1e;color:#f44336;border:1px solid #5c2e2e}"
".msg-info{background:#1e2d3d;color:#2196f3;border:1px solid #2e3d5c}"
"/* Topic info */"
".tinfo{font-size:11px;color:#666;margin-top:4px;line-height:1.6}"
".tinfo b{color:#1fa3ec}"
"/* Footer */"
".footer{text-align:center;padding:20px;font-size:11px;color:#444}"
"</style>"
"</head>"
"<body>"
"<div class='header'>"
"<div>"
"<h3>&#9881; STM32 W5500</h3>"
"<div class='sub'>MQTT Configuration</div>"
"</div>"
"</div>"
"<div class='nav'>"
"<a href='/'>&#8962; Main</a>"
"<a href='/config' class='active'>&#9889; MQTT</a>"
"</div>"
"<div class='main'>"
"<div class='sect'>MQTT Broker</div>"
"<div class='field'><label>Host</label>"
"<input type='text' id='host' maxlength='63' placeholder='192.168.1.100'></div>"
"<div class='row2'>"
"<div class='field'><label>Port</label>"
"<input type='number' id='port' min='1' max='65535' placeholder='1883'></div>"
"<div class='field'><label>TelePeriod (s)</label>"
"<input type='number' id='tele' min='10' max='3600' placeholder='300'></div>"
"</div>"
"<div class='field'><label>Client</label>"
"<input type='text' id='cid' maxlength='31' placeholder='STM32_W5500'></div>"
"<div class='field'><label>User</label>"
"<input type='text' id='user' maxlength='31' placeholder='(kosong jika tidak perlu)'></div>"
"<div class='field'><label>Password</label>"
"<input type='password' id='pass' maxlength='31'></div>"
"<div class='sect'>MQTT Topic</div>"
"<div class='field'><label>Topic</label>"
"<input type='text' id='topic' maxlength='31' placeholder='stm32'"
" oninput='updatePreview()'></div>"
"<div class='field'><label>Full Topic</label>"
"<input type='text' id='ftopic' maxlength='63' placeholder='%prefix%/%topic%/'"
" oninput='updatePreview()'></div>"
"<div class='preview' id='prev'>"
"<div class='tinfo'>"
"Subscribe: <b id='p_cmnd'>cmnd/stm32/</b><br>"
"Publish stat: <b id='p_stat'>stat/stm32/</b><br>"
"Publish tele: <b id='p_tele'>tele/stm32/</b>"
"</div>"
"</div>"
"<div class='sect'>Status</div>"
"<div class='toggle'>"
"<input type='checkbox' id='en'>"
"<label for='en'>MQTT Enabled</label>"
"</div>"
"<div class='btns'>"
"<button class='btn btn-save' onclick='save()'>&#128190; Save</button>"
"<button class='btn btn-reset' onclick='resetDef()'>&#8635; Default</button>"
"</div>"
"<div id='msg'></div>"
"</div>"
"<div class='footer'>STM32F103 + W5500 &bull; Tasmota Style</div>"
"<script>"
"function buildTopic(prefix,topic,ftopic){"
"return ftopic.replace('%prefix%',prefix).replace('%topic%',topic);}"
"function updatePreview(){"
"let t=document.getElementById('topic').value||'stm32';"
"let ft=document.getElementById('ftopic').value||'%prefix%/%topic%/';"
"document.getElementById('p_cmnd').innerText=buildTopic('cmnd',t,ft);"
"document.getElementById('p_stat').innerText=buildTopic('stat',t,ft);"
"document.getElementById('p_tele').innerText=buildTopic('tele',t,ft);}"
"async function load(){"
"try{"
"let r=await fetch('/config/data');"
"if(!r.ok)return;"
"let d=await r.json();"
"document.getElementById('host').value=d.host||'';"
"document.getElementById('port').value=d.port||1883;"
"document.getElementById('cid').value=d.client_id||'';"
"document.getElementById('user').value=d.user||'';"
"document.getElementById('pass').value=d.password||'';"
"document.getElementById('topic').value=d.topic||'';"
"document.getElementById('ftopic').value=d.full_topic||'%prefix%/%topic%/';"
"document.getElementById('tele').value=d.tele_period||300;"
"document.getElementById('en').checked=d.enabled==1;"
"updatePreview();"
"}catch(e){}}"
"function resetDef(){"
"document.getElementById('host').value='192.168.1.100';"
"document.getElementById('port').value=1883;"
"document.getElementById('cid').value='STM32_W5500';"
"document.getElementById('user').value='';"
"document.getElementById('pass').value='';"
"document.getElementById('topic').value='stm32';"
"document.getElementById('ftopic').value='%prefix%/%topic%/';"
"document.getElementById('tele').value=300;"
"document.getElementById('en').checked=true;"
"updatePreview();}"
"async function save(){"
"let m=document.getElementById('msg');"
"m.style.display='block';m.className='msg-info';"
"m.innerText='Menyimpan...';"
"let fd=new URLSearchParams();"
"fd.append('host',document.getElementById('host').value);"
"fd.append('port',document.getElementById('port').value);"
"fd.append('client_id',document.getElementById('cid').value);"
"fd.append('user',document.getElementById('user').value);"
"fd.append('password',document.getElementById('pass').value);"
"fd.append('topic',document.getElementById('topic').value);"
"fd.append('full_topic',document.getElementById('ftopic').value);"
"fd.append('tele_period',document.getElementById('tele').value);"
"fd.append('enabled',document.getElementById('en').checked?'1':'0');"
"try{"
"let r=await fetch('/config',{method:'POST',body:fd.toString(),"
"headers:{'Content-Type':'application/x-www-form-urlencoded'}});"
"let t=await r.text();"
"if(r.ok){m.className='msg-ok';m.innerText='\u2713 '+t;}"
"else{m.className='msg-err';m.innerText='Gagal: '+t;}"
"}catch(e){m.className='msg-err';m.innerText='Error: '+e;}}"
"document.addEventListener('DOMContentLoaded',load);"
"</script>"
"</body></html>";

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
// HELPER: Ambil field dari body URL-encoded
// =============================================================================
static void parse_field(const char *body, const char *key,
                         char *out, size_t out_len)
{
    char search[72];
    snprintf(search, sizeof(search), "%s=", key);

    const char *p = strstr(body, search);
    if (!p) { out[0] = '\0'; return; }

    p += strlen(search);
    const char *end = strchr(p, '&');
    size_t len = end ? (size_t)(end - p) : strlen(p);
    if (len >= out_len) len = out_len - 1;

    static char tmp[200];
    if (len >= sizeof(tmp)) len = sizeof(tmp) - 1;
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
// HELPER: Kirim JSON config + topic preview
// =============================================================================
static void send_config_json(void)
{
    MqttConfig_t cfg;
    MqttConfig_Load(&cfg);

    // Build preview topic
    char t_cmnd[96], t_stat[96], t_tele[96];
    MqttConfig_BuildTopic(&cfg, "cmnd", t_cmnd, sizeof(t_cmnd));
    MqttConfig_BuildTopic(&cfg, "stat", t_stat, sizeof(t_stat));
    MqttConfig_BuildTopic(&cfg, "tele", t_tele, sizeof(t_tele));

    char json[512];
    snprintf(json, sizeof(json),
        "{"
        "\"host\":\"%s\","
        "\"port\":%d,"
        "\"client_id\":\"%s\","
        "\"user\":\"%s\","
        "\"password\":\"%s\","
        "\"topic\":\"%s\","
        "\"full_topic\":\"%s\","
        "\"tele_period\":%d,"
        "\"enabled\":%d,"
        "\"t_cmnd\":\"%s\","
        "\"t_stat\":\"%s\","
        "\"t_tele\":\"%s\""
        "}",
        cfg.host, cfg.port, cfg.client_id,
        cfg.user, cfg.password,
        cfg.topic, cfg.full_topic,
        cfg.tele_period, cfg.enabled,
        t_cmnd, t_stat, t_tele);

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

uint32_t g_html_config_size = 0;
// =============================================================================
// HELPER: Stream HTML dari ROM ke TCP 256 byte per chunk
// =============================================================================
static void send_html_page(void)
{
    uint16_t total = (uint16_t)strlen(HTML_CONFIG_PAGE);

    char hdr[128];
    snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n\r\n",
        total);
    W5500_SendTCP((uint8_t *)hdr, (uint16_t)strlen(hdr));

    const char *ptr       = HTML_CONFIG_PAGE;

    g_html_config_size = sizeof(HTML_CONFIG_PAGE);

    uint16_t    remaining = total;
    while (remaining > 0) {
        uint16_t chunk = (remaining > 256) ? 256 : remaining;
        W5500_SendTCP((uint8_t *)ptr, chunk);
        ptr       += chunk;
        remaining -= chunk;
    }
}

// =============================================================================
// FUNGSI PUBLIK
// =============================================================================

void WebConfig_Init(void)
{
    // Tidak perlu aksi khusus — config dibaca on-demand
    (void)0;
}

void WebConfig_Handle(uint8_t *rx_buf, uint16_t rx_len)
{
    (void)rx_len;
    char *req = (char *)rx_buf;

    // =========================================================
    // GET /config/data → JSON (dipanggil JavaScript)
    // =========================================================
    if (strstr(req, "GET /config/data") != NULL) {
        send_config_json();
        return;
    }

    // =========================================================
    // GET /config → kirim halaman HTML
    // =========================================================
    if (strstr(req, "GET /config") != NULL) {
        send_html_page();
        return;
    }

    // =========================================================
    // POST /config → parse, simpan ke SPI Flash
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

        parse_field(body, "host",        cfg.host,       sizeof(cfg.host));
        parse_field(body, "client_id",   cfg.client_id,  sizeof(cfg.client_id));
        parse_field(body, "user",        cfg.user,       sizeof(cfg.user));
        parse_field(body, "password",    cfg.password,   sizeof(cfg.password));
        parse_field(body, "topic",       cfg.topic,      sizeof(cfg.topic));
        parse_field(body, "full_topic",  cfg.full_topic, sizeof(cfg.full_topic));

        char tmp[8] = {0};

        parse_field(body, "port", tmp, sizeof(tmp));
        cfg.port = (uint16_t)atoi(tmp);
        if (cfg.port == 0) cfg.port = 1883;

        parse_field(body, "tele_period", tmp, sizeof(tmp));
        cfg.tele_period = (uint16_t)atoi(tmp);
        if (cfg.tele_period < 10) cfg.tele_period = 300;

        parse_field(body, "enabled", tmp, sizeof(tmp));
        cfg.enabled = (tmp[0] == '1') ? 1 : 0;

        // Full topic default jika kosong
        if (cfg.full_topic[0] == '\0') {
            strncpy(cfg.full_topic, MQTT_DEFAULT_FULL_TOPIC,
                    sizeof(cfg.full_topic) - 1);
        }

        cfg.magic = MQTT_CONFIG_MAGIC;

        int err = MqttConfig_Save(&cfg);
        if (err == 0) {
            g_config_updated = 1;
            send_response(200, "OK", "Config tersimpan!");
        } else {
            send_response(500, "Error", "Gagal simpan ke flash");
        }
        return;
    }
}
