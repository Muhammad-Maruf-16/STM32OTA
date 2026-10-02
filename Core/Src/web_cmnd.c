#include "web_cmnd.h"
#include "can_bus.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// =============================================================================
// FORWARD DECLARATION
// =============================================================================
extern void W5500_SendTCP(const uint8_t *data, uint16_t len);
extern CAN_HandleTypeDef hcan;

// =============================================================================
// HTML — CAN Command GUI
// Sama style dark theme seperti /config (Tasmota style)
// =============================================================================
static const char HTML_CMND_PAGE[] =
"<!DOCTYPE html>"
"<html>"
"<head>"
"<meta charset='UTF-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>STM32 - CAN Command</title>"
"<style>"
"*{box-sizing:border-box}"
"body{font-family:'Segoe UI',Arial,sans-serif;background:#1e1e1e;color:#ddd;"
"margin:0;padding:0;font-size:14px}"
".header{background:#1fa3ec;padding:10px 16px;display:flex;"
"align-items:center;justify-content:space-between}"
".header h3{margin:0;color:#fff;font-size:16px;font-weight:600}"
".header .sub{color:#cde;font-size:11px}"
".nav{background:#282828;padding:0 12px;display:flex;gap:2px;flex-wrap:wrap}"
".nav a{color:#aaa;text-decoration:none;padding:8px 12px;font-size:12px;"
"border-bottom:2px solid transparent;display:block}"
".nav a.active{color:#1fa3ec;border-bottom-color:#1fa3ec}"
".nav a:hover{color:#fff}"
".main{max-width:480px;margin:0 auto;padding:16px}"
".sect{font-size:11px;font-weight:700;color:#1fa3ec;letter-spacing:1px;"
"text-transform:uppercase;margin:18px 0 8px;border-bottom:1px solid #333;padding-bottom:4px}"
".field{margin-bottom:10px}"
".field label{display:block;font-size:12px;color:#999;margin-bottom:3px}"
".field input[type=text]{"
"width:100%;padding:6px 8px;background:#2d2d2d;border:1px solid #444;"
"border-radius:3px;color:#ddd;font-size:13px;outline:none;font-family:monospace}"
".field input:focus{border-color:#1fa3ec;background:#333}"
".hint{font-size:11px;color:#555;margin-top:3px}"
".preview-box{background:#1a1a1a;border:1px solid #333;border-radius:3px;"
"padding:8px 10px;font-size:12px;font-family:monospace;color:#888;"
"margin-top:6px;min-height:28px;word-break:break-all}"
".preview-box span.ok{color:#4caf50}"
".preview-box span.err{color:#f44336}"
".preview-box span.dim{color:#555}"
".btn{width:100%;padding:10px;border:none;border-radius:3px;font-size:13px;"
"cursor:pointer;font-weight:600;background:#1fa3ec;color:#fff;margin-top:4px}"
".btn:hover{background:#1b8fd4}"
".btn:active{background:#1780c0}"
"#msg{margin-top:10px;padding:8px 10px;border-radius:3px;font-size:12px;"
"display:none;text-align:center}"
".msg-ok{background:#1e3d1e;color:#4caf50;border:1px solid #2e5c2e}"
".msg-err{background:#3d1e1e;color:#f44336;border:1px solid #5c2e2e}"
".msg-info{background:#1e2d3d;color:#2196f3;border:1px solid #2e3d5c}"
".log-box{background:#1a1a1a;border:1px solid #333;border-radius:3px;"
"padding:8px 10px;font-size:11px;font-family:monospace;color:#888;"
"max-height:160px;overflow-y:auto;margin-top:4px}"
".log-box .entry{padding:2px 0;border-bottom:1px solid #222}"
".log-box .entry:last-child{border-bottom:none}"
".log-box .t{color:#555;margin-right:6px}"
".log-box .ok{color:#4caf50}"
".log-box .err{color:#f44336}"
".footer{text-align:center;padding:20px;font-size:11px;color:#444}"
"</style>"
"</head>"
"<body>"
"<div class='header'>"
"<div>"
"<h3>&#9881; STM32 W5500</h3>"
"<div class='sub'>CAN Command</div>"
"</div>"
"</div>"
"<div class='nav'>"
"<a href='/'>&#8962; Main</a>"
"<a href='/config'>&#9889; MQTT</a>"
"<a href='/cmnd' class='active'>&#128268; CAN</a>"
"</div>"
"<div class='main'>"
"<div class='sect'>CAN Frame</div>"
"<div class='field'>"
"<label>CAN ID (hex)</label>"
"<input type='text' id='can_id' maxlength='8' placeholder='201' oninput='updatePreview()'>"
"<div class='hint'>Contoh: 201 &nbsp;|&nbsp; 7FF &nbsp;|&nbsp; 18DAF101 (extended)</div>"
"</div>"
"<div class='field'>"
"<label>Data / MSG (hex, maks 8 byte = 16 karakter)</label>"
"<input type='text' id='can_msg' maxlength='16' placeholder='0100000000000000' oninput='updatePreview()'>"
"<div class='hint'>Contoh: 01 &nbsp;&rarr;&nbsp; 01 &nbsp;|&nbsp; Relay ON: 01000000</div>"
"</div>"
"<div class='sect'>Preview Frame</div>"
"<div class='preview-box' id='prev'><span class='dim'>Isi ID dan MSG di atas</span></div>"
"<button class='btn' onclick='send()'>&#9654; Kirim CAN</button>"
"<div id='msg'></div>"
"<div class='sect'>Log Pengiriman</div>"
"<div class='log-box' id='log'>"
"<div class='entry'><span class='t'>--:--:--</span><span class='dim'>Belum ada pengiriman</span></div>"
"</div>"
"</div>"
"<div class='footer'>STM32F103 + W5500 &bull; Tasmota Style</div>"
"<script>"
/* ---- Helper: hex validation ---- */
"function isHex(s){return /^[0-9a-fA-F]*$/.test(s);}"
"function hexBytes(s){"
"s=s.replace(/\\s/g,'');"
"if(s.length%2!==0)s='0'+s;"  /* pad ganjil */
"return s.toUpperCase();}"
"function countBytes(s){s=s.replace(/\\s/g,'');return Math.ceil(s.length/2);}"
/* ---- Preview update ---- */
"function updatePreview(){"
"let id=document.getElementById('can_id').value.trim();"
"let msg=document.getElementById('can_msg').value.trim();"
"let p=document.getElementById('prev');"
"if(!id&&!msg){p.innerHTML=\"<span class='dim'>Isi ID dan MSG di atas</span>\";return;}"
"if(!isHex(id)||!isHex(msg)){"
"p.innerHTML=\"<span class='err'>&#9888; Karakter tidak valid — gunakan 0-9 dan A-F saja</span>\";return;}"
"let dlc=countBytes(msg);"
"if(dlc>8){"
"p.innerHTML=\"<span class='err'>&#9888; Data melebihi 8 byte (DLC max 8)</span>\";return;}"
"let idU=id.toUpperCase().padStart(3,'0');"
"let msgU=hexBytes(msg);"
"let pairs=[];"
"for(let i=0;i<msgU.length;i+=2)pairs.push(msgU.substr(i,2));"
"p.innerHTML=\"<span class='ok'>ID: 0x\"+idU+\"&nbsp;&nbsp;DLC: \"+dlc+\"&nbsp;&nbsp;DATA: \"+pairs.join(' ')+\"</span>\";}"
/* ---- Timestamp ---- */
"function ts(){"
"let d=new Date();return d.getHours().toString().padStart(2,'0')+':'"
"+d.getMinutes().toString().padStart(2,'0')+':'"
"+d.getSeconds().toString().padStart(2,'0');}"
/* ---- Log append ---- */
"function logAdd(text,ok){"
"let box=document.getElementById('log');"
"let first=box.querySelector('.entry');"
"if(first&&first.innerText.includes('Belum'))first.remove();"
"let div=document.createElement('div');div.className='entry';"
"div.innerHTML=\"<span class='t'>\"+ts()+\"</span><span class='\""
"+(ok?'ok':'err')+\"'>\"+text+\"</span>\";"
"box.insertBefore(div,box.firstChild);"
"while(box.children.length>30)box.removeChild(box.lastChild);}"
/* ---- Send ---- */
"async function send(){"
"let id=document.getElementById('can_id').value.trim();"
"let msg=document.getElementById('can_msg').value.trim();"
"let m=document.getElementById('msg');"
"if(!id||!msg){m.style.display='block';m.className='msg-err';"
"m.innerText='ID dan MSG wajib diisi';return;}"
"if(!isHex(id)||!isHex(msg)){m.style.display='block';m.className='msg-err';"
"m.innerText='Karakter tidak valid';return;}"
"if(countBytes(msg)>8){m.style.display='block';m.className='msg-err';"
"m.innerText='Data melebihi 8 byte';return;}"
"m.style.display='block';m.className='msg-info';m.innerText='Mengirim...';"
"let fd=new URLSearchParams();"
"fd.append('id',id);"
"fd.append('msg',msg);"
"try{"
"let r=await fetch('/cmnd',{method:'POST',body:fd.toString(),"
"headers:{'Content-Type':'application/x-www-form-urlencoded'}});"
"let t=await r.text();"
"if(r.ok){m.className='msg-ok';m.innerText='\u2713 '+t;"
"logAdd('TX ID=0x'+id.toUpperCase()+' MSG='+msg.toUpperCase()+' \u2192 '+t,true);}"
"else{m.className='msg-err';m.innerText='Gagal: '+t;"
"logAdd('ERR: '+t,false);}"
"}catch(e){m.className='msg-err';m.innerText='Error: '+e;"
"logAdd('ERR: '+e,false);}}"
/* ---- Init ---- */
"document.addEventListener('DOMContentLoaded',updatePreview);"
"</script>"
"</body></html>";

// =============================================================================
// HELPER: URL decode (sama persis dengan web_config.c)
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
    char search[32];
    snprintf(search, sizeof(search), "%s=", key);

    const char *p = strstr(body, search);
    if (!p) { out[0] = '\0'; return; }

    p += strlen(search);
    const char *end = strchr(p, '&');
    size_t len = end ? (size_t)(end - p) : strlen(p);
    if (len >= out_len) len = out_len - 1;

    static char tmp[64];
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
// HELPER: Stream HTML ke TCP 256 byte per chunk
// =============================================================================
static void send_html_page(void)
{
    uint16_t total = (uint16_t)strlen(HTML_CMND_PAGE);

    char hdr[128];
    snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n\r\n",
        total);
    W5500_SendTCP((uint8_t *)hdr, (uint16_t)strlen(hdr));

    const char *ptr       = HTML_CMND_PAGE;
    uint16_t    remaining = total;
    while (remaining > 0) {
        uint16_t chunk = (remaining > 256) ? 256 : remaining;
        W5500_SendTCP((uint8_t *)ptr, chunk);
        ptr       += chunk;
        remaining -= chunk;
    }
}

// =============================================================================
// HELPER: Hex string → byte array
// Contoh: "0102FF" → {0x01, 0x02, 0xFF}, return DLC (jumlah byte)
// =============================================================================
static uint8_t hex_str_to_bytes(const char *str, uint8_t *out, uint8_t max_bytes)
{
    /* Bersihkan spasi, ambil maks 16 char */
    char clean[17] = {0};
    uint8_t ci = 0;
    for (uint8_t i = 0; str[i] && ci < 16; i++) {
        char c = str[i];
        if (c == ' ' || c == '\t') continue;
        clean[ci++] = c;
    }

    /* Jika ganjil: ABAIKAN karakter TERAKHIR (bukan pad depan)
       Misal "1233218" → "123321" → 12 33 21
       Konsisten: karakter yang belum punya pasangan dibuang */
    if (ci % 2 != 0) {
        ci--;  /* buang karakter terakhir */
    }

    uint8_t dlc = 0;
    for (uint8_t i = 0; i < ci && dlc < max_bytes; i += 2) {
        #define HEX_NIBBLE(c) (((c)|0x20) > '9' ? ((c)|0x20) - 'a' + 10 : (c) - '0')
        out[dlc++] = (uint8_t)((HEX_NIBBLE(clean[i]) << 4) | HEX_NIBBLE(clean[i + 1]));
        #undef HEX_NIBBLE
    }
    return dlc;
}
// =============================================================================
// FUNGSI PUBLIK
// =============================================================================

void WebCmnd_Handle(uint8_t *rx_buf, uint16_t rx_len)
{
    (void)rx_len;
    char *req = (char *)rx_buf;

    // =========================================================
    // GET /cmnd → kirim halaman HTML
    // =========================================================
    if (strstr(req, "GET /cmnd") != NULL) {
        send_html_page();
        return;
    }

    // =========================================================
    // POST /cmnd → parse id + msg, kirim CAN
    // =========================================================
    if (strstr(req, "POST /cmnd") != NULL) {
        char *body = strstr(req, "\r\n\r\n");
        if (!body) {
            send_response(400, "Bad Request", "Body tidak ditemukan");
            return;
        }
        body += 4;

        char s_id[12]  = {0};
        char s_msg[20] = {0};

        parse_field(body, "id",  s_id,  sizeof(s_id));
        parse_field(body, "msg", s_msg, sizeof(s_msg));

        if (s_id[0] == '\0' || s_msg[0] == '\0') {
            send_response(400, "Bad Request", "Parameter id / msg tidak ada");
            return;
        }

        /* Parse CAN ID */
        uint32_t can_id = strtoul(s_id, NULL, 16);

        /* Parse MSG bytes */
        uint8_t data[8] = {0};
        uint8_t dlc = hex_str_to_bytes(s_msg, data, 8);

        if (dlc == 0) {
            send_response(400, "Bad Request", "MSG tidak valid atau kosong");
            return;
        }

        /* Kirim via CAN */
        CAN_TxHeaderTypeDef tx_hdr;
        memset(&tx_hdr, 0, sizeof(tx_hdr));

        /* Tentukan Standard / Extended berdasarkan nilai ID */
        if (can_id <= 0x7FF) {
            tx_hdr.IDE   = CAN_ID_STD;
            tx_hdr.StdId = can_id;
        } else {
            tx_hdr.IDE   = CAN_ID_EXT;
            tx_hdr.ExtId = can_id;
        }
        tx_hdr.RTR = CAN_RTR_DATA;
        tx_hdr.DLC = dlc;

        uint32_t tx_mailbox;
        HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan, &tx_hdr, data, &tx_mailbox);

        if (status == HAL_OK) {
            char resp[64];
            snprintf(resp, sizeof(resp), "OK ID=0x%lX DLC=%u", can_id, dlc);
            send_response(200, "OK", resp);
        } else {
            send_response(500, "Error", "HAL_CAN_AddTxMessage gagal");
        }
        return;
    }
}
