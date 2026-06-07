/*
 * ESP32 Camera Surveillance Car — app_httpd.cpp
 *
 * Port 80 : httpd serves the HTML page.
 * Port 81 : raw TCP server — handles WebSocket upgrade manually,
 *           streams JPEG frames as WS binary frames, receives text commands.
 *           Each client runs on a dedicated FreeRTOS task.
 *
 * Why raw sockets for port 81:
 *   esp_http_server is single-threaded; pushing frames through
 *   httpd_queue_work saturates its internal queue and starves command
 *   handling. A raw socket task has no such contention.
 */

#include "esp_http_server.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "shared.h"
#include "ota.h"
#include "mbedtls/sha1.h"
#include "mbedtls/base64.h"

#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <fcntl.h>
#include <string.h>
#include <strings.h>
#include <WiFi.h>

// ── Motor speed ramp ──────────────────────────────────────────────────────────
volatile int currentSpeed = 0;
volatile int targetSpeed  = 0;

static void setMotorSpeedPercent(int pct) {
    int pwm = map(constrain(pct, 0, 100), 0, 100, 128, 255);
    ledcWrite(ENA_PIN, pwm);
    ledcWrite(ENB_PIN, pwm);
}

void motorRampTask(void *) {
    while (true) {
        if      (currentSpeed < targetSpeed) currentSpeed++;
        else if (currentSpeed > targetSpeed) currentSpeed--;
        setMotorSpeedPercent(currentSpeed);
        vTaskDelay(pdMS_TO_TICKS(15));
    }
}

// ── Motor / LED GPIO queue — GPIO writes run on core 0 ───────────────────────
typedef struct {
    int lf, lb, rf, rb;  // -1 = don't touch
    int led;             // -1 = no change
} gpio_cmd_t;

static QueueHandle_t gpio_queue = NULL;

void WheelAct(int nLf, int nLb, int nRf, int nRb) {
    digitalWrite(gpLf, nLf);
    digitalWrite(gpLb, nLb);
    digitalWrite(gpRf, nRf);
    digitalWrite(gpRb, nRb);
    targetSpeed = (nLf || nLb || nRf || nRb) ? 60 : 0;
}

static void gpio_task(void *) {
    gpio_cmd_t cmd;
    while (true) {
        if (xQueueReceive(gpio_queue, &cmd, portMAX_DELAY) == pdTRUE) {
            if (cmd.lf >= 0) WheelAct(cmd.lf, cmd.lb, cmd.rf, cmd.rb);
            if (cmd.led >= 0) digitalWrite(gpLed, cmd.led);
        }
    }
}

static void queue_motors(int lf, int lb, int rf, int rb) {
    gpio_cmd_t cmd = { lf, lb, rf, rb, -1 };
    xQueueSend(gpio_queue, &cmd, 0);
}

static void queue_led(int state) {
    gpio_cmd_t cmd = { -1, -1, -1, -1, state };
    xQueueSend(gpio_queue, &cmd, 0);
}

// ── Command dispatcher ────────────────────────────────────────────────────────
static void handle_command(const char *cmd) {
    if      (!strcmp(cmd, "go"))     queue_motors(HIGH, LOW,  HIGH, LOW);
    else if (!strcmp(cmd, "back"))   queue_motors(LOW,  HIGH, LOW,  HIGH);
    else if (!strcmp(cmd, "left"))   queue_motors(HIGH, LOW,  LOW,  HIGH);
    else if (!strcmp(cmd, "right"))  queue_motors(LOW,  HIGH, HIGH, LOW);
    else if (!strcmp(cmd, "stop"))   queue_motors(LOW,  LOW,  LOW,  LOW);
    else if (!strcmp(cmd, "ledon"))  queue_led(HIGH);
    else if (!strcmp(cmd, "ledoff")) queue_led(LOW);
    Serial.printf("CMD: %s\n", cmd);
}

// ── WebSocket helpers ─────────────────────────────────────────────────────────

// Send a WebSocket frame. opcode: 0x01=text, 0x02=binary, 0x08=close.
// Returns true on success; drops frame cleanly if socket is congested.
static bool ws_send(int sock, uint8_t opcode, const uint8_t *payload, size_t len) {
    // Drop frame if socket isn't ready
    fd_set wfds;
    FD_ZERO(&wfds);
    FD_SET(sock, &wfds);
    struct timeval tv = { 0, 0 };
    if (select(sock + 1, NULL, &wfds, NULL, &tv) <= 0 || !FD_ISSET(sock, &wfds)) {
        Serial.println("frame skipped");
        return true;
    }

    // Build header
    uint8_t hdr[10];
    int hlen = 0;
    hdr[hlen++] = 0x80 | opcode;
    if (len <= 125) {
        hdr[hlen++] = (uint8_t)len;
    } else if (len <= 65535) {
        hdr[hlen++] = 126;
        hdr[hlen++] = (len >> 8) & 0xFF;
        hdr[hlen++] =  len       & 0xFF;
    } else {
        hdr[hlen++] = 127;
        for (int i = 7; i >= 0; i--)
            hdr[hlen++] = (len >> (8 * i)) & 0xFF;
    }

    if (send(sock, hdr, hlen, MSG_NOSIGNAL) != hlen) {
        Serial.printf("ws_send: header failed errno=%d\n", errno);
        return false;
    }

    // Send payload in chunks
    for (size_t sent = 0; sent < len; ) {
        size_t chunk = min(len - sent, (size_t)4096);
        int r = send(sock, payload + sent, chunk, MSG_NOSIGNAL);
        if (r <= 0) {
            Serial.printf("ws_send: payload failed errno=%d\n", errno);
            return false;
        }
        sent += r;
    }
    return true;
}

// Receive one WebSocket frame (browser frames are always masked).
// Returns payload length or -1 on error. Caller must free *buf_out.
static int ws_recv(int sock, uint8_t *opcode_out, uint8_t **buf_out) {
    uint8_t hdr[2];
    if (recv(sock, hdr, 2, MSG_WAITALL) != 2) return -1;

    uint8_t opcode = hdr[0] & 0x0F;
    bool    masked  = (hdr[1] & 0x80) != 0;
    size_t  plen    = hdr[1] & 0x7F;

    if (plen == 126) {
        uint8_t ext[2];
        if (recv(sock, ext, 2, MSG_WAITALL) != 2) return -1;
        plen = ((size_t)ext[0] << 8) | ext[1];
    } else if (plen == 127) {
        uint8_t ext[8];
        if (recv(sock, ext, 8, MSG_WAITALL) != 8) return -1;
        plen = 0;
        for (int i = 0; i < 8; i++) plen = (plen << 8) | ext[i];
    }

    uint8_t mask[4] = {};
    if (masked && recv(sock, mask, 4, MSG_WAITALL) != 4) return -1;

    uint8_t *buf = NULL;
    if (plen > 0) {
        buf = (uint8_t *)malloc(plen + 1);
        if (!buf) return -1;
        if ((size_t)recv(sock, buf, plen, MSG_WAITALL) != plen) { free(buf); return -1; }
        if (masked)
            for (size_t i = 0; i < plen; i++) buf[i] ^= mask[i % 4];
        buf[plen] = 0;
    }

    *opcode_out = opcode;
    *buf_out    = buf;
    return (int)plen;
}

// Perform the WebSocket HTTP upgrade handshake.
static bool ws_handshake(int sock) {
    char req[1024] = {};
    int  total = 0;

    while (total < (int)sizeof(req) - 1) {
        int n = recv(sock, req + total, sizeof(req) - 1 - total, 0);
        if (n <= 0) return false;
        total += n;
        req[total] = 0;
        if (strstr(req, "\r\n\r\n")) break;
    }

    char *ks = strcasestr(req, "Sec-WebSocket-Key:");
    if (!ks) return false;
    ks += 18;
    while (*ks == ' ') ks++;
    char *ke = strstr(ks, "\r\n");
    if (!ke || (ke - ks) >= 64) return false;

    char ws_key[64];
    memcpy(ws_key, ks, ke - ks);
    ws_key[ke - ks] = 0;

    char combined[128];
    snprintf(combined, sizeof(combined), "%s258EAFA5-E914-47DA-95CA-C5AB0DC85B11", ws_key);

    uint8_t sha1_out[20];
    mbedtls_sha1((const unsigned char *)combined, strlen(combined), sha1_out);

    unsigned char accept[64];
    size_t accept_len = 0;
    mbedtls_base64_encode(accept, sizeof(accept), &accept_len, sha1_out, sizeof(sha1_out));
    accept[accept_len] = 0;

    char resp[256];
    snprintf(resp, sizeof(resp),
             "HTTP/1.1 101 Switching Protocols\r\n"
             "Upgrade: websocket\r\n"
             "Connection: Upgrade\r\n"
             "Sec-WebSocket-Accept: %s\r\n\r\n",
             accept);

    return send(sock, resp, strlen(resp), MSG_NOSIGNAL) > 0;
}

// ── Per-client stream + control task ─────────────────────────────────────────
static uint32_t last_fail = 0;

static void client_task(void *arg) {
    int sock = ((int)(intptr_t)arg);

    Serial.printf("Client connected, sock=%d\n", sock);

    struct timeval tv = { 10, 0 };
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    while (true) {
        // Non-blocking check for incoming command
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(sock, &rfds);
        struct timeval zero = { 0, 0 };
        if (select(sock + 1, &rfds, NULL, NULL, &zero) > 0 && FD_ISSET(sock, &rfds)) {
            uint8_t opcode = 0, *payload = NULL;
            int rlen = ws_recv(sock, &opcode, &payload);
            if (rlen > 0) {
                if (opcode == 0x01 && payload) handle_command((const char *)payload);
                else if (opcode == 0x08)       { free(payload); break; }
            } else if (rlen < 0) {
                Serial.printf("ws_recv failed errno=%d\n", errno);
                free(payload);
                break;
            }
            free(payload);
        }

        // Capture frame
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb || fb->len == 0) {
            if (fb) esp_camera_fb_return(fb);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        uint8_t *jpg   = NULL;
        size_t   jlen  = 0;
        bool     alloc = false;

        if (fb->format == PIXFORMAT_JPEG) {
            jpg = fb->buf;
            jlen = fb->len;
        } else {
            alloc = frame2jpg(fb, 80, &jpg, &jlen);
        }

        if (jpg && jlen > 0) {
            bool ok = ws_send(sock, 0x02, jpg, jlen);
            if (ok) {
                diag.framesSent++;
                diag.lastFrameMs = millis();
                strcpy(diag.lastEvent, "Frame sent");
            } else {
                diag.wsSendErrors++;
                strcpy(diag.lastEvent, "Frame send failed");
                last_fail = millis();
            }
            esp_camera_fb_return(fb);
            if (!ok) break;
        } else {
            esp_camera_fb_return(fb);
        }

        if (alloc && jpg) free(jpg);

        vTaskDelay(pdMS_TO_TICKS(millis() - last_fail < 2000 ? 150 : 60));
    }

    Serial.printf("Client disconnected, sock=%d\n", sock);
    queue_motors(LOW, LOW, LOW, LOW);
    close(sock);
    vTaskDelete(NULL);
}

// ── Raw TCP / WebSocket server (port 81) ──────────────────────────────────────
static void tcp_server_task(void *) {
    int srv = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(81);
    bind(srv, (struct sockaddr *)&addr, sizeof(addr));
    listen(srv, 1);
    Serial.println("WS server listening on port 81");

    while (true) {
        struct sockaddr_in ca;
        socklen_t clen = sizeof(ca);
        int csock = accept(srv, (struct sockaddr *)&ca, &clen);
        if (csock < 0) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }

        int flag = 1;
        setsockopt(csock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

        Serial.printf("New TCP connection, sock=%d\n", csock);
        if (!ws_handshake(csock)) {
            Serial.println("WS handshake failed");
            close(csock);
            continue;
        }
        Serial.println("WS handshake OK");

        // Pass socket as a pointer-sized value to avoid a malloc
        xTaskCreatePinnedToCore(client_task, "ws_client", 8192,
                                (void *)(intptr_t)csock, 3, NULL, 1);
    }
}

// ── HTML page handler (port 80) ───────────────────────────────────────────────
//
// Layout (single viewport, no scroll):
//
//   ┌─────────────────────────────────────────┐
//   │  [●] CAM   fps · rssi · heap  [☀ light] │  ← header bar
//   ├──────────────────────────┬──────────────┤
//   │                          │   ↑ Forward  │
//   │      camera canvas       │ ← Stop →     │
//   │    (fills remaining h)   │   ↓ Backward │
//   ├──────────────────────────┴──────────────┤
//   │  uptime · frames · heap · event …       │  ← status strip
//   └─────────────────────────────────────────┘
//
// On narrow screens the dpad moves below the canvas.

static esp_err_t index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");

    // The page is assembled as a raw string. Keeping it readable here matters
    // more than saving a few bytes — the ESP32 sends it once per connection.
    String page = R"rawhtml(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>ESP32 CAM Car</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  :root{
    --bg:#111;--surface:#1c1c1e;--border:#2c2c2e;
    --text:#f2f2f7;--muted:#8e8e93;
    --accent:#0a84ff;--danger:#ff453a;--warn:#ffd60a;
  }
  html,body{height:100%;overflow:hidden;background:var(--bg);color:var(--text);
    font:14px/1 -apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif}

  /* ── layout ── */
  #app{display:flex;flex-direction:column;height:100vh}
  #header{display:flex;align-items:center;gap:10px;padding:6px 16px;
    background:var(--surface);border-bottom:1px solid var(--border);flex-shrink:0}

  #main{
    display:flex;flex:1;min-height:0;
    align-items:center;justify-content:center;
    gap:40px;padding:20px 32px
  }

  /* camera: height-driven, portrait aspect (3:4 after rotation) */
  #cam-wrap{
    height:100%;
    aspect-ratio:3/4;
    flex-shrink:0;
    background:#000;
    border-radius:10px;
    overflow:hidden;
    display:flex;align-items:center;justify-content:center;
    position:relative
  }
  #cv{
    width:100%;height:100%;
    display:block;
    object-fit:contain
  }

  /* dpad panel */
  #dpad{
    display:flex;flex-direction:column;align-items:center;justify-content:center;
    gap:8px;flex-shrink:0
  }

  #status{padding:5px 16px;background:var(--surface);border-top:1px solid var(--border);
    flex-shrink:0;display:flex;flex-wrap:wrap;gap:4px 16px;font-size:12px;color:var(--muted)}
  #status span b{color:var(--text);font-weight:500}

  /* ── header items ── */
  #dot{width:8px;height:8px;border-radius:50%;background:var(--danger);flex-shrink:0}
  #dot.live{background:#30d158}
  #hinfo{flex:1;display:flex;gap:14px;font-size:12px;color:var(--muted)}
  #hinfo span b{color:var(--text);font-weight:500}

  /* ── dpad buttons — 120×120 px ── */
  .drow{display:flex;gap:8px}
  .btn{
    width:120px;height:120px;border-radius:16px;border:none;cursor:pointer;
    background:var(--border);color:var(--text);font-size:44px;
    display:flex;align-items:center;justify-content:center;
    user-select:none;-webkit-user-select:none;touch-action:none;
    transition:background .1s,transform .08s
  }
  .btn:active,.btn.pressed{background:#3a3a3c;transform:scale(.93)}
  .btn.stop{background:#3a1a1a;color:var(--danger)}
  .btn.stop:active,.btn.stop.pressed{background:#5a1f1f}

  /* ── resolution selector ── */
  #ressel{
    padding:4px 8px;border-radius:8px;border:1px solid var(--border);
    background:var(--surface);color:var(--text);font-size:12px;cursor:pointer
  }

  /* ── grab button ── */
  #grabbtn{
    margin-top:8px;padding:10px 0;width:376px;border-radius:12px;
    border:1px solid var(--border);background:var(--surface);
    color:var(--text);font-size:15px;cursor:pointer;
    transition:background .1s
  }
  #grabbtn:active{background:#3a3a3c}

  /* ── light toggle ── */
  #lightbtn{
    padding:5px 12px;border-radius:8px;border:1px solid var(--border);
    background:transparent;color:var(--muted);font-size:12px;cursor:pointer;
    transition:background .15s,color .15s;white-space:nowrap
  }
  #lightbtn.on{background:#2a2200;color:var(--warn);border-color:#554400}
</style>
</head>
<body>
<div id="app">

  <div id="header">
    <div id="dot"></div>
    <span style="font-weight:500;font-size:13px">CAM</span>
    <div id="hinfo">
      <span>FPS&nbsp;<b id="hfps">–</b></span>
      <span>RSSI&nbsp;<b id="hrssi">–</b></span>
      <span>Heap&nbsp;<b id="hheap">–</b></span>
      <button id="grabbtn" onclick="grabFrame()">&#x1F4F7; Save frame</button>
    </div>
    <select id="ressel" onchange="setRes(this.value)">
      <option value="7">QVGA 320×240</option>
      <option value="8">HVGA 480×320</option>
      <option value="9">VGA 640×480</option>
      <option value="10" selected>SVGA 800×600</option>
      <option value="11">XGA 1024×768</option>
      <option value="12">HD 1280×720</option>
      <option value="13">SXGA 1280×1024</option>
      <option value="14">UXGA 1600×1200</option>
    </select>
    <button id="lightbtn" onmousedown="toggleLight()">&#9728; Light</button>
  </div>

  <div id="main">
    <div id="cam-wrap">
      <canvas id="cv"></canvas>
    </div>

    <div id="dpad">
      <div class="drow">
        <button class="btn" id="btn-go"
          onmousedown="hold('go')" onmouseup="rel()" onmouseleave="rel()"
          ontouchstart="hold('go')" ontouchend="rel()">&#9650;</button>
      </div>
      <div class="drow">
        <button class="btn" id="btn-left"
          onmousedown="hold('left')" onmouseup="rel()" onmouseleave="rel()"
          ontouchstart="hold('left')" ontouchend="rel()">&#9664;</button>
        <button class="btn stop" id="btn-stop"
          onmousedown="send('stop')">&#9632;</button>
        <button class="btn" id="btn-right"
          onmousedown="hold('right')" onmouseup="rel()" onmouseleave="rel()"
          ontouchstart="hold('right')" ontouchend="rel()">&#9654;</button>
      </div>
      <div class="drow">
        <button class="btn" id="btn-back"
          onmousedown="hold('back')" onmouseup="rel()" onmouseleave="rel()"
          ontouchstart="hold('back')" ontouchend="rel()">&#9660;</button>
      </div>
    </div>
  </div>

  <div id="status">
    <span>Uptime&nbsp;<b id="suptime">–</b></span>
    <span>Frames&nbsp;<b id="sframes">–</b></span>
    <span>Dropped&nbsp;<b id="sdrop">–</b></span>
    <span>Min heap&nbsp;<b id="sminheap">–</b></span>
    <span>Largest blk&nbsp;<b id="slarge">–</b></span>
    <span>TX err&nbsp;<b id="serr">–</b></span>
    <span id="sevent" style="flex:1;text-align:right;overflow:hidden;text-overflow:ellipsis;white-space:nowrap"></span>
  </div>

</div>
<script>
var ws=null,lightOn=false,lastFrame=0,fps=0,fpsT=0,fpsC=0;
var cv=document.getElementById('cv'),ctx=cv.getContext('2d');
var dot=document.getElementById('dot');
var activeBtn=null;

function send(cmd){if(ws&&ws.readyState===1)ws.send(cmd);}

function hold(cmd){
  send(cmd);
  var id='btn-'+({'go':'go','back':'back','left':'left','right':'right'}[cmd]||cmd);
  if(activeBtn){activeBtn.classList.remove('pressed');}
  activeBtn=document.getElementById(id);
  if(activeBtn)activeBtn.classList.add('pressed');
}
function rel(){
  send('stop');
  if(activeBtn){activeBtn.classList.remove('pressed');activeBtn=null;}
}

function setRes(val){
  fetch('/cmd?var=framesize&val='+val)
    .then(function(r){if(!r.ok)console.warn('setRes failed');});
}

function grabFrame(){
  var ts=new Date().toISOString().replace(/[:.]/g,'-').slice(0,-1);
  cv.toBlob(function(blob){
    var a=document.createElement('a');
    a.href=URL.createObjectURL(blob);
    a.download='frame-'+ts+'.jpg';
    a.click();
    URL.revokeObjectURL(a.href);
  },'image/jpeg',0.92);
}

function toggleLight(){
  lightOn=!lightOn;
  send(lightOn?'ledon':'ledoff');
  var b=document.getElementById('lightbtn');
  b.classList.toggle('on',lightOn);
}

function connect(){
  ws=new WebSocket('ws://)rawhtml" + WiFiAddr + R"rawhtml(:81');
  ws.binaryType='arraybuffer';
  ws.onopen=function(){dot.className='live';};
  ws.onclose=function(){dot.className='';setTimeout(connect,2000);};
  ws.onerror=function(){ws.close();};
  ws.onmessage=function(e){
    if(!(e.data instanceof ArrayBuffer))return;
    fpsC++;
    var now=Date.now();
    if(now-fpsT>=1000){fps=fpsC;fpsC=0;fpsT=now;document.getElementById('hfps').textContent=fps;}
    createImageBitmap(new Blob([e.data],{type:'image/jpeg'})).then(function(b){
      // Size the canvas to the rotated image's native pixels.
      // CSS object-fit:contain on the wrapper does the rest.
      cv.width=b.height;
      cv.height=b.width;
      ctx.translate(b.height/2, b.width/2);
      ctx.rotate(-Math.PI/2);
      ctx.drawImage(b, -b.width/2, -b.height/2);
      ctx.resetTransform();
    });
  };
}

var kd={};
var keyCmd={ArrowUp:'go',ArrowDown:'back',ArrowLeft:'left',ArrowRight:'right'};
var keyPrio=['ArrowUp','ArrowDown','ArrowLeft','ArrowRight'];
document.addEventListener('keydown',function(e){
  if(e.key===' '){e.preventDefault();if(!kd[' ']){kd[' ']=true;toggleLight();}return;}
  if(!keyCmd[e.key])return;
  e.preventDefault();
  if(kd[e.key])return;
  kd[e.key]=true;
  hold(keyCmd[e.key]);
});
document.addEventListener('keyup',function(e){
  if(e.key===' '){e.preventDefault();kd[' ']=false;return;}
  if(!keyCmd[e.key])return;
  e.preventDefault();
  kd[e.key]=false;
  for(var i=0;i<keyPrio.length;i++){
    if(kd[keyPrio[i]]){hold(keyCmd[keyPrio[i]]);return;}
  }
  rel();
});

function kb(v){if(v>=1048576)return(v/1048576).toFixed(1)+'M';if(v>=1024)return(v/1024).toFixed(0)+'K';return v;}
function upfmt(s){var h=Math.floor(s/3600),m=Math.floor((s%3600)/60),sec=s%60;
  if(h)return h+'h '+m+'m';if(m)return m+'m '+sec+'s';return sec+'s';}

setInterval(function(){
  fetch('/status').then(function(r){return r.json();}).then(function(j){
    document.getElementById('hrssi').textContent=j.rssi+'dBm';
    document.getElementById('hheap').textContent=kb(j.heap);
    document.getElementById('suptime').textContent=upfmt(j.uptime);
    document.getElementById('sframes').textContent=j.frames;
    document.getElementById('sdrop').textContent=j.dropped;
    document.getElementById('sminheap').textContent=kb(j.minHeap);
    document.getElementById('slarge').textContent=kb(j.largestBlock);
    document.getElementById('serr').textContent=j.sendErr;
    document.getElementById('sevent').textContent=j.event;
  });
},1000);

connect();
</script>
</body>
</html>
)rawhtml";

    return httpd_resp_send(req, page.c_str(), HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_handler(httpd_req_t *req) {
    char json[512];
    snprintf(json, sizeof(json),
        "{\"uptime\":%u,\"heap\":%u,\"minHeap\":%u,\"largestBlock\":%u,"
        "\"frames\":%u,\"dropped\":%u,\"recvErr\":%u,\"sendErr\":%u,"
        "\"lastFrame\":%u,\"lastCmd\":%u,\"rssi\":%d,\"event\":\"%s\"}",
        diag.uptime, diag.freeHeap, diag.minFreeHeap, diag.largestBlock,
        diag.framesSent, diag.framesDropped, diag.wsRecvErrors, diag.wsSendErrors,
        diag.lastFrameMs, diag.lastCommandMs, diag.wifiRssi, diag.lastEvent);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t cmd_handler(httpd_req_t *req) {
    char buf[64];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing query");
        return ESP_FAIL;
    }

    char var[32], val[32];
    if (httpd_query_key_value(buf, "var", var, sizeof(var)) != ESP_OK ||
        httpd_query_key_value(buf, "val", val, sizeof(val)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad params");
        return ESP_FAIL;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No sensor");
        return ESP_FAIL;
    }

    int res = -1;
    if (!strcmp(var, "framesize")) {
        res = s->set_framesize(s, (framesize_t)atoi(val));
        Serial.printf("CMD: framesize=%s -> %d\n", val, res);
    }
    // extend here for contrast, brightness, etc.

    if (res != 0) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Sensor cmd failed");
        return ESP_FAIL;
    }

    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}


void startCameraServer() {
    gpio_queue = xQueueCreate(10, sizeof(gpio_cmd_t));
    if (!gpio_queue) { Serial.println("ERROR: gpio_queue creation failed"); return; }
    xTaskCreatePinnedToCore(gpio_task, "gpio_task", 2048, NULL, 8, NULL, 0);

    httpd_config_t cfg   = HTTPD_DEFAULT_CONFIG();
    cfg.server_port      = 80;
    cfg.ctrl_port        = 32780;
    cfg.max_uri_handlers = 8;
    cfg.recv_wait_timeout = 2;
    cfg.send_wait_timeout = 2;

    static const httpd_uri_t cmd_uri = {
        .uri     = "/cmd",
        .method  = HTTP_GET,
        .handler = cmd_handler,
    };

    static const httpd_uri_t index_uri  = { "/",       HTTP_GET, index_handler,  NULL };
    static const httpd_uri_t status_uri = { "/status", HTTP_GET, status_handler, NULL };

    httpd_handle_t page_httpd = NULL;
    if (httpd_start(&page_httpd, &cfg) == ESP_OK) {
        httpd_register_uri_handler(page_httpd, &index_uri);
        httpd_register_uri_handler(page_httpd, &status_uri);
        httpd_register_uri_handler(page_httpd, &cmd_uri);
        ota_register(page_httpd);
        Serial.println("HTTP server started on port 80");
    } else {
        Serial.println("HTTP server failed to start");
    }

    xTaskCreatePinnedToCore(tcp_server_task, "tcp_srv", 4096, NULL, 2, NULL, 0);
}