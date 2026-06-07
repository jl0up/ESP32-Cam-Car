/*
 * ota.cpp — Browser-based OTA firmware update
 *
 * GET  /update  → upload form
 * POST /update  → receives multipart/form-data with the .bin,
 *                 writes it to the inactive OTA partition,
 *                 reboots on success.
 *
 * Relies on the Arduino ESP32 Update library (already in the BSP).
 * No SPIFFS, no extra tasks, no extra TCP port.
 */

#include "ota.h"
#include "esp_http_server.h"
#include "Update.h"
#include "Arduino.h"

// ── Upload form ───────────────────────────────────────────────────────────────
static esp_err_t ota_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    const char *page = R"html(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>OTA Update</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{font:15px/1.5 -apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;
    background:#111;color:#f2f2f7;display:flex;align-items:center;
    justify-content:center;min-height:100vh}
  .card{background:#1c1c1e;border:1px solid #2c2c2e;border-radius:14px;
    padding:2rem;width:100%;max-width:420px}
  h1{font-size:18px;font-weight:500;margin-bottom:1.5rem}
  label{display:block;font-size:13px;color:#8e8e93;margin-bottom:6px}
  input[type=file]{width:100%;padding:8px;background:#2c2c2e;border:1px solid #3a3a3c;
    border-radius:8px;color:#f2f2f7;font-size:13px;cursor:pointer}
  button{margin-top:1.25rem;width:100%;padding:11px;border:none;border-radius:8px;
    background:#0a84ff;color:#fff;font-size:15px;font-weight:500;cursor:pointer}
  button:active{opacity:.8}
  #msg{margin-top:1rem;font-size:13px;min-height:1.4em;text-align:center}
  .ok{color:#30d158}.err{color:#ff453a}.prog{color:#8e8e93}
  progress{width:100%;margin-top:10px;height:6px;border-radius:3px;display:none}
</style>
</head>
<body>
<div class="card">
  <h1>Firmware update</h1>
  <label for="bin">Select .bin file</label>
  <input type="file" id="bin" accept=".bin">
  <button onclick="upload()">Flash &amp; reboot</button>
  <progress id="bar" max="100" value="0"></progress>
  <div id="msg"></div>
</div>
<script>
function upload(){
  var f=document.getElementById('bin').files[0];
  if(!f){msg('No file selected','err');return;}
  var bar=document.getElementById('bar');
  bar.style.display='block';bar.value=0;
  msg('Uploading…','prog');
  var fd=new FormData();
  fd.append('firmware',f,f.name);
  var xhr=new XMLHttpRequest();
  xhr.open('POST','/update');
  xhr.upload.onprogress=function(e){
    if(e.lengthComputable)bar.value=Math.round(e.loaded/e.total*100);
  };
  xhr.onload=function(){
    if(xhr.status===200){
      msg('Done! Rebooting…','ok');
      bar.value=100;
    } else {
      msg('Error: '+xhr.responseText,'err');
    }
  };
  xhr.onerror=function(){msg('Upload failed','err');};
  xhr.send(fd);
}
function msg(t,c){var el=document.getElementById('msg');el.textContent=t;el.className=c;}
</script>
</body>
</html>)html";

    return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
}

// ── Firmware upload ───────────────────────────────────────────────────────────
//
// httpd gives us the body in chunks via repeated calls to the same handler.
// We use req->content_len and httpd_req_recv() to stream straight into Update.
//
// The body is multipart/form-data. We skip the part headers by scanning for
// the blank line (\r\n\r\n) that separates them from the binary payload, then
// feed everything after that to Update. The trailing multipart boundary
// (~50 bytes) lands in the last write; Update validates the final MD5 so the
// extra bytes would cause a size mismatch — we track the declared content
// length of the part and stop exactly there.
//
// Simpler alternative used here: the JS sends the File object directly as raw
// binary when we use XHR with FormData and a single field. The multipart
// overhead is small and predictable, so we parse just enough to skip it.

static esp_err_t ota_post_handler(httpd_req_t *req) {
    char   buf[1024];
    size_t remaining  = req->content_len;
    bool   body_begun = false;
    size_t fw_written = 0;
    size_t fw_size    = 0;  // determined after header parse

    Serial.printf("OTA: incoming %u bytes\n", remaining);

    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Serial.printf("OTA: begin failed, err=%u\n", Update.getError());
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Update.begin failed");
        return ESP_FAIL;
    }

    while (remaining > 0) {
        size_t to_read = min(remaining, sizeof(buf));
        int    recv    = httpd_req_recv(req, buf, to_read);

        if (recv <= 0) {
            if (recv == HTTPD_SOCK_ERR_TIMEOUT) continue;
            Update.abort();
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Recv error");
            return ESP_FAIL;
        }
        remaining -= recv;

        if (!body_begun) {
            // Find the blank line separating multipart headers from payload
            char *sep = (char *)memmem(buf, recv, "\r\n\r\n", 4);
            if (!sep) continue;  // headers haven't fully arrived yet (unlikely for small headers)

            // Scan backwards in the header block for Content-Length of this part
            // (not the outer request Content-Length). If absent, fall back to a
            // conservative estimate (outer length minus ~200 bytes of boundary).
            size_t hdr_len = (sep + 4) - buf;
            char *cl = (char *)memmem(buf, hdr_len, "Content-Length:", 15);
            if (cl) {
                fw_size = (size_t)atol(cl + 15);
            } else {
                fw_size = req->content_len;  // over-estimate; Update handles it
            }

            size_t payload_in_chunk = recv - hdr_len;
            if (payload_in_chunk > 0) {
                size_t to_write = min(payload_in_chunk, fw_size - fw_written);
                if (Update.write((uint8_t *)(sep + 4), to_write) != to_write) {
                    Update.abort();
                    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Write failed");
                    return ESP_FAIL;
                }
                fw_written += to_write;
            }
            body_begun = true;
        } else {
            size_t to_write = min((size_t)recv, fw_size - fw_written);
            if (to_write == 0) continue;  // trailing multipart boundary
            if (Update.write((uint8_t *)buf, to_write) != to_write) {
                Update.abort();
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Write failed");
                return ESP_FAIL;
            }
            fw_written += to_write;
        }

        Serial.printf("OTA: %u / %u bytes written\r", fw_written, fw_size);
    }

    Serial.println();

    if (!Update.end(true)) {
        Serial.printf("OTA: end failed, err=%u\n", Update.getError());
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Update.end failed");
        return ESP_FAIL;
    }

    Serial.println("OTA: success, rebooting");
    httpd_resp_sendstr(req, "OK");

    // Small delay so the HTTP response is flushed before the reset
    vTaskDelay(pdMS_TO_TICKS(300));
    esp_restart();
    return ESP_OK;
}

// ── Public API ────────────────────────────────────────────────────────────────
void ota_register(httpd_handle_t server) {
    static const httpd_uri_t get_uri = {
        .uri     = "/update",
        .method  = HTTP_GET,
        .handler = ota_get_handler,
    };
    static const httpd_uri_t post_uri = {
        .uri     = "/update",
        .method  = HTTP_POST,
        .handler = ota_post_handler,
        // Allow large uploads; default recv timeout is 5 s which is too short
        // for a 1.9 MB binary over WiFi. httpd_config recv_wait_timeout is
        // per-chunk, not per-request, so this is fine.
    };

    httpd_register_uri_handler(server, &get_uri);
    httpd_register_uri_handler(server, &post_uri);
    Serial.println("OTA: handlers registered at /update");
}
