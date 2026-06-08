/*
 * ESP32 Camera Surveillance Car — main sketch
 */

#include "esp_camera.h"
#include "shared.h"
#include "secret.h"
#include <WiFi.h>

#define CAMERA_MODEL_AI_THINKER


// ── Camera pin map (AI-Thinker) ───────────────────────────────────────────────
#define PWDN_GPIO_NUM    32
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM     0
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27
#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      21
#define Y4_GPIO_NUM      19
#define Y3_GPIO_NUM      18
#define Y2_GPIO_NUM       5
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22

#define FRAMESIZE_OK FRAMESIZE_SVGA

// ── Global definitions (declared extern in shared.h) ─────────────────────────
int gpLb  = 12;   // Left  motor backward
int gpLf  = 14;   // Left  motor forward
int gpRb  = 15;   // Right motor backward
int gpRf  = 13;   // Right motor forward
int gpLed =  4;   // Flash LED
// NOTE: GPIO 4 is shared with the camera flash AND Y2 data line on some
// AI-Thinker revisions. If you see frame corruption when toggling the LED,
// rewire the LED to another GPIO and update gpLed above.

int ENA_PIN = 3;
int ENB_PIN = 1;

String WiFiAddr = "";

diag_t diag = {};

// ── Diagnostics task ──────────────────────────────────────────────────────────
static void monitorTask(void *) {
    while (true) {
        diag.uptime       = millis() / 1000;
        diag.freeHeap     = ESP.getFreeHeap();
        diag.minFreeHeap  = ESP.getMinFreeHeap();
        diag.largestBlock = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
        diag.wifiRssi     = WiFi.RSSI();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ── Forward declarations ──────────────────────────────────────────────────────
void motorRampTask(void *);
void startCameraServer();

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.setDebugOutput(true);

    // Motor & LED pins
    pinMode(gpLb,  OUTPUT); digitalWrite(gpLb,  LOW);
    pinMode(gpLf,  OUTPUT); digitalWrite(gpLf,  LOW);
    pinMode(gpRb,  OUTPUT); digitalWrite(gpRb,  LOW);
    pinMode(gpRf,  OUTPUT); digitalWrite(gpRf,  LOW);
    pinMode(gpLed, OUTPUT); digitalWrite(gpLed, LOW);

    // PWM motor speed
    ledcAttach(ENA_PIN, 880, 8);
    ledcAttach(ENB_PIN, 880, 8);

    // Camera config
    camera_config_t config = {};
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0       = Y2_GPIO_NUM;
    config.pin_d1       = Y3_GPIO_NUM;
    config.pin_d2       = Y4_GPIO_NUM;
    config.pin_d3       = Y5_GPIO_NUM;
    config.pin_d4       = Y6_GPIO_NUM;
    config.pin_d5       = Y7_GPIO_NUM;
    config.pin_d6       = Y8_GPIO_NUM;
    config.pin_d7       = Y9_GPIO_NUM;
    config.pin_xclk     = XCLK_GPIO_NUM;
    config.pin_pclk     = PCLK_GPIO_NUM;
    config.pin_vsync    = VSYNC_GPIO_NUM;
    config.pin_href     = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;
    config.xclk_freq_hz = 10000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size   = FRAMESIZE_OK;
    config.jpeg_quality = 30;
    config.fb_count     = psramFound() ? 3 : 1;

    strcpy(diag.lastEvent, "Boot");

    if (esp_camera_init(&config) != ESP_OK) {
        Serial.println("Camera init failed");
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        Serial.println("Camera sensor not detected");
        return;
    }
    s->set_framesize(s, FRAMESIZE_OK);
    Serial.println("Camera OK");

    // WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    WiFi.setSleep(false);
    WiFiAddr = WiFi.localIP().toString();
    Serial.printf("\nWiFi connected — http://%s\n", WiFiAddr.c_str());

    startCameraServer();

    xTaskCreatePinnedToCore(motorRampTask, "motorRamp", 2048, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(monitorTask,   "monitor",   4096, NULL, 1, NULL, 1);
}

void loop() {
    // All work happens in FreeRTOS tasks — do not add code here.
}
