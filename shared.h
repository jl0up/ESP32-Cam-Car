/*
 * shared.h — types and extern declarations shared between translation units
 */

#pragma once

#include <Arduino.h>

// ── Diagnostics ───────────────────────────────────────────────────────────────
typedef struct {
    uint32_t uptime;
    uint32_t framesSent;
    uint32_t framesDropped;
    uint32_t wsRecvCount;
    uint32_t wsSendErrors;
    uint32_t wsRecvErrors;
    uint32_t reconnects;
    uint32_t freeHeap;
    uint32_t minFreeHeap;
    uint32_t largestBlock;
    uint32_t clientStack;
    uint32_t lastFrameMs;
    uint32_t lastCommandMs;
    int      wifiRssi;
    char     lastEvent[64];
} diag_t;

// ── Globals defined in the .ino, used in app_httpd.cpp ───────────────────────
extern int     gpLb, gpLf, gpRb, gpRf, gpLed;
extern int     ENA_PIN, ENB_PIN;
extern String  WiFiAddr;
extern diag_t  diag;
