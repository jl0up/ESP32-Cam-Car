#pragma once

#include "esp_http_server.h"

// Register /update GET (form) and POST (upload) handlers on an existing httpd.
void ota_register(httpd_handle_t server);
