#pragma once

#include "esp_netif.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_netif_t *app_ethernet_connect(void);

#ifdef __cplusplus
}
#endif
