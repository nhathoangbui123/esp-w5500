#pragma once

#include "esp_eth_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t eth_w5500_init(esp_eth_handle_t *eth_handles_out[], uint8_t *eth_cnt_out);

esp_err_t eth_w5500_deinit(esp_eth_handle_t *eth_handles, uint8_t eth_cnt);

#ifdef __cplusplus
}
#endif
