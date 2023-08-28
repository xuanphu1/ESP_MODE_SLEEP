/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include "esp_err.h"


#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void wait_gpio_inactive(void);

esp_err_t register_gpio_wakeup(void);

esp_err_t register_timer_wakeup(int);

esp_err_t register_uart_wakeup(void);

esp_err_t register_touch_wakeup(void);

void light_sleep(void);

#ifdef __cplusplus
}
#endif
