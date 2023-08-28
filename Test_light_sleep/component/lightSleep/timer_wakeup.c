/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include "esp_check.h"
#include "esp_sleep.h"


static const char *TAG = "timer_wakeup";

esp_err_t register_timer_wakeup(int timerWakeupTimeUs)
{
    ESP_RETURN_ON_ERROR(esp_sleep_enable_timer_wakeup(timerWakeupTimeUs), TAG, "Configure timer as wakeup source failed");
    ESP_LOGI(TAG, "timer wakeup source is ready");
    return ESP_OK;
}
