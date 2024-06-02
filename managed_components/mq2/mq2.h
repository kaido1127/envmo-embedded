#ifndef mq2_h
#define mq2_h
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "../../../essp_idf/components/esp_adc_cal/esp_adc_cal.h"
#include "esp_err.h"
#include "adc_types_legacy.h"
#include "esp_adc_cal_legacy.h"

#define ID_MQ2  0X03
#define ESP_ERROR_MQ2_INIT_FAILED ((ID_MQ2 << 12)|(0x00))
#define ESP_ERROR_MQ2_READ_DATA_FAILED ((ID_MQ2 << 12)|(0x01))

esp_err_t mq2_init();
esp_err_t mq2_reading(uint32_t *gas);

#endif
