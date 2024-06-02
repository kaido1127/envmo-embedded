
#include "mq2.h"
esp_adc_cal_characteristics_t adc1_chars;
#define DEFAULT_VREF    1100
esp_err_t mq2_init()
{
    //cấu hình độ phân giải của ADC là 12 bit cho phép ADC chuyển đổi tín hiệu analog thành các giá trị số từ 0 đến 4095.
    esp_err_t error_1 = adc1_config_width(ADC_WIDTH_BIT_12); 
    ESP_ERROR_CHECK_WITHOUT_ABORT(error_1);
    //cấu hình độ suy giảm của kênh ADC1 kênh 6 được cấu hình với độ suy giảm 11 dB chỉnh khoảng điện áp đầu vào của ADC để phù hợp với tín hiệu đầu vào.
    esp_err_t error_2 = adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11); 
    ESP_ERROR_CHECK_WITHOUT_ABORT(error_2);
    //Hàm này lấy các tham số như đơn vị ADC, độ suy giảm, độ phân giải và điện áp tham chiếu mặc định, và sau đó lưu trữ thông số hiệu chuẩn vào biến adc1_chars
    esp_err_t error_3 = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, &adc1_chars);
    ESP_ERROR_CHECK_WITHOUT_ABORT(error_3);
    if (error_1 == ESP_OK && 
        error_2 == ESP_OK &&
        error_3 == ESP_OK)
    {
        ESP_LOGI(__func__, "MQ2 initialize successful.");
        return ESP_OK;
    } else {
        ESP_LOGE(__func__, "MQ2 initialize failed.");
        return ESP_ERROR_MQ2_INIT_FAILED ;
    }
}

esp_err_t mq2_reading(uint32_t* gas)
{
    // esp_adc_cal_characteristics_t adc_chars;
        uint32_t adc_reading = 0;
        for (int i = 0; i < 50; i++)
        {
            adc_reading += adc1_get_raw(ADC1_CHANNEL_6);
            vTaskDelay(pdMS_TO_TICKS(10));
            // ESP_LOGI(__func__,"Gia tri raw= %u",adc_reading);
        }
        adc_reading = adc_reading / 50;
        // ESP_LOGI(__func__,"Tong raw = %u ", adc_reading);
        *gas = adc_reading / 10.0;
        ESP_LOGI(__func__,"Khi CO = %d", *gas);
        return ESP_OK;
}