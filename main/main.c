#include "connect_wifi.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "sdkconfig.h"
#include "esp_idf_lib_helpers.h"
#include "ets_sys.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_event_loop.h"
#include "esp_event.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_chip_info.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_attr.h"
#include "esp_spi_flash.h"
#include "mqtt_client.h"
#include "esp_tls.h"
#include "esp_ota_ops.h"
#include <sys/param.h>
#include <src/PubSubClient.h>

#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "i2cdev/i2cdev.h"
#include "driver/spi_common.h"
#include "driver/ledc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "freertos/event_groups.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "libIP2Location\IP2Location.h"
#include "../managed_components/mq2/mq2.h"
#include "bme280.h"
#include "bmp280.h"

#define WAIT_10_TICK (TickType_t)(10 / portTICK_RATE_MS)
#define PERIOD_GET_DATA_FROM_SENSOR (TickType_t)(5000 / portTICK_RATE_MS)
#define PERIOD_SOUND (TickType_t)(100 / portTICK_RATE_MS)
#define QUEUE_SIZE 10U

const char *mqtt_broker = "broker.emqx.io";
const char *mqtt_username = "publisher";
const char *mqtt_password = "publisher123";
const int mqtt_port = 1883;
const char *topic = "topic/env_info_queue";

bmp280_t bme280_device;
bmp280_params_t bme280_params;

struct dataSensor_st
{
    float temperature;
    float pressure;
    float humidity;
    uint32_t gas;
    uint8_t location_from_IP;
    uint8_t MAC_Addr;
    float probability;
};

struct dataSensor_st dataFromSensor;
uint8_t dataFromSensor.MAC_Addr = esp_read_mac();

QueueHandle_t dataSensorSent_toMQTT;

static const char *TAG = "example";
char REQUEST[512];
char recv_buf[512];
char SUBREQUEST[150];

/**
 * @brief connect to mqtt broker
 *
 */
void mqtt_get_task(void)
{
    client.setServer(mqtt_broker, mqtt_port);
    while (!client.connected())
    {
        String client_id = "envmo_client";
        client_id += String(WiFi.macAddress());
        Serial.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());
        if (client.connect(client_id.c_str(), mqtt_username, mqtt_password))
        {
            Serial.println("Public emqx mqtt broker connected");
        }
        else
        {
            Serial.print("failed with state ");
            Serial.print(client.state());
            delay(2000);
        }
    }
    // publish and subscribe
    client.publish(topic, "Hi EMQX I'm ESP32 ^^");
    client.subscribe(topic);
}

/**
 * @brief this task get data fromsensor and use DS-Evidence algorithm to calculate the probability from 2 sensor
 * @authors daktngominh@gmail.com
 *
 */
void readDataFromSensor()
{
    for (;;)
    {
        TickType_t task_lastWakeTime;
        task_lastWakeTime = xTaskGetTickCount();
        // BME280
        bme280_readSensorData(&bme280_device, &(dataFromSensor.temperature),
                              &(dataFromSensor.pressure),
                              &(dataFromSensor.humidity));
        // MQ2
        mq2_reading(&(dataFromSensor.gas));
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        ESP_LOGI(__func__, "temp= %.2f, hum = %.2f, press = %.2f,  gas = %u ", dataFromSensor.temperature, dataFromSensor.humidity, dataFromSensor.pressure, dataFromSensor.gas);
        float DS_fire;
        float DS_noFire;
        float temperatureProbability = (dataFromSensor.temperature - 20) / 65;
        if (temperatureProbability < 0)
        {
            temperatureProbability = 0;
        }
        ESP_LOGI(__func__, "Xac Suat temperature = %.2f ", temperatureProbability);
        float humidityProbability = (-dataFromSensor.humidity / 100) + 1;
        ESP_LOGI(__func__, "Xac Suat humidity = %.2f ", humidityProbability);
        float gasProbability;
        gasProbability = (float)(dataFromSensor.gas - 300) / (10000 - 300);
        ESP_LOGI(__func__, "Nong Do gas = %u ", dataFromSensor.gas);
        ESP_LOGW(__func__, "Xac Suat gas = %.2f ", gasProbability);
        if (gasProbability < 0)
        {
            gasProbability = 0;
            ESP_LOGI(__func__, "bi gan bang 0");
        }
        ESP_LOGI(__func__, "Xac Suat gas = %.2f ", gasProbability);
        // calculate fire probability
        DS_fire = (temperatureProbability * humidityProbability) / (1 - (1 - temperatureProbability) * humidityProbability - temperatureProbability * (1 - humidityProbability));
        DS_noFire = 1 - DS_fire;
        ESP_LOGI(__func__, "DS lan 1 = %.2f", DS_fire);
        DS_fire = (DS_fire * gasProbability) / (1 - (DS_noFire * gasProbability) - DS_fire * (1 - gasProbability));
        DS_noFire = 1 - DS_fire;
        ESP_LOGI(__func__, "DS lan 2 = %.2f", DS_fire);
        dataFromSensor.probability = DS_fire;

        if (xQueueSendToBack(dataSensorSent_toMQTT, (void *)&dataFromSensor, WAIT_10_TICK * 5) != pdPASS)
        {
            ESP_LOGE(__func__, "Failed to post the data sensor to dataSensorSentToHTTP Queue.");
        }
        else
        {
            ESP_LOGI(__func__, "Succeeded to post the data sensor to dataSensorSentToHTTP Queue.");
        }
        vTaskDelayUntil(&task_lastWakeTime, PERIOD_GET_DATA_FROM_SENSOR);
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2cdev_init());
    ESP_LOGI(__func__, "Initialize BME280 sensor(I2C/Wire%d).", CONFIG_BME_I2C_PORT);

    ESP_ERROR_CHECK_WITHOUT_ABORT(bme280_init(&bme280_device, &bme280_params, BME280_ADDRESS,
                                              CONFIG_BME_I2C_PORT, CONFIG_BME_PIN_NUM_SDA, CONFIG_BME_PIN_NUM_SCL));

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    dataSensorSent_toMQTT = xQueueCreate(QUEUE_SIZE, sizeof(struct dataSensor_st));
    while (dataSensorSent_toMQTT == NULL)
    {
        ESP_LOGE(__func__, "Create dataSensorSentToHTTP Queue failed.");
        ESP_LOGI(__func__, "Retry to create dataSensorSentToHTTP Queue...");
        vTaskDelay(500 / portTICK_RATE_MS);
        dataSensorSent_toMQTT = xQueueCreate(QUEUE_SIZE, sizeof(struct dataSensor_st));
    };

    connect_wifi();
    if (wifi_connect_status)
    {

        xTaskCreate(&mqtt_get_task, "mqtt_get_task", 8192, NULL, 5, NULL);
    }
    xTaskCreate(&readDataFromSensor, "readDataFromSensor", 8192, NULL, 6, NULL);
}