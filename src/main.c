#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <u8g2_esp32_hal.h>
#include <ultrasonic.h>
#include <dht.h>

#define MAX_DISTANCE_CM 500
#define DHT         GPIO_NUM_14
#define TRIGGER     GPIO_NUM_12
#define ECHO        GPIO_NUM_13
#define PIN_SDA     GPIO_NUM_5
#define PIN_SCL     GPIO_NUM_4

static const char *TAG = "OLED";
u8g2_t u8g2;
QueueHandle_t bufferTemperature;
QueueHandle_t bufferDistance;

void task_ultrasonic(void *pvParamters);
void task_dht(void *pvParameters);
void task_oled(void *pvParameters);

void task_ultrasonic(void *pvParamters) {
    ultrasonic_sensor_t sensor = {
        .trigger_pin = TRIGGER,
        .echo_pin = ECHO
    };

    ultrasonic_init(&sensor);

    while (1) {
        uint32_t distance = 0;
        esp_err_t res = ultrasonic_measure_cm(&sensor, MAX_DISTANCE_CM, &distance);
        if (res != ESP_OK) {

            switch (res) {
                case ESP_ERR_ULTRASONIC_PING:
                    ESP_LOGE(TAG, "ERROR - Cannot ping (device is in invalid state)");
                    break;
                case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                    ESP_LOGE(TAG, "ERROR - Ping timeout (no device found)");
                    break;
                case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                    ESP_LOGE(TAG, "ERROR - Echo timeout (i.e. distance too big)");
                    break;
                default:
                    ESP_LOGE(TAG, "ERROR - %d", res);
            }
        }

        ESP_LOGE(TAG, "Distance: %d cm", distance);
        xQueueSend(bufferDistance, &distance, pdMS_TO_TICKS(0));

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void task_dht(void *pvParameters) {
    int16_t temperature = 0;
    int16_t humidity = 0;

    while (1) {
        if (dht_read_data(DHT_TYPE_DHT11, DHT, &humidity, &temperature) == ESP_OK) {
            humidity = humidity / 10;
            temperature = temperature / 10;
            ESP_LOGE(TAG, "Humidity: %d%% Temp: %dC", humidity, temperature);
            xQueueSend(bufferTemperature, &temperature, pdMS_TO_TICKS(0));
        } else {
            ESP_LOGE(TAG, "Could not read data from sensor");
        }

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void task_oled(void *pvParameters)
{
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.sda = PIN_SDA;
    u8g2_esp32_hal.scl = PIN_SCL;
    u8g2_esp32_hal_init(u8g2_esp32_hal);

    u8g2_Setup_ssd1306_i2c_128x64_noname_f(
        &u8g2,
        U8G2_R0,
        u8g2_esp32_i2c_byte_cb,
        u8g2_esp32_gpio_and_delay_cb
    );

    u8x8_SetI2CAddress(&u8g2.u8x8, 0x78); //0x3C
    u8g2_InitDisplay(&u8g2);

    u8g2_SetPowerSave(&u8g2, 0);
    u8g2_ClearBuffer(&u8g2);
    u8g2_DrawFrame(&u8g2, 0, 0, 128, 64);
    u8g2_SetFont(&u8g2, u8g2_font_6x10_mf);
    u8g2_DrawUTF8(&u8g2, 15, 15, "IoT Aplicada");
    u8g2_SendBuffer(&u8g2);
    uint16_t temp;
    char stringTemperatura[10];
    uint16_t dist;
    char stringDistancia[10];
    while(1) {
        xQueueReceive(bufferTemperature, &temp, pdMS_TO_TICKS(2000));
        u8g2_DrawUTF8(&u8g2, 15, 30, "Temp: ");
        sprintf(stringTemperatura, "%d C", temp);
        u8g2_DrawUTF8(&u8g2,80,30, stringTemperatura);
        ESP_LOGE(TAG, "stringTemperatura: %s", stringTemperatura);
        ESP_LOGE(TAG, "temp: %d", temp);
        xQueueReceive(bufferDistance, &dist, pdMS_TO_TICKS(2000));
        u8g2_DrawUTF8(&u8g2,15, 45, "Dist: ");
        ESP_LOGE(TAG, "stringDistancia: %s", stringDistancia);
        ESP_LOGE(TAG, "dist: %d", dist);
        sprintf(stringDistancia,"%d cm", dist);
        u8g2_DrawUTF8(&u8g2,80, 45, stringDistancia);
        u8g2_SendBuffer(&u8g2);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void app_main(void) {
    gpio_pad_select_gpio(DHT);
    gpio_set_direction(DHT, GPIO_MODE_INPUT);
    gpio_set_pull_mode(DHT, GPIO_PULLUP_ONLY);

    gpio_pad_select_gpio(TRIGGER);
    gpio_set_direction(TRIGGER, GPIO_MODE_INPUT);
    gpio_pad_select_gpio(ECHO);
    gpio_set_direction(ECHO, GPIO_MODE_INPUT);

    bufferTemperature = xQueueCreate(5, sizeof(uint16_t));
    bufferDistance = xQueueCreate(5, sizeof(uint16_t));

    xTaskCreate(task_ultrasonic, "task_ultrasonic", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    xTaskCreate(task_dht, "task_dht", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    xTaskCreate(task_oled, "task_oled", 2048, NULL, 2, NULL);
}