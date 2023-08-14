#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"
#include "driver/gpio.h"

#define RELAY_GPIO GPIO_NUM_27
#define PCA9685_I2C_ADDRESS 0x40

static const char *TAG = "spot-micro";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting application");

    gpio_set_direction(RELAY_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(RELAY_GPIO, 1); // 1 = on, 0 = off

    while (true)
    {
        gpio_set_level(RELAY_GPIO, 0);
        vTaskDelay(1000);
        gpio_set_level(RELAY_GPIO, 1);
        vTaskDelay(1000);
    }
}
