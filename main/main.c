#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "pca9685.h"

#define RELAY_GPIO GPIO_NUM_27

#define PCA9685_I2C_ADDRESS PCA9685_ADDR_BASE
#define PCA9685_PWM_FREQ_HZ 50
#define PCA9685_SDA_GPIO GPIO_NUM_18
#define PCA9685_SCL_GPIO GPIO_NUM_19

static const char *TAG = "spot-micro";

i2c_dev_t init_pca()
{
    i2c_dev_t pca;
    memset(&pca, 0, sizeof(i2c_dev_t));

    // Initialize the PCA board.
    ESP_ERROR_CHECK(pca9685_init_desc(&pca, PCA9685_I2C_ADDRESS, 0, PCA9685_SDA_GPIO, PCA9685_SCL_GPIO));
    ESP_ERROR_CHECK(pca9685_init(&pca));
    ESP_ERROR_CHECK(pca9685_restart(&pca));

    return pca;
}

void set_pwm(i2c_dev_t *pca, uint8_t channel, uint16_t pwm)
{
    if (pca9685_set_pwm_value(pca, channel, pwm) != ESP_OK)
        ESP_LOGE(TAG, "Could not set PWM value (%d) to ch%d", pwm, channel);
}

void set_servo(i2c_dev_t *pca, uint8_t channel, uint16_t angle)
{
    const int16_t servo_min[12] = {114, 112, 123, 117, 114, 130, 118, 112, 123, 117, 118, 126};
    const float servo_c_factor[12] = {2.369, 2.371, 2.297, 2.316, 2.349, 2.217, 2.286, 2.376, 2.297, 2.326, 2.326, 2.267};
    uint16_t pwm = (uint16_t)(servo_min[channel] + (angle * servo_c_factor[channel]));

    set_pwm(pca, channel, pwm);
}

void rest_and_stand_up(void *pvParameters)
{
    i2c_dev_t pca = init_pca();
    uint16_t freq;

    // Set PWM frequency.
    ESP_ERROR_CHECK(pca9685_set_pwm_frequency(&pca, PCA9685_PWM_FREQ_HZ));
    ESP_ERROR_CHECK(pca9685_get_pwm_frequency(&pca, &freq));

    ESP_LOGI(TAG, "Freq %dHz, real %d", PCA9685_PWM_FREQ_HZ, freq);

    while (true)
    {
        // Spot up position.
        // FL
        set_servo(&pca, 0, 90);
        set_servo(&pca, 1, 120);
        set_servo(&pca, 2, 90);
        // FR
        set_servo(&pca, 3, 90);
        set_servo(&pca, 4, 60);
        set_servo(&pca, 5, 90);
        // RL
        set_servo(&pca, 6, 90);
        set_servo(&pca, 7, 120);
        set_servo(&pca, 8, 90);
        // RR
        set_servo(&pca, 9, 90);
        set_servo(&pca, 10, 60);
        set_servo(&pca, 11, 90);

        vTaskDelay(2000 / portTICK_PERIOD_MS);

        // Spot resting position.
        // FL
        set_servo(&pca, 0, 90);
        set_servo(&pca, 1, 160);
        set_servo(&pca, 2, 0);
        // FR
        set_servo(&pca, 3, 90);
        set_servo(&pca, 4, 30);
        set_servo(&pca, 5, 180);
        // RL
        set_servo(&pca, 6, 90);
        set_servo(&pca, 7, 160);
        set_servo(&pca, 8, 0);
        // RR
        set_servo(&pca, 9, 90);
        set_servo(&pca, 10, 30);
        set_servo(&pca, 11, 180);

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting application");

    // Init i2cdev library
    ESP_ERROR_CHECK(i2cdev_init());

    // Configure relay GPIO as output.
    // Relay is on by default.
    gpio_set_direction(RELAY_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(RELAY_GPIO, 1); // 1 = on, 0 = off

    xTaskCreate(rest_and_stand_up, "rest_and_stand_up", 1024 * 2, (void *)0, 10, NULL);

    ESP_LOGI(TAG, "Stopping application");
}
