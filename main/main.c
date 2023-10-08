#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "servo.h"
#include "ik.h"

#define RELAY_GPIO GPIO_NUM_27

static const char *TAG = "spot-micro";
// float rest_position_angles[12] = {90, 160, 0, 90, 30, 180, 90, 160, 0, 90, 30, 180};
// float stand_up_position_angles[12] = {90, 120, 90, 90, 60, 90, 90, 120, 90, 90, 60, 90};

void set_spot_height(float height)
{
    float leg_angles[3];

    leg_height_ik(height, leg_angles);

    ESP_LOGI(TAG, "set spot height=%f -> (%f, %f, %f)", height, leg_angles[0], leg_angles[1], leg_angles[2]);

    set_leg_state(LEG_FL, leg_angles);
    set_leg_state(LEG_RL, leg_angles);

    // Rigth side servos mirrors left servos so
    // when a given servo is 0 degrees the oposite is 180
    leg_angles[1] = 180.0 - leg_angles[1];
    leg_angles[2] = 180.0 - leg_angles[2];

    set_leg_state(LEG_FR, leg_angles);
    set_leg_state(LEG_RR, leg_angles);
    update_servos(50);
}

/**
 * Make spot stand up and down
 */
void up_and_down(void *pvParameters)
{
    while (true)
    {
        for (int c = 20; c < 180; c += 2)
        {
            set_spot_height(c);
        }

        for (int c = 180; c > 20; c -= 2)
        {
            set_spot_height(c);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting application");

    // Configure relay GPIO as output.
    // Relay is on by default.
    gpio_set_direction(RELAY_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(RELAY_GPIO, 1); // 1 = on, 0 = off

    init_servos();

    xTaskCreate(up_and_down, "up_and_down", 1024 * 2, (void *)0, 10, NULL);
}
