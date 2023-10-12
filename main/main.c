#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include <math.h>
#include "servo.h"
#include "ik.h"

#define RELAY_GPIO GPIO_NUM_27

static const char *TAG = "spot-micro";

void set_spot_height(float height)
{
    float leg_angles[3];

    leg_ik(0, height, leg_angles);

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

float circle(float d, float x)
{
    float r = d / 2;
    return sqrtf(powf(r, 2) - powf(x + r, 2));
}

void move_fl_rr(float x, float y)
{
    float leg_angles[3];

    leg_ik(x, y, leg_angles);

    set_leg_state(LEG_FL, leg_angles);

    // Rigth side servos mirrors left servos so
    // when a given servo is 0 degrees the oposite is 180
    leg_angles[1] = 180.0 - leg_angles[1];
    leg_angles[2] = 180.0 - leg_angles[2];

    set_leg_state(LEG_RR, leg_angles);
    update_servos(50);
}

void move_rl_fr(float x, float y)
{
    float leg_angles[3];

    leg_ik(x, y, leg_angles);

    set_leg_state(LEG_RL, leg_angles);

    // Rigth side servos mirrors left servos so
    // when a given servo is 0 degrees the oposite is 180
    leg_angles[1] = 180.0 - leg_angles[1];
    leg_angles[2] = 180.0 - leg_angles[2];

    set_leg_state(LEG_FR, leg_angles);
    update_servos(50);
}

void move_forward(void *pvParameters)
{
    float from = 0;
    float to = -50;
    float height = 125;
    float diameter = 75;
    float speed = 3;

    while (true)
    {
        for (float x = from; x > to; x -= speed)
        {
            move_fl_rr(x, height - circle(diameter, x));
        }

        for (float x = to; x < from; x += speed)
        {
            move_fl_rr(x, height);
        }

        for (float x = from; x > to; x -= speed)
        {
            move_rl_fr(x, height - circle(diameter, x));
        }

        for (float x = to; x < from; x += speed)
        {
            move_rl_fr(x, height);
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
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

    vTaskDelay(3000 / portTICK_PERIOD_MS);

    // xTaskCreate(up_and_down, "up_and_down", 1024 * 2, (void *)0, 10, NULL);
    xTaskCreate(move_forward, "move_forward", 1024 * 2, (void *)0, 10, NULL);
}
