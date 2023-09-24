#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "pca9685.h"

#define RELAY_GPIO GPIO_NUM_27

#define PCA9685_I2C_ADDRESS PCA9685_ADDR_BASE
#define PCA9685_PWM_FREQ_HZ 50
#define PCA9685_SDA_GPIO GPIO_NUM_18
#define PCA9685_SCL_GPIO GPIO_NUM_19

#define DEFAULT_DURATION_MS 3000

typedef struct {
    uint8_t channel;
    uint16_t current_angle;
    uint16_t target_angle;
    int64_t last_update_time;
    int elapsed_time;
    int duration_ms;
} ServoState;

static const char *TAG = "spot-micro";
ServoState servo_states[12];

// Set this values after servos calibration
// https://github.com/michaelkubina/SpotMicroESP32/tree/master/assembly#servo-calibration
uint16_t rest_position_angles[12] = {90, 160, 0, 90, 30, 180, 90, 160, 0, 90, 30, 180};
uint16_t stand_up_position_angles[12] = {90, 120, 90, 90, 60, 90, 90, 120, 90, 90, 60, 90}; 

void init_servo_states()
{
    uint16_t default_angles[12];
    uint16_t target_angles[12];

    // Adjust these defaults based on the robot's initial pose
    // Copy rest_position_angles to default_angles and target_angles
    memcpy(default_angles, rest_position_angles, sizeof(default_angles));
    memcpy(target_angles, rest_position_angles, sizeof(target_angles));

    for (int i = 0; i < 12; i++) {
        servo_states[i].channel = i;
        servo_states[i].current_angle = default_angles[i];
        servo_states[i].target_angle = target_angles[i];
        servo_states[i].last_update_time = esp_timer_get_time();
        servo_states[i].elapsed_time = 0;
        servo_states[i].duration_ms = DEFAULT_DURATION_MS;
    }
}

void set_servos_states(uint16_t target_angles[12])
{
    for (int i = 0; i < 12; i++) {
        servo_states[i].target_angle = target_angles[i];
        servo_states[i].last_update_time = esp_timer_get_time();
        servo_states[i].elapsed_time = 0;
        servo_states[i].duration_ms = DEFAULT_DURATION_MS;
    }
}

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

uint16_t lerp(uint16_t start, uint16_t end, float t) {
    return (uint16_t)(start * (1.0f - t) + end * t);
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

void update_servo_position(i2c_dev_t *pca, ServoState *servo) {
    // Gets current time in microseconds
    int64_t current_time = esp_timer_get_time();  
    int elapsed_ms = (current_time - servo->last_update_time) / 1000;

    if (servo->elapsed_time < servo->duration_ms) {
        servo->elapsed_time += elapsed_ms;
        float t = (float)servo->elapsed_time / (float)servo->duration_ms;
        t = (t > 1.0f) ? 1.0f : t;  // Clamp t to the range [0, 1]
        servo->current_angle = lerp(servo->current_angle, servo->target_angle, t);
        set_servo(pca, servo->channel, servo->current_angle);
    }

    servo->last_update_time = current_time;
}

bool all_servos_reached_target()
{
    for (int i = 0; i < 12; i++) {
        if (servo_states[i].elapsed_time < servo_states[i].duration_ms) {
            return false; // At least one servo hasn't reached its target
        }
    }
    return true; // All servos have reached their targets
}

void update_servos(i2c_dev_t *pca, int update_interval_ms)
{
    while (true) {
        for (int i = 0; i < 12; i++) {
            update_servo_position(pca, &servo_states[i]);
        }

        if (all_servos_reached_target()) {
            break;
        }

        vTaskDelay(update_interval_ms / portTICK_PERIOD_MS);
    }
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
        set_servos_states(stand_up_position_angles);
        update_servos(&pca, 50); // try 20ms to (50Hz)

        vTaskDelay(2000 / portTICK_PERIOD_MS);

        set_servos_states(rest_position_angles);
        update_servos(&pca, 50);

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

    init_servo_states();

    xTaskCreate(rest_and_stand_up, "rest_and_stand_up", 1024 * 2, (void *)0, 10, NULL);
}
