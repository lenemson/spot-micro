#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "pca9685.h"
#include "servo.h"

static const char *TAG = "servo";
i2c_dev_t pca;
ServoState servo_states[12];

/**
 * Servo channels stored by leg id.
 * See servo.h for the definition of leg ids.
 * See https://github.com/michaelkubina/SpotMicroESP32/tree/b192facc57606a074d5a535afebaf3f2a5523ee7/electronics
 * for the wiring of servos to the pca board.
 */
uint8_t servos_by_leg[4][3] = {
    {0, 1, 2},   // Front left leg
    {3, 4, 5},   // Front right leg
    {6, 7, 8},   // Rear left leg
    {9, 10, 11}, // Rear right leg
};

i2c_dev_t init_pca()
{
    i2c_dev_t pca;
    memset(&pca, 0, sizeof(i2c_dev_t));

    // Initialize the PCA board.
    ESP_ERROR_CHECK(pca9685_init_desc(&pca, PCA9685_I2C_ADDRESS, 0, PCA9685_SDA_GPIO, PCA9685_SCL_GPIO));
    ESP_ERROR_CHECK(pca9685_init(&pca));
    ESP_ERROR_CHECK(pca9685_restart(&pca));

    uint16_t freq;

    // Set PWM frequency.
    ESP_ERROR_CHECK(pca9685_set_pwm_frequency(&pca, PCA9685_PWM_FREQ_HZ));
    ESP_ERROR_CHECK(pca9685_get_pwm_frequency(&pca, &freq));

    ESP_LOGI(TAG, "Frequency set to %dHz, actual frequency: %d", PCA9685_PWM_FREQ_HZ, freq);

    return pca;
}

void init_servos()
{
    // Init i2cdev library
    ESP_ERROR_CHECK(i2cdev_init());

    pca = init_pca();

    // Adjust these defaults based on the robot's initial pose.
    // Rest position.
    float target_angles[12] = {90, 160, 0, 90, 30, 180, 90, 160, 0, 90, 30, 180};

    for (int i = 0; i < 12; i++)
    {
        servo_states[i].channel = i;
        servo_states[i].current_angle = target_angles[i];
        servo_states[i].target_angle = target_angles[i];
        servo_states[i].last_update_time = esp_timer_get_time();
        servo_states[i].elapsed_time = 0;
        servo_states[i].duration_ms = 3000;
    }

    update_servos(50);
}

void set_servo_state(uint8_t servo_channel, float target_angle)
{
    servo_states[servo_channel].target_angle = target_angle;
    servo_states[servo_channel].last_update_time = esp_timer_get_time();
    servo_states[servo_channel].elapsed_time = 0;
    servo_states[servo_channel].duration_ms = DEFAULT_DURATION_MS;
}

/**
 * Set angles for all servos at once
 */
void set_servos_states(float target_angles[12])
{
    for (int i = 0; i < 12; i++)
    {
        set_servo_state(i, target_angles[i]);
    }
}

/**
 * Set angles for all 3 servos of the same leg
 * target_angles[0] is shoulder servo
 * target_angles[1] is upper leg servo
 * target_angles[2] is lower leg servo
 */
void set_leg_state(uint8_t leg_id, float target_angles[3])
{
    for (int i = 0; i < 3; i += 1)
    {
        uint8_t servo_channel = servos_by_leg[leg_id][i];
        set_servo_state(servo_channel, target_angles[i]);
    }
}

float lerp(float start, float end, float t)
{
    return start * (1.0f - t) + end * t;
}

void set_pwm(uint8_t channel, uint16_t pwm)
{
    if (pca9685_set_pwm_value(&pca, channel, pwm) != ESP_OK)
        ESP_LOGE(TAG, "Could not set PWM value (%d) to ch%d", pwm, channel);
}

void set_servo(uint8_t channel, float angle)
{
    // Set this values after servos calibration
    // https://github.com/michaelkubina/SpotMicroESP32/tree/master/assembly#servo-calibration
    const int16_t servo_min[12] = {114, 112, 123, 117, 114, 130, 118, 112, 123, 117, 118, 126};
    const float servo_c_factor[12] = {2.369, 2.371, 2.297, 2.316, 2.349, 2.217, 2.286, 2.376, 2.297, 2.326, 2.326, 2.267};
    uint16_t pwm = (uint16_t)(servo_min[channel] + (angle * servo_c_factor[channel]));

    set_pwm(channel, pwm);
}

void update_servo_position(ServoState *servo)
{
    // Gets current time in microseconds
    int64_t current_time = esp_timer_get_time();
    int elapsed_ms = (current_time - servo->last_update_time) / 1000;

    if (servo->elapsed_time < servo->duration_ms)
    {
        servo->elapsed_time += elapsed_ms;
        float t = (float)servo->elapsed_time / (float)servo->duration_ms;
        t = (t > 1.0f) ? 1.0f : t; // Clamp t to the range [0, 1]
        servo->current_angle = lerp(servo->current_angle, servo->target_angle, t);
        set_servo(servo->channel, servo->current_angle);
    }

    servo->last_update_time = current_time;
}

bool all_servos_reached_target()
{
    for (int i = 0; i < 12; i++)
    {
        if (servo_states[i].elapsed_time < servo_states[i].duration_ms)
        {
            return false; // At least one servo hasn't reached its target
        }
    }
    return true; // All servos have reached their targets
}

void update_servos(int update_interval_ms)
{
    while (true)
    {
        for (int i = 0; i < 12; i++)
        {
            update_servo_position(&servo_states[i]);
        }

        if (all_servos_reached_target())
        {
            break;
        }

        vTaskDelay(update_interval_ms / portTICK_PERIOD_MS);
    }
}