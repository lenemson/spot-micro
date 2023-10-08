#include "pca9685.h"

#define PCA9685_I2C_ADDRESS PCA9685_ADDR_BASE
#define PCA9685_PWM_FREQ_HZ 50
#define PCA9685_SDA_GPIO GPIO_NUM_18
#define PCA9685_SCL_GPIO GPIO_NUM_19

#define DEFAULT_DURATION_MS 10

#define LEG_FL 0 // Front left leg
#define LEG_FR 1 // Front right leg
#define LEG_RL 2 // Rear left leg
#define LEG_RR 3 // Rear right leg

typedef struct
{
    uint8_t channel;
    float current_angle;
    float target_angle;
    int64_t last_update_time;
    int elapsed_time;
    int duration_ms;
} ServoState;

void init_servos();
void set_servos_states(float target_angles[12]);
void set_leg_state(uint8_t leg_id, float target_angles[3]);
void update_servos(int update_interval_ms);