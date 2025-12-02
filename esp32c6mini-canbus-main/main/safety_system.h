#ifndef SAFETY_SYSTEM_H
#define SAFETY_SYSTEM_H

#include <stdbool.h>
#include "motor_control.h"
#include "esp_err.h"

typedef struct {
    bool enabled;
    float position_limit;
    float jump_threshold;
    float velocity_limit;
    bool emergency_stop;
} safety_config_t;

extern safety_config_t safety_config;

// Function declarations
bool validate_motor_position(dm_motor_t *motor, float new_position);
bool validate_motor_velocity(dm_motor_t *motor, float velocity);
void emergency_stop_all_motors(void);
void reset_emergency_stop(void);

#endif // SAFETY_SYSTEM_H