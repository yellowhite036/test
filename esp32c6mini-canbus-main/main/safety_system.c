#include "safety_system.h"
#include "esp_log.h"
#include "esp_now.h"
#include <math.h>

static const char *TAG = "SAFETY_SYSTEM";

safety_config_t safety_config = {
    .enabled = true,
    .position_limit = 360.0f,
    .jump_threshold = 30.0f,
    .velocity_limit = 100.0f,
    .emergency_stop = false
};

bool validate_motor_position(dm_motor_t *motor, float new_position) {
    if (!safety_config.enabled) return true;
    
    // Check position limits
    if (fabs(new_position) > safety_config.position_limit) {
        ESP_LOGW(TAG, "Motor %d position %.2f exceeds limit %.2f", 
                motor->slave_id, new_position, safety_config.position_limit);
        return false;
    }
    
    // Check for position jumps
    float position_diff = fabs(new_position - motor->position);
    if (position_diff > safety_config.jump_threshold) {
        ESP_LOGW(TAG, "Motor %d position jump %.2f exceeds threshold %.2f", 
                motor->slave_id, position_diff, safety_config.jump_threshold);
        return false;
    }
    
    return true;
}

bool validate_motor_velocity(dm_motor_t *motor, float velocity) {
    if (!safety_config.enabled) return true;
    
    if (fabs(velocity) > safety_config.velocity_limit) {
        ESP_LOGW(TAG, "Motor %d velocity %.2f exceeds limit %.2f", 
                motor->slave_id, velocity, safety_config.velocity_limit);
        return false;
    }
    
    return true;
}

void emergency_stop_all_motors(void) {
    ESP_LOGE(TAG, "EMERGENCY STOP ACTIVATED - Disabling all motors");
    safety_config.emergency_stop = true;
    
    for (int i = 0; i < motor_count; i++) {
        motor_disable(&motors[i]);
        motors[i].enabled = false;
    }
    
    // Send ESP-NOW emergency stop message
    uint8_t emergency_msg[16];
    emergency_msg[0] = 0xFF; // Emergency stop marker
    esp_now_send(NULL, emergency_msg, 16);
}

void reset_emergency_stop(void) {
    ESP_LOGI(TAG, "Emergency stop reset");
    safety_config.emergency_stop = false;
}