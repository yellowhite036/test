#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_twai_types.h"

#define MAX_MOTORS 16

// Motor types matching DM_CAN.py
typedef enum {
    DM4310 = 0,
    DM4310_48V = 1,
    DM4340 = 2,
    DM4340_48V = 3,
    DM6006 = 4,
    DM8006 = 5,
    DM8009 = 6,
    DM10010L = 7,
    DM10010 = 8,
    DMH3510 = 9,
    DMH6215 = 10,
    DMG6220 = 11
} dm_motor_type_t;

typedef enum {
    CTRL_MIT = 1,
    CTRL_POS_VEL = 2,
    CTRL_VEL = 3,
    CTRL_TORQUE_POS = 4
} dm_control_type_t;

typedef struct {
    uint8_t slave_id;
    uint8_t master_id;
    dm_motor_type_t motor_type;
    dm_control_type_t control_mode;
    float position;
    float velocity;
    float torque;
    bool enabled;
    bool connected;
} dm_motor_t;

// Motor management
extern dm_motor_t motors[MAX_MOTORS];
extern int32_t motor_count;

// TWAI node handle (new driver)
extern twai_node_handle_t twai_node;

// Motor limit parameters [PMAX, VMAX, TMAX] - matches DM_CAN.py
typedef struct {
    float p_max;
    float v_max;
    float t_max;
} motor_limits_t;

extern const motor_limits_t motor_limits[12];

// Function declarations
esp_err_t init_can(void);
esp_err_t motor_enable(dm_motor_t *motor);
esp_err_t motor_disable(dm_motor_t *motor);
esp_err_t motor_set_zero_position(dm_motor_t *motor);
esp_err_t motor_control_mit(dm_motor_t *motor, float kp, float kd, float q, float dq, float tau);
esp_err_t motor_control_pos_vel(dm_motor_t *motor, float position, float velocity);
esp_err_t motor_control_vel(dm_motor_t *motor, float velocity);
esp_err_t motor_control_pos_force(dm_motor_t *motor, float pos_des, float vel_des, float i_des);
esp_err_t motor_refresh_status(dm_motor_t *motor);
esp_err_t switch_control_mode(dm_motor_t *motor, dm_control_type_t mode);
dm_motor_t* find_motor_by_id(int motor_id);
void process_can_message(const twai_frame_t *frame);
void can_rx_task(void *pvParameters);
void can_error_monitor_task(void *pvParameters);
void log_can_status(void);
bool twai_on_rx_done_callback(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx);
esp_err_t load_motor_config(void);

// Utility functions
uint16_t float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(uint16_t x, float x_min, float x_max, int bits);
void float_to_uint8s(float value, uint8_t *bytes);
float uint8s_to_float(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3);

#endif // MOTOR_CONTROL_H