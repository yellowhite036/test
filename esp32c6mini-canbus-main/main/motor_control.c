#include "motor_control.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nvs.h"
#include <string.h>
#include <inttypes.h>
#include <math.h>

// TXD0 and RXD0 pins for CAN transceiver
#define CAN_TX_GPIO_NUM GPIO_NUM_16
#define CAN_RX_GPIO_NUM GPIO_NUM_17

static const char *TAG = "MOTOR_CONTROL";
static QueueHandle_t can_rx_queue;

dm_motor_t motors[MAX_MOTORS];
int32_t motor_count = 0;

// TWAI node handle for new driver
twai_node_handle_t twai_node = NULL;

// Helper function to create and send a TWAI frame
static esp_err_t twai_send_frame(uint32_t id, const uint8_t *data, size_t len, uint32_t timeout_ms) {
    twai_frame_header_t header = {
        .id = id,
        .dlc = len,
        .ide = 0,  // Standard frame
        .rtr = 0,  // Data frame
        .fdf = 0,  // Classic CAN
        .brs = 0,
        .esi = 0,
    };

    twai_frame_t frame = {
        .header = header,
        .buffer = (uint8_t *)data,
        .buffer_len = len
    };

    return twai_node_transmit(twai_node, &frame, pdMS_TO_TICKS(timeout_ms));
}

// Motor limit parameters matching DM_CAN.py Limit_Param
const motor_limits_t motor_limits[12] = {
    {12.5f, 30.0f, 10.0f},   // DM4310
    {12.5f, 50.0f, 10.0f},   // DM4310_48V
    {12.5f, 8.0f, 28.0f},    // DM4340
    {12.5f, 10.0f, 28.0f},   // DM4340_48V
    {12.5f, 45.0f, 20.0f},   // DM6006
    {12.5f, 45.0f, 40.0f},   // DM8006
    {12.5f, 45.0f, 54.0f},   // DM8009
    {12.5f, 25.0f, 200.0f},  // DM10010L
    {12.5f, 20.0f, 200.0f},  // DM10010
    {12.5f, 280.0f, 1.0f},   // DMH3510
    {12.5f, 45.0f, 10.0f},   // DMH6215
    {12.5f, 45.0f, 10.0f}    // DMG6220
};

// Utility functions matching DM_CAN.py
uint16_t float_to_uint(float x, float x_min, float x_max, int bits) {
    if (x <= x_min) x = x_min;
    else if (x > x_max) x = x_max;
    
    float span = x_max - x_min;
    float data_norm = (x - x_min) / span;
    return (uint16_t)(data_norm * ((1 << bits) - 1));
}

float uint_to_float(uint16_t x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float data_norm = (float)x / ((1 << bits) - 1);
    return data_norm * span + x_min;
}

void float_to_uint8s(float value, uint8_t *bytes) {
    union {
        float f;
        uint8_t b[4];
    } floatUnion;
    
    floatUnion.f = value;
    bytes[0] = floatUnion.b[0];
    bytes[1] = floatUnion.b[1];
    bytes[2] = floatUnion.b[2];
    bytes[3] = floatUnion.b[3];
}

float uint8s_to_float(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
    union {
        float f;
        uint8_t b[4];
    } floatUnion;
    
    floatUnion.b[0] = b0;
    floatUnion.b[1] = b1;
    floatUnion.b[2] = b2;
    floatUnion.b[3] = b3;
    return floatUnion.f;
}

esp_err_t init_can(void)
{
    // Configure the onchip TWAI node
    twai_onchip_node_config_t node_config = {
        .io_cfg = {
            .tx = CAN_TX_GPIO_NUM,
            .rx = CAN_RX_GPIO_NUM,
            .quanta_clk_out = -1,
            .bus_off_indicator = -1,
        },
        .clk_src = 0,  // Use default clock source
        .bit_timing = {
            .bitrate = 1000000,  // 1 Mbps
            .sp_permill = 0,      // Use default sample point
            .ssp_permill = 0,
        },
        .fail_retry_cnt = -1,  // Retry forever
        .tx_queue_depth = 10,
        .intr_priority = 0,
        .flags = {
            .enable_self_test = 0,
            .enable_loopback = 0,
            .enable_listen_only = 0,
            .no_receive_rtr = 0,
        }
    };

    // Create the TWAI node
    esp_err_t result = twai_new_node_onchip(&node_config, &twai_node);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create TWAI node: %s", esp_err_to_name(result));
        return result;
    }

    // Register RX callback
    twai_event_callbacks_t callbacks = {
        .on_rx_done = twai_on_rx_done_callback,
    };
    result = twai_node_register_event_callbacks(twai_node, &callbacks, NULL);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register TWAI callbacks: %s", esp_err_to_name(result));
        twai_node_delete(twai_node);
        return result;
    }

    // Create queue for processing messages
    can_rx_queue = xQueueCreate(10, sizeof(twai_frame_t));
    if (can_rx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create CAN RX queue");
        twai_node_delete(twai_node);
        return ESP_FAIL;
    }

    // Enable/start the TWAI node
    result = twai_node_enable(twai_node);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable TWAI node: %s", esp_err_to_name(result));
        twai_node_delete(twai_node);
        return result;
    }

    ESP_LOGI(TAG, "TWAI node initialized successfully (new driver)");
    return ESP_OK;
}

esp_err_t motor_enable(dm_motor_t *motor)
{
    ESP_LOGI(TAG, "Enabling motor %d", motor->slave_id);

    uint8_t data[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    esp_err_t result = twai_send_frame(motor->slave_id, data, sizeof(data), 1000);

    if (result == ESP_OK) {
        motor->enabled = true;
        ESP_LOGI(TAG, "Motor %d enabled successfully", motor->slave_id);
    } else {
        ESP_LOGE(TAG, "Failed to enable motor %d: %s", motor->slave_id, esp_err_to_name(result));
        ESP_LOGE(TAG, "CAN TX Error - checking CAN bus status:");
        log_can_status();
    }
    return result;
}

esp_err_t motor_disable(dm_motor_t *motor)
{
    ESP_LOGI(TAG, "Disabling motor %d", motor->slave_id);

    uint8_t data[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    esp_err_t result = twai_send_frame(motor->slave_id, data, sizeof(data), 1000);

    if (result == ESP_OK) {
        motor->enabled = false;
        ESP_LOGI(TAG, "Motor %d disabled successfully", motor->slave_id);
    } else {
        ESP_LOGE(TAG, "Failed to disable motor %d: %s", motor->slave_id, esp_err_to_name(result));
        ESP_LOGE(TAG, "CAN TX Error - checking CAN bus status:");
        log_can_status();
    }
    return result;
}

esp_err_t motor_set_zero_position(dm_motor_t *motor)
{
    ESP_LOGI(TAG, "Setting zero position for motor %d", motor->slave_id);

    uint8_t data[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    esp_err_t result = twai_send_frame(motor->slave_id, data, sizeof(data), 1000);

    if (result == ESP_OK) {
        motor->position = 0.0f;
        ESP_LOGI(TAG, "Motor %d zero position set successfully", motor->slave_id);
    } else {
        ESP_LOGE(TAG, "Failed to send zero position command to motor %d: %s", motor->slave_id, esp_err_to_name(result));
        ESP_LOGE(TAG, "CAN TX Error - checking CAN bus status:");
        log_can_status();
    }
    return result;
}

esp_err_t motor_control_mit(dm_motor_t *motor, float kp, float kd, float q, float dq, float tau)
{
    if (motor->motor_type >= 12) return ESP_ERR_INVALID_ARG;

    const motor_limits_t *limits = &motor_limits[motor->motor_type];

    uint16_t kp_uint = float_to_uint(kp, 0, 500, 12);
    uint16_t kd_uint = float_to_uint(kd, 0, 5, 12);
    uint16_t q_uint = float_to_uint(q, -limits->p_max, limits->p_max, 16);
    uint16_t dq_uint = float_to_uint(dq, -limits->v_max, limits->v_max, 12);
    uint16_t tau_uint = float_to_uint(tau, -limits->t_max, limits->t_max, 12);

    uint8_t data[] = {
        (q_uint >> 8) & 0xFF,
        q_uint & 0xFF,
        dq_uint >> 4,
        ((dq_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F),
        kp_uint & 0xFF,
        kd_uint >> 4,
        ((kd_uint & 0x0F) << 4) | ((tau_uint >> 8) & 0x0F),
        tau_uint & 0xFF
    };

    esp_err_t result = twai_send_frame(motor->slave_id, data, sizeof(data), 1000);
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "Motor %d MIT control: kp=%.2f, kd=%.2f, q=%.2f, dq=%.2f, tau=%.2f",
                motor->slave_id, kp, kd, q, dq, tau);
    }
    return result;
}

esp_err_t motor_control_pos_vel(dm_motor_t *motor, float position, float velocity)
{
    // Position/velocity control mode (matches DM_CAN.py control_Pos_Vel)
    uint8_t pos_bytes[4], vel_bytes[4];
    float_to_uint8s(position, pos_bytes);
    float_to_uint8s(velocity, vel_bytes);

    uint8_t data[] = {
        pos_bytes[0], pos_bytes[1], pos_bytes[2], pos_bytes[3],
        vel_bytes[0], vel_bytes[1], vel_bytes[2], vel_bytes[3]
    };

    esp_err_t result = twai_send_frame(0x100 + motor->slave_id, data, sizeof(data), 1000);
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "Motor %d position/velocity command sent: pos=%.2f, vel=%.2f",
                motor->slave_id, position, velocity);
    }
    return result;
}

esp_err_t motor_control_vel(dm_motor_t *motor, float velocity)
{
    // Velocity control mode (matches DM_CAN.py control_Vel)
    uint8_t vel_bytes[4];
    float_to_uint8s(velocity, vel_bytes);

    uint8_t data[] = {vel_bytes[0], vel_bytes[1], vel_bytes[2], vel_bytes[3], 0, 0, 0, 0};

    esp_err_t result = twai_send_frame(0x200 + motor->slave_id, data, sizeof(data), 1000);
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "Motor %d velocity command sent: vel=%.2f", motor->slave_id, velocity);
    }
    return result;
}

esp_err_t motor_control_pos_force(dm_motor_t *motor, float pos_des, float vel_des, float i_des)
{
    // Position/force control mode (matches DM_CAN.py control_pos_force)
    uint8_t pos_bytes[4];
    float_to_uint8s(pos_des, pos_bytes);

    uint16_t vel_uint = (uint16_t)vel_des;
    uint16_t i_uint = (uint16_t)i_des;

    uint8_t data[] = {
        pos_bytes[0], pos_bytes[1], pos_bytes[2], pos_bytes[3],
        vel_uint & 0xFF, vel_uint >> 8,
        i_uint & 0xFF, i_uint >> 8
    };

    esp_err_t result = twai_send_frame(0x300 + motor->slave_id, data, sizeof(data), 1000);
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "Motor %d position/force command sent: pos=%.2f, vel=%.2f, i=%.2f",
                motor->slave_id, pos_des, vel_des, i_des);
    }
    return result;
}

esp_err_t motor_refresh_status(dm_motor_t *motor)
{
    uint8_t data[] = {
        motor->slave_id & 0xFF,
        (motor->slave_id >> 8) & 0xFF,
        0xCC,
        0x00, 0x00, 0x00, 0x00, 0x00
    };

    return twai_send_frame(0x7FF, data, sizeof(data), 100);
}

esp_err_t switch_control_mode(dm_motor_t *motor, dm_control_type_t mode)
{
    ESP_LOGI(TAG, "Switching motor %d control mode from %d to %d", motor->slave_id, motor->control_mode, mode);

    // Validate control mode
    if (mode < CTRL_MIT || mode > CTRL_TORQUE_POS) {
        ESP_LOGE(TAG, "Invalid control mode %d for motor %d", mode, motor->slave_id);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[] = {
        motor->slave_id & 0xFF,
        (motor->slave_id >> 8) & 0xFF,
        0x55,
        10, // RID for control mode
        (uint8_t)mode, 0x00, 0x00, 0x00
    };

    esp_err_t result = twai_send_frame(0x7FF, data, sizeof(data), 1000);
    if (result == ESP_OK) {
        motor->control_mode = mode;
        ESP_LOGI(TAG, "Motor %d control mode switched to %d successfully", motor->slave_id, mode);
    } else {
        ESP_LOGE(TAG, "Failed to switch control mode for motor %d: %s", motor->slave_id, esp_err_to_name(result));
        ESP_LOGE(TAG, "CAN TX Error - checking CAN bus status:");
        log_can_status();
    }
    return result;
}

dm_motor_t* find_motor_by_id(int motor_id)
{
    for (int i = 0; i < motor_count; i++) {
        if (motors[i].slave_id == motor_id) {
            return &motors[i];
        }
    }
    return NULL;
}

void process_can_message(const twai_frame_t *frame)
{
    // Process motor feedback messages (matches DM_CAN.py __process_packet)
    if (frame->header.dlc >= 8 && frame->buffer != NULL && frame->buffer_len >= 8) {
        uint32_t can_id = frame->header.id;
        const uint8_t *data = frame->buffer;
        uint8_t cmd = data[1]; // CMD position in the protocol

        if (cmd == 0x11) { // Motor feedback command
            dm_motor_t *motor = NULL;

            // Find motor by CAN ID or Master ID
            if (can_id != 0x00) {
                motor = find_motor_by_id((int)can_id);
            } else {
                // Handle Master ID case
                uint8_t master_id = data[0] & 0x0F;
                for (int i = 0; i < motor_count; i++) {
                    if (motors[i].master_id == master_id) {
                        motor = &motors[i];
                        break;
                    }
                }
            }

            if (motor && motor->motor_type < 12) {
                // Parse motor feedback data according to DM protocol
                uint16_t q_uint = (data[2] << 8) | data[3];
                uint16_t dq_uint = (data[4] << 4) | (data[5] >> 4);
                uint16_t tau_uint = ((data[5] & 0x0F) << 8) | data[6];

                const motor_limits_t *limits = &motor_limits[motor->motor_type];

                motor->position = uint_to_float(q_uint, -limits->p_max, limits->p_max, 16);
                motor->velocity = uint_to_float(dq_uint, -limits->v_max, limits->v_max, 12);
                motor->torque = uint_to_float(tau_uint, -limits->t_max, limits->t_max, 12);
                motor->connected = true;

                ESP_LOGD(TAG, "Motor %d feedback: pos=%.3f, vel=%.3f, torque=%.3f",
                        motor->slave_id, motor->position, motor->velocity, motor->torque);
            }
        }
    }
}

// RX callback for new TWAI driver (called from ISR context)
bool twai_on_rx_done_callback(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    // Receive frame from ISR
    static uint8_t rx_buffer[64];  // Buffer for frame data
    twai_frame_t rx_frame = {
        .buffer = rx_buffer,
        .buffer_len = sizeof(rx_buffer)
    };

    esp_err_t result = twai_node_receive_from_isr(handle, &rx_frame);
    if (result == ESP_OK) {
        // Send frame to queue for processing in task context
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(can_rx_queue, &rx_frame, &xHigherPriorityTaskWoken);
        return xHigherPriorityTaskWoken == pdTRUE;
    }

    return false;
}

void can_rx_task(void *pvParameters)
{
    static uint8_t rx_buffer[64];  // Buffer for frame data
    twai_frame_t rx_frame;

    ESP_LOGI(TAG, "CAN RX task started");

    while (1) {
        // Wait for frames from the queue (populated by ISR callback)
        if (xQueueReceive(can_rx_queue, &rx_frame, pdMS_TO_TICKS(1000)) == pdTRUE) {
            // Copy the data to our local buffer since the ISR buffer is shared
            if (rx_frame.buffer && rx_frame.buffer_len <= sizeof(rx_buffer)) {
                memcpy(rx_buffer, rx_frame.buffer, rx_frame.buffer_len);
                rx_frame.buffer = rx_buffer;
                process_can_message(&rx_frame);
            }
        }
    }
}

esp_err_t load_motor_config(void)
{
    // Try to load from NVS first
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("motor_config", NVS_READONLY, &nvs_handle);
    if (err == ESP_OK) {
        err = nvs_get_i32(nvs_handle, "motor_count", &motor_count);
        if (err == ESP_OK && motor_count > 0 && motor_count <= MAX_MOTORS) {
            for (int i = 0; i < motor_count; i++) {
                char key[32];
                snprintf(key, sizeof(key), "motor_%d", i);
                size_t required_size = sizeof(dm_motor_t);
                nvs_get_blob(nvs_handle, key, &motors[i], &required_size);
            }
            nvs_close(nvs_handle);
            ESP_LOGI(TAG, "Loaded motor configuration from NVS: %" PRId32 " motors", motor_count);
            return ESP_OK;
        }
        nvs_close(nvs_handle);
    }
    
    ESP_LOGI(TAG, "Loading default motor configuration with common DM motors");
    
    // Default motor configuration for testing
    motor_count = 0;
    
    // Motor 1: DM4340
    motors[motor_count].slave_id = 1;
    motors[motor_count].master_id = 17;
    motors[motor_count].motor_type = DM4340;
    motors[motor_count].enabled = false;
    motors[motor_count].connected = false;
    motors[motor_count].control_mode = CTRL_MIT;
    motors[motor_count].position = 0.0f;
    motors[motor_count].velocity = 0.0f;
    motors[motor_count].torque = 0.0f;
    motor_count++;
    
    // Motor 2: DM6006
    motors[motor_count].slave_id = 2;
    motors[motor_count].master_id = 18;
    motors[motor_count].motor_type = DM6006;
    motors[motor_count].enabled = false;
    motors[motor_count].connected = false;
    motors[motor_count].control_mode = CTRL_MIT;
    motors[motor_count].position = 0.0f;
    motors[motor_count].velocity = 0.0f;
    motors[motor_count].torque = 0.0f;
    motor_count++;
    
    // Motor 3: DMH3510
    motors[motor_count].slave_id = 3;
    motors[motor_count].master_id = 19;
    motors[motor_count].motor_type = DMH3510;
    motors[motor_count].enabled = false;
    motors[motor_count].connected = false;
    motors[motor_count].control_mode = CTRL_MIT;
    motors[motor_count].position = 0.0f;
    motors[motor_count].velocity = 0.0f;
    motors[motor_count].torque = 0.0f;
    motor_count++;
    
    ESP_LOGI(TAG, "Loaded %" PRId32 " default motors for testing", motor_count);
    return ESP_OK;
}

void log_can_status(void) {
    if (twai_node == NULL) {
        ESP_LOGE(TAG, "TWAI node not initialized");
        return;
    }

    twai_node_status_t status;
    twai_node_record_t stats;
    esp_err_t result = twai_node_get_info(twai_node, &status, &stats);

    if (result == ESP_OK) {
        ESP_LOGI(TAG, "=== TWAI Node Status ===");

        // Decode state
        const char* state_str = "UNKNOWN";
        switch(status.state) {
            case TWAI_ERROR_ACTIVE: state_str = "ACTIVE"; break;
            case TWAI_ERROR_WARNING: state_str = "WARNING"; break;
            case TWAI_ERROR_PASSIVE: state_str = "PASSIVE"; break;
            case TWAI_ERROR_BUS_OFF: state_str = "BUS_OFF"; break;
        }

        ESP_LOGI(TAG, "State: %s", state_str);
        ESP_LOGI(TAG, "TX Error Counter: %u", status.tx_error_count);
        ESP_LOGI(TAG, "RX Error Counter: %u", status.rx_error_count);
        ESP_LOGI(TAG, "Bus Errors: %"PRIu32, stats.bus_err_num);

        // Warnings
        if (status.state == TWAI_ERROR_BUS_OFF) {
            ESP_LOGE(TAG, "CAN BUS OFF - Check wiring and termination!");
        }
        if (status.tx_error_count > 96 || status.rx_error_count > 96) {
            ESP_LOGW(TAG, "High error counters - Check CAN bus quality!");
        }
    } else {
        ESP_LOGE(TAG, "Failed to get TWAI node info: %s", esp_err_to_name(result));
    }
}

void can_error_monitor_task(void *pvParameters) {
    ESP_LOGI(TAG, "CAN error monitor task started");

    twai_node_status_t prev_status = {0};
    TickType_t last_log_time = 0;

    while (1) {
        if (twai_node == NULL) {
            ESP_LOGE(TAG, "TWAI node not initialized in monitor task");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        twai_node_status_t current_status;
        twai_node_record_t current_stats;
        esp_err_t result = twai_node_get_info(twai_node, &current_status, &current_stats);

        if (result == ESP_OK) {
            bool should_log = false;

            // Log every 30 seconds for basic status
            if ((xTaskGetTickCount() - last_log_time) > pdMS_TO_TICKS(30000)) {
                should_log = true;
                last_log_time = xTaskGetTickCount();
            }

            // Log immediately on state changes
            if (current_status.state != prev_status.state) {
                ESP_LOGW(TAG, "TWAI state changed: %d -> %d", prev_status.state, current_status.state);
                should_log = true;
            }

            // Log on significant error increases
            if (current_status.tx_error_count > prev_status.tx_error_count + 10 ||
                current_status.rx_error_count > prev_status.rx_error_count + 10) {
                ESP_LOGW(TAG, "TWAI errors increased significantly");
                should_log = true;
            }

            // Critical states
            if (current_status.state == TWAI_ERROR_BUS_OFF) {
                ESP_LOGE(TAG, "TWAI in critical state: BUS_OFF");
                should_log = true;
            }

            if (should_log) {
                log_can_status();
            }

            // Bus recovery attempt
            if (current_status.state == TWAI_ERROR_BUS_OFF) {
                ESP_LOGW(TAG, "Attempting TWAI bus recovery...");
                twai_node_recover(twai_node);
                vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for recovery
            }

            prev_status = current_status;
        } else {
            ESP_LOGE(TAG, "Failed to get TWAI node info in monitor task: %s", esp_err_to_name(result));
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Check every second
    }
}