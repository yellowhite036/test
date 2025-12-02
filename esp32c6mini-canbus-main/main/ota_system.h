#ifndef OTA_SYSTEM_H
#define OTA_SYSTEM_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_ota_ops.h"

// OTA System
typedef struct {
    bool ota_in_progress;
    esp_ota_handle_t ota_handle;
    const esp_partition_t *update_partition;
    size_t total_size;
    size_t received_size;
    uint8_t expected_chunk;
    uint32_t firmware_crc;
} ota_state_t;

extern ota_state_t ota_state;

// ESP-NOW data types for OTA
typedef enum {
    ESPNOW_DATA_TYPE_MOTOR_DATA = 0,
    ESPNOW_DATA_TYPE_OTA_BEGIN = 1,
    ESPNOW_DATA_TYPE_OTA_DATA = 2,
    ESPNOW_DATA_TYPE_OTA_END = 3,
    ESPNOW_DATA_TYPE_OTA_ABORT = 4,
    ESPNOW_DATA_TYPE_EMERGENCY_STOP = 5,
} espnow_data_type_t;

typedef struct {
    uint8_t type;
    uint8_t len;
    uint8_t data[240]; // ESP-NOW max payload is ~250 bytes
} espnow_data_t;

typedef struct {
    uint32_t total_size;
    uint32_t crc32;
    char version[32];
} ota_begin_data_t;

typedef struct {
    uint8_t chunk_id;
    uint8_t data_len;
    uint8_t data[230];
} ota_data_chunk_t;

// Function declarations
esp_err_t ota_begin(uint32_t total_size, uint32_t crc32, const char* version);
esp_err_t ota_write_data(uint8_t chunk_id, const uint8_t* data, uint8_t data_len);
esp_err_t ota_end(void);
void ota_abort(void);
void handle_ota_message(const espnow_data_t* data);

#endif // OTA_SYSTEM_H