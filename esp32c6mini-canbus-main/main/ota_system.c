#include "ota_system.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "OTA_SYSTEM";

ota_state_t ota_state = {0};

esp_err_t ota_begin(uint32_t total_size, uint32_t crc32, const char* version) {
    ESP_LOGI(TAG, "OTA Begin: size=%lu, crc=0x%lx, version=%s", total_size, crc32, version);
    
    if (ota_state.ota_in_progress) {
        ESP_LOGW(TAG, "OTA already in progress, aborting previous session");
        esp_ota_abort(ota_state.ota_handle);
    }
    
    ota_state.update_partition = esp_ota_get_next_update_partition(NULL);
    if (ota_state.update_partition == NULL) {
        ESP_LOGE(TAG, "Failed to get OTA update partition");
        return ESP_FAIL;
    }
    
    esp_err_t err = esp_ota_begin(ota_state.update_partition, total_size, &ota_state.ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        return err;
    }
    
    ota_state.ota_in_progress = true;
    ota_state.total_size = total_size;
    ota_state.received_size = 0;
    ota_state.expected_chunk = 0;
    ota_state.firmware_crc = crc32;
    
    ESP_LOGI(TAG, "OTA session started successfully");
    return ESP_OK;
}

esp_err_t ota_write_data(uint8_t chunk_id, const uint8_t* data, uint8_t data_len) {
    if (!ota_state.ota_in_progress) {
        ESP_LOGE(TAG, "OTA not in progress");
        return ESP_FAIL;
    }
    
    if (chunk_id != ota_state.expected_chunk) {
        ESP_LOGE(TAG, "Unexpected chunk ID: expected %d, got %d", ota_state.expected_chunk, chunk_id);
        return ESP_FAIL;
    }
    
    esp_err_t err = esp_ota_write(ota_state.ota_handle, data, data_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
        return err;
    }
    
    ota_state.received_size += data_len;
    ota_state.expected_chunk++;
    
    ESP_LOGI(TAG, "OTA chunk %d written, progress: %zu/%zu bytes (%.1f%%)", 
            chunk_id, ota_state.received_size, ota_state.total_size,
            (float)ota_state.received_size * 100.0f / ota_state.total_size);
    
    return ESP_OK;
}

esp_err_t ota_end(void) {
    if (!ota_state.ota_in_progress) {
        ESP_LOGE(TAG, "OTA not in progress");
        return ESP_FAIL;
    }
    
    if (ota_state.received_size != ota_state.total_size) {
        ESP_LOGE(TAG, "OTA size mismatch: expected %zu, received %zu", 
                ota_state.total_size, ota_state.received_size);
        esp_ota_abort(ota_state.ota_handle);
        ota_state.ota_in_progress = false;
        return ESP_FAIL;
    }
    
    esp_err_t err = esp_ota_end(ota_state.ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        ota_state.ota_in_progress = false;
        return err;
    }
    
    err = esp_ota_set_boot_partition(ota_state.update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        ota_state.ota_in_progress = false;
        return err;
    }
    
    ota_state.ota_in_progress = false;
    ESP_LOGI(TAG, "OTA completed successfully. Restarting...");
    
    // Restart after a short delay
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
    
    return ESP_OK;
}

void ota_abort(void) {
    if (ota_state.ota_in_progress) {
        esp_ota_abort(ota_state.ota_handle);
        ota_state.ota_in_progress = false;
        ESP_LOGI(TAG, "OTA session aborted");
    }
}

void handle_ota_message(const espnow_data_t* data) {
    switch (data->type) {
        case ESPNOW_DATA_TYPE_OTA_BEGIN: {
            if (data->len >= sizeof(ota_begin_data_t)) {
                ota_begin_data_t* begin_data = (ota_begin_data_t*)data->data;
                ota_begin(begin_data->total_size, begin_data->crc32, begin_data->version);
            }
            break;
        }
        case ESPNOW_DATA_TYPE_OTA_DATA: {
            if (data->len >= sizeof(ota_data_chunk_t)) {
                ota_data_chunk_t* chunk_data = (ota_data_chunk_t*)data->data;
                ota_write_data(chunk_data->chunk_id, chunk_data->data, chunk_data->data_len);
            }
            break;
        }
        case ESPNOW_DATA_TYPE_OTA_END: {
            ota_end();
            break;
        }
        case ESPNOW_DATA_TYPE_OTA_ABORT: {
            ota_abort();
            break;
        }
        default:
            ESP_LOGW(TAG, "Unknown OTA message type: %d", data->type);
            break;
    }
}