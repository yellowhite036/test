#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"

// Local modules
#include "motor_control.h"
#include "safety_system.h"
#include "ota_system.h"
#include "web_server.h"

#define GPIO8 8
#define ESPNOW_MAXDELAY 512

// RGB LED pins
#define RGB_LED_RED_GPIO    21
#define RGB_LED_GREEN_GPIO  22
#define RGB_LED_BLUE_GPIO   23

// LEDC (PWM) configuration for RGB LED
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT // 8-bit resolution (0-255)
#define LEDC_FREQUENCY          5000             // 5kHz frequency

#define LEDC_CHANNEL_RED        LEDC_CHANNEL_0
#define LEDC_CHANNEL_GREEN      LEDC_CHANNEL_1
#define LEDC_CHANNEL_BLUE       LEDC_CHANNEL_2

// RGB LED brightness control (0-100%)
#define RGB_LED_DEFAULT_BRIGHTNESS  1  // Default to 1% brightness

#define RGBLED_255 255

static const char *TAG = "MOTOR_WEBSERVER";

static QueueHandle_t espnow_queue;

// Global RGB LED brightness (0-100%)
static uint8_t rgb_led_brightness = RGB_LED_DEFAULT_BRIGHTNESS;

// WiFi and ESP-NOW
static void configGPIO(int pin) {
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
}

static void led_task(void *pvParameters) {
    while (1) {
        gpio_set_level(GPIO8, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO8, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// Initialize LEDC (PWM) for RGB LED
static void rgb_led_init(void) {
    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configure LEDC channel for RED
    ledc_channel_config_t ledc_channel_red = {
        .gpio_num       = RGB_LED_RED_GPIO,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_RED,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_red));

    // Configure LEDC channel for GREEN
    ledc_channel_config_t ledc_channel_green = {
        .gpio_num       = RGB_LED_GREEN_GPIO,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_GREEN,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_green));

    // Configure LEDC channel for BLUE
    ledc_channel_config_t ledc_channel_blue = {
        .gpio_num       = RGB_LED_BLUE_GPIO,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_BLUE,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_blue));

    ESP_LOGI(TAG, "RGB LED PWM initialized on GPIO %d, %d, %d",
             RGB_LED_RED_GPIO, RGB_LED_GREEN_GPIO, RGB_LED_BLUE_GPIO);
}

// Set RGB LED color (values 0-255 for each channel)
// Brightness is automatically applied based on rgb_led_brightness variable
// Note: RGB LED is active-low (common anode), so PWM values are inverted
static void rgb_led_set_color(uint8_t red, uint8_t green, uint8_t blue) {
    // Apply brightness scaling (0-100%)
    uint8_t scaled_red = (red * rgb_led_brightness) / 100;
    uint8_t scaled_green = (green * rgb_led_brightness) / 100;
    uint8_t scaled_blue = (blue * rgb_led_brightness) / 100;

    // Invert for active-low LED (255 = off, 0 = full on)
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RED, 255 - scaled_red);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RED);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_GREEN, 255 - scaled_green);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_GREEN);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BLUE, 255 - scaled_blue);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BLUE);
}

// Set RGB LED brightness (0-100%)
static void rgb_led_set_brightness(uint8_t brightness) {
    if (brightness > 100) {
        brightness = 100;
    }
    rgb_led_brightness = brightness;
    ESP_LOGI(TAG, "RGB LED brightness set to %d%%", brightness);
}

// RGB LED task with color cycling effects
static void rgb_led_task(void *pvParameters) {
    ESP_LOGI(TAG, "RGB LED task started with %d%% brightness", rgb_led_brightness);

    uint8_t mode = 0;
    uint8_t brightness = 0;

    while (1) {
        switch (mode) {
            case 0: // Red fade in/out
                for (brightness = 0; brightness < RGBLED_255; brightness++) {
                    rgb_led_set_color(brightness, 0, 0);
                    vTaskDelay(pdMS_TO_TICKS(5));
                }
                for (brightness = RGBLED_255; brightness > 0; brightness--) {
                    rgb_led_set_color(brightness, 0, 0);
                    vTaskDelay(pdMS_TO_TICKS(5));
                }
                break;

            case 1: // Green fade in/out
                for (brightness = 0; brightness < RGBLED_255; brightness++) {
                    rgb_led_set_color(0, brightness, 0);
                    vTaskDelay(pdMS_TO_TICKS(5));
                }
                for (brightness = RGBLED_255; brightness > 0; brightness--) {
                    rgb_led_set_color(0, brightness, 0);
                    vTaskDelay(pdMS_TO_TICKS(5));
                }
                break;

            case 2: // Blue fade in/out
                for (brightness = 0; brightness < RGBLED_255; brightness++) {
                    rgb_led_set_color(0, 0, brightness);
                    vTaskDelay(pdMS_TO_TICKS(5));
                }
                for (brightness = RGBLED_255; brightness > 0; brightness--) {
                    rgb_led_set_color(0, 0, brightness);
                    vTaskDelay(pdMS_TO_TICKS(5));
                }
                break;

            case 3: // Rainbow cycle
                for (int hue = 0; hue < 360; hue++) {
                    // Simple HSV to RGB conversion for rainbow effect
                    float h = hue / 60.0;
                    float c = RGBLED_255;
                    float x = c * (1 - fabs(fmod(h, 2) - 1));

                    uint8_t r = 0, g = 0, b = 0;
                    if (hue < 60) { r = c; g = x; b = 0; }
                    else if (hue < 120) { r = x; g = c; b = 0; }
                    else if (hue < 180) { r = 0; g = c; b = x; }
                    else if (hue < 240) { r = 0; g = x; b = c; }
                    else if (hue < 300) { r = x; g = 0; b = c; }
                    else { r = c; g = 0; b = x; }

                    rgb_led_set_color(r, g, b);
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                break;
        }

        // Cycle through modes
        mode = (mode + 1) % 4;
        vTaskDelay(pdMS_TO_TICKS(500)); // Pause between modes
    }
}

static void motor_test_task(void *pvParameters) {
    ESP_LOGI(TAG, "Motor test task started");

    // Enable motor A power
    gpio_set_level(GPIO_NUM_14, 1);
    gpio_set_level(GPIO_NUM_15, 0);
    gpio_set_level(GPIO_NUM_18, 0);
    gpio_set_level(GPIO_NUM_19, 1);

    // Enable motor B power
    gpio_set_level(GPIO_NUM_2, 1);
    gpio_set_level(GPIO_NUM_3, 0);
    gpio_set_level(GPIO_NUM_4, 0);
    gpio_set_level(GPIO_NUM_5, 1);

    ESP_LOGI(TAG, "Motor A and Motor B power enabled");

    while (1) {
        // Forward PWM sequence - 10 pulses
        for(int i=0; i<10; i++) {
            // Motor A forward
            gpio_set_level(GPIO_NUM_14, 1);
            gpio_set_level(GPIO_NUM_15, 0);
            gpio_set_level(GPIO_NUM_18, 0);
            gpio_set_level(GPIO_NUM_19, 1);
            // Motor B forward
            gpio_set_level(GPIO_NUM_2, 1);
            gpio_set_level(GPIO_NUM_3, 0);
            gpio_set_level(GPIO_NUM_4, 0);
            gpio_set_level(GPIO_NUM_5, 1);
            vTaskDelay(pdMS_TO_TICKS(100)); // Run 100 ms forward

            // Coast both motors
            gpio_set_level(GPIO_NUM_14, 0);
            gpio_set_level(GPIO_NUM_15, 0);
            gpio_set_level(GPIO_NUM_18, 0);
            gpio_set_level(GPIO_NUM_19, 0);
            gpio_set_level(GPIO_NUM_2, 0);
            gpio_set_level(GPIO_NUM_3, 0);
            gpio_set_level(GPIO_NUM_4, 0);
            gpio_set_level(GPIO_NUM_5, 0);
            vTaskDelay(pdMS_TO_TICKS(30)); // Rest for 30ms

            // Charge hi-side bootstrap capacitors
            gpio_set_level(GPIO_NUM_14, 0);
            gpio_set_level(GPIO_NUM_15, 1);
            gpio_set_level(GPIO_NUM_18, 1);
            gpio_set_level(GPIO_NUM_19, 0);
            gpio_set_level(GPIO_NUM_2, 0);
            gpio_set_level(GPIO_NUM_3, 1);
            gpio_set_level(GPIO_NUM_4, 1);
            gpio_set_level(GPIO_NUM_5, 0);
            vTaskDelay(pdMS_TO_TICKS(1)); // Charge hi-side bootstrap cap

            // Coast before next cycle
            gpio_set_level(GPIO_NUM_14, 0);
            gpio_set_level(GPIO_NUM_15, 0);
            gpio_set_level(GPIO_NUM_18, 0);
            gpio_set_level(GPIO_NUM_19, 0);
            gpio_set_level(GPIO_NUM_2, 0);
            gpio_set_level(GPIO_NUM_3, 0);
            gpio_set_level(GPIO_NUM_4, 0);
            gpio_set_level(GPIO_NUM_5, 0);
            vTaskDelay(pdMS_TO_TICKS(1)); // Slow down before continuing
        }

        // Brake both motors for 5 seconds
        gpio_set_level(GPIO_NUM_14, 0);
        gpio_set_level(GPIO_NUM_15, 1);
        gpio_set_level(GPIO_NUM_18, 0);
        gpio_set_level(GPIO_NUM_19, 1);
        gpio_set_level(GPIO_NUM_2, 0);
        gpio_set_level(GPIO_NUM_3, 1);
        gpio_set_level(GPIO_NUM_4, 0);
        gpio_set_level(GPIO_NUM_5, 1);
        vTaskDelay(pdMS_TO_TICKS(5000));

        // Backward PWM sequence - 10 pulses
        for(int i=0; i<10; i++) {
            // Motor A backward
            gpio_set_level(GPIO_NUM_14, 0);
            gpio_set_level(GPIO_NUM_15, 1);
            gpio_set_level(GPIO_NUM_18, 1);
            gpio_set_level(GPIO_NUM_19, 0);
            // Motor B backward
            gpio_set_level(GPIO_NUM_2, 0);
            gpio_set_level(GPIO_NUM_3, 1);
            gpio_set_level(GPIO_NUM_4, 1);
            gpio_set_level(GPIO_NUM_5, 0);
            vTaskDelay(pdMS_TO_TICKS(100)); // Run 100 ms backward

            // Coast both motors
            gpio_set_level(GPIO_NUM_14, 0);
            gpio_set_level(GPIO_NUM_15, 0);
            gpio_set_level(GPIO_NUM_18, 0);
            gpio_set_level(GPIO_NUM_19, 0);
            gpio_set_level(GPIO_NUM_2, 0);
            gpio_set_level(GPIO_NUM_3, 0);
            gpio_set_level(GPIO_NUM_4, 0);
            gpio_set_level(GPIO_NUM_5, 0);
            vTaskDelay(pdMS_TO_TICKS(30)); // Rest for 30ms

            // Charge hi-side bootstrap capacitors
            gpio_set_level(GPIO_NUM_14, 1);
            gpio_set_level(GPIO_NUM_15, 0);
            gpio_set_level(GPIO_NUM_18, 0);
            gpio_set_level(GPIO_NUM_19, 1);
            gpio_set_level(GPIO_NUM_2, 1);
            gpio_set_level(GPIO_NUM_3, 0);
            gpio_set_level(GPIO_NUM_4, 0);
            gpio_set_level(GPIO_NUM_5, 1);
            vTaskDelay(pdMS_TO_TICKS(1)); // Charge hi-side bootstrap cap

            // Coast before next cycle
            gpio_set_level(GPIO_NUM_14, 0);
            gpio_set_level(GPIO_NUM_15, 0);
            gpio_set_level(GPIO_NUM_18, 0);
            gpio_set_level(GPIO_NUM_19, 0);
            gpio_set_level(GPIO_NUM_2, 0);
            gpio_set_level(GPIO_NUM_3, 0);
            gpio_set_level(GPIO_NUM_4, 0);
            gpio_set_level(GPIO_NUM_5, 0);
            vTaskDelay(pdMS_TO_TICKS(1)); // Slow down before continuing
        }

        // Brake both motors for 5 seconds
        gpio_set_level(GPIO_NUM_14, 0);
        gpio_set_level(GPIO_NUM_15, 1);
        gpio_set_level(GPIO_NUM_18, 0);
        gpio_set_level(GPIO_NUM_19, 1);
        gpio_set_level(GPIO_NUM_2, 0);
        gpio_set_level(GPIO_NUM_3, 1);
        gpio_set_level(GPIO_NUM_4, 0);
        gpio_set_level(GPIO_NUM_5, 1);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGI(TAG, "ESP-NOW send success");
    } else {
        ESP_LOGE(TAG, "ESP-NOW send failed");
    }
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    ESP_LOGI(TAG, "ESP-NOW data received from "MACSTR", len=%d", MAC2STR(recv_info->src_addr), len);
    
    if (len >= sizeof(espnow_data_t)) {
        espnow_data_t* espnow_data = (espnow_data_t*)data;
        
        // Handle OTA messages
        if (espnow_data->type >= ESPNOW_DATA_TYPE_OTA_BEGIN && 
            espnow_data->type <= ESPNOW_DATA_TYPE_OTA_ABORT) {
            handle_ota_message(espnow_data);
        } else if (espnow_data->type == ESPNOW_DATA_TYPE_EMERGENCY_STOP) {
            emergency_stop_all_motors();
        }
    }
    
    xQueueSend(espnow_queue, data, 0);
}

static void espnow_init(void) {
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    
    espnow_queue = xQueueCreate(6, 250);
    if (espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create queue fail");
    }
    
    ESP_LOGI(TAG, "ESP-NOW initialized");
}

static void espnow_add_peer(uint8_t *peer_addr) {
    esp_now_peer_info_t peer_info = {};
    peer_info.channel = 1;
    peer_info.ifidx = WIFI_IF_STA;
    peer_info.encrypt = false;
    memcpy(peer_info.peer_addr, peer_addr, ESP_NOW_ETH_ALEN);

    esp_now_add_peer(&peer_info);
}

static void espnow_task(void *pvParameters) {
    uint8_t recv_data[250];
    while (1) {
        if (xQueueReceive(espnow_queue, recv_data, 100 / portTICK_PERIOD_MS) == pdTRUE) {
            ESP_LOGI(TAG, "ESP-NOW message processed");
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // Ensure CPU yield
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    static int retry_count = 0;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retry_count < 5) {
            ESP_LOGI(TAG, "WiFi disconnected, retry %d...", retry_count);
            esp_wifi_connect();
            retry_count++;
            // REMOVED blocking vTaskDelay() - event handlers must not block!
        } else {
            ESP_LOGE(TAG, "Max retries reached, stopping WiFi");
            esp_wifi_stop();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "WiFi connected! IP: " IPSTR, IP2STR(&event->ip_info.ip));

        retry_count = 0; // Reset on success
    }
}

static void wifi_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
 
    wifi_config_t wifi_config = {
        /**.sta = {
            .ssid = "076519399",
            .password = "076519399",
        },
        */

        // .sta = {
        //     .ssid = "Kaomu",
        //     .password = "22846559",
        // }

        .sta = {
            .ssid = "TIGO5G1",
            .password = "cafebabe",
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,  // Explicitly set WPA2
            .pmf_cfg = {
                .capable = true,
                .required = false  // Try with PMF capable but not required
            },
        }
        // .sta = {
        //     .ssid = "TIGO5G2.1",
        //     .password = "abcdefab",
        //     .threshold.authmode = WIFI_AUTH_WPA2_PSK,  // Explicitly set WPA2
        //     .pmf_cfg = {
        //         .capable = true,
        //         .required = false  // Try with PMF capable but not required
        //     },
        // },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    // Set additional compatibility options
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));  // Disable power save for stability

    // Enable WiFi 6 (802.11ax) - ESP32-C6 supports WiFi 6
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX));

    ESP_LOGI(TAG, "WiFi set to 802.11ax mode (WiFi 6 enabled)");

    // Set bandwidth to 20MHz (can also use WIFI_BW_HT40 for better WiFi 6 performance if supported by your router)
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20));

    ESP_ERROR_CHECK(esp_wifi_start());

    // IMPORTANT: Set TX power AFTER esp_wifi_start() - this was causing watchdog issues!
    // Reduce TX power to help with potential hardware/power supply issues
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(44)); // 11 dBm instead of default 20 dBm
    ESP_LOGI(TAG, "WiFi TX power reduced to 11 dBm");

    ESP_LOGI(TAG, "WiFi initialized");
}

void app_main(void) {
    printf("ESP32-C6 Motor Control WebServer (Modular)\n");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    configGPIO(GPIO8);
    // Motor A GPIOs
    configGPIO(GPIO_NUM_14);
    configGPIO(GPIO_NUM_15);
    configGPIO(GPIO_NUM_18);
    configGPIO(GPIO_NUM_19);
    // Motor B GPIOs
    configGPIO(GPIO_NUM_2);
    configGPIO(GPIO_NUM_3);
    configGPIO(GPIO_NUM_4);
    configGPIO(GPIO_NUM_5);

    // Initialize RGB LED PWM
    rgb_led_init();

    // Reconfigure watchdog for longer timeout during WiFi initialization
    // WiFi scanning can take 5+ seconds, which exceeds default watchdog timeout
    ESP_LOGI(TAG, "Reconfiguring task watchdog for WiFi init...");
    ESP_ERROR_CHECK(esp_task_wdt_deinit());
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 10000,  // 10 seconds timeout
        .idle_core_mask = (1 << CONFIG_FREERTOS_NUMBER_OF_CORES) - 1,
        .trigger_panic = true,
    };
    ESP_ERROR_CHECK(esp_task_wdt_init(&twdt_config));
    // Note: IDLE task is automatically subscribed by esp_task_wdt_init() when idle_core_mask is set

    wifi_init();
    espnow_init();

    // Restore normal watchdog timeout after WiFi init completes
    ESP_LOGI(TAG, "Restoring task watchdog to normal timeout...");
    ESP_ERROR_CHECK(esp_task_wdt_deinit());
    twdt_config.timeout_ms = 5000;  // 5 seconds is default
    ESP_ERROR_CHECK(esp_task_wdt_init(&twdt_config));
    // Note: IDLE task is automatically subscribed again
    ESP_ERROR_CHECK(init_can());
    
    // Initial CAN status check
    ESP_LOGI(TAG, "CAN bus initialized, checking status...");
    vTaskDelay(pdMS_TO_TICKS(1000)); // Let CAN settle
    log_can_status();
    
    // Load motor configuration
    load_motor_config();

    // Create tasks
    xTaskCreate(led_task, "led_task", 2048, NULL, 5, NULL);
    xTaskCreate(espnow_task, "espnow_task", 4096, NULL, 4, NULL);
    xTaskCreate(can_rx_task, "can_rx_task", 4096, NULL, 6, NULL);
    xTaskCreate(can_error_monitor_task, "can_error_monitor", 3072, NULL, 4, NULL);
    xTaskCreate(motor_test_task, "motor_test_task", 4096, NULL, 5, NULL);
    xTaskCreate(rgb_led_task, "rgb_led_task", 3072, NULL, 3, NULL);
    
    // Start web server
    start_webserver();

    uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    espnow_add_peer(broadcast_mac);

    // Motor status refresh task
    int refresh_counter = 0;

    while (1) {
        if (refresh_counter % 10 == 0) { // Every 1 second
            for (int i = 0; i < motor_count; i++) {
                //motor_refresh_status(&motors[i]);
            }
        }
        refresh_counter++;
        vTaskDelay(pdMS_TO_TICKS(100)); // Delay to prevent tight loop
    }
}