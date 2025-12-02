#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "esp_http_server.h"
#include "esp_err.h"

extern httpd_handle_t server;

// Function declarations
esp_err_t start_webserver(void);
esp_err_t stop_webserver(void);

// Handler functions
esp_err_t index_handler(httpd_req_t *req);
esp_err_t admin_handler(httpd_req_t *req);
esp_err_t api_motors_handler(httpd_req_t *req);
esp_err_t api_motor_control_handler(httpd_req_t *req);
esp_err_t api_config_handler(httpd_req_t *req);
esp_err_t api_safety_handler(httpd_req_t *req);
esp_err_t api_monitoring_handler(httpd_req_t *req);
esp_err_t api_motor_test_handler(httpd_req_t *req);
esp_err_t api_motor_status_handler(httpd_req_t *req);
esp_err_t api_espnow_devices_handler(httpd_req_t *req);
esp_err_t api_ota_upload_handler(httpd_req_t *req);
esp_err_t api_motor_management_handler(httpd_req_t *req);
esp_err_t ota_page_handler(httpd_req_t *req);
esp_err_t api_system_info_handler(httpd_req_t *req);

#endif // WEB_SERVER_H