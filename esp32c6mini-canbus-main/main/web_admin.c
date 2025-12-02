#include "web_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "cJSON.h"
#include "ota_system.h"
#include <string.h>

static const char *TAG = "WEB_ADMIN";

esp_err_t admin_handler(httpd_req_t *req) {
    const char* admin_html = 
    "<!DOCTYPE html>"
    "<html><head>"
    "<title>ESP32-C6 Admin Panel - OTA Update</title>"
    "<meta name='viewport' content='width=device-width, initial-scale=1'>"
    "<style>"
    "body{font-family:Arial,sans-serif;margin:0;padding:20px;background:#f5f5f5}"
    ".container{max-width:800px;margin:0 auto;background:white;padding:30px;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1)}"
    "h1{color:#333;text-align:center;margin-bottom:30px;border-bottom:2px solid #007bff;padding-bottom:10px}"
    ".section{margin-bottom:30px;padding:20px;border:1px solid #ddd;border-radius:5px}"
    ".section h2{color:#007bff;margin-top:0}"
    ".upload-area{border:2px dashed #007bff;padding:40px;text-align:center;margin:20px 0;border-radius:5px;background:#f8f9fa}"
    ".upload-area.dragover{background:#e3f2fd;border-color:#1976d2}"
    "input[type='file']{margin:20px 0}"
    "button{background:#007bff;color:white;padding:12px 24px;border:none;border-radius:5px;cursor:pointer;font-size:16px;margin:5px}"
    "button:hover{background:#0056b3}"
    "button:disabled{background:#ccc;cursor:not-allowed}"
    ".progress{width:100%;height:20px;background:#f0f0f0;border-radius:10px;margin:20px 0;overflow:hidden}"
    ".progress-bar{height:100%;background:#007bff;width:0%;transition:width 0.3s}"
    ".status{padding:15px;margin:20px 0;border-radius:5px;border-left:4px solid}"
    ".status.info{background:#e3f2fd;border-color:#2196f3;color:#1565c0}"
    ".status.success{background:#e8f5e8;border-color:#4caf50;color:#2e7d32}"
    ".status.error{background:#ffebee;border-color:#f44336;color:#c62828}"
    ".status.warning{background:#fff3e0;border-color:#ff9800;color:#ef6c00}"
    ".info-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:20px}"
    ".info-item{padding:15px;background:#f8f9fa;border-radius:5px;border-left:4px solid #007bff}"
    ".info-item h3{margin:0 0 10px 0;color:#333}"
    ".info-item p{margin:0;color:#666}"
    ".device-list{max-height:200px;overflow-y:auto;border:1px solid #ddd;border-radius:5px;padding:10px}"
    ".device-item{padding:10px;border-bottom:1px solid #eee;display:flex;justify-content:space-between;align-items:center}"
    ".device-item:last-child{border-bottom:none}"
    ".device-status{padding:4px 8px;border-radius:3px;font-size:12px;font-weight:bold}"
    ".device-status.connected{background:#4caf50;color:white}"
    ".device-status.disconnected{background:#f44336;color:white}"
    "</style>"
    "</head><body>"
    
    "<div class='container'>"
    "<h1>🔧 ESP32-C6 Admin Panel</h1>"
    
    "<!-- System Information -->"
    "<div class='section'>"
    "<h2>📊 System Information</h2>"
    "<div class='info-grid' id='systemInfo'>"
    "<div class='info-item'><h3>Firmware Version</h3><p id='firmwareVersion'>Loading...</p></div>"
    "<div class='info-item'><h3>Free Memory</h3><p id='freeMemory'>Loading...</p></div>"
    "<div class='info-item'><h3>Uptime</h3><p id='uptime'>Loading...</p></div>"
    "<div class='info-item'><h3>WiFi Status</h3><p id='wifiStatus'>Loading...</p></div>"
    "</div>"
    "</div>"
    
    "<!-- ESP-NOW Device Management -->"
    "<div class='section'>"
    "<h2>📡 ESP-NOW Devices</h2>"
    "<div class='device-list' id='deviceList'>Loading devices...</div>"
    "<button onclick='scanDevices()'>🔍 Scan for Devices</button>"
    "<button onclick='sendTestMessage()'>📤 Send Test Message</button>"
    "</div>"
    
    "<!-- OTA Update Section -->"
    "<div class='section'>"
    "<h2>🚀 OTA Firmware Update</h2>"
    "<div class='status info'>"
    "<strong>Instructions:</strong><br>"
    "1. Build your firmware using the build script<br>"
    "2. Select the generated .bin file<br>"
    "3. Choose update method (Web or ESP-NOW)<br>"
    "4. Click upload to start the update process"
    "</div>"
    
    "<div class='upload-area' id='uploadArea' ondrop='handleDrop(event)' ondragover='handleDragOver(event)' ondragleave='handleDragLeave(event)'>"
    "<p>📁 Drag and drop firmware file here or click to select</p>"
    "<input type='file' id='firmwareFile' accept='.bin' onchange='handleFileSelect(event)' style='display:none'>"
    "<button onclick='document.getElementById(\"firmwareFile\").click()'>Select Firmware File</button>"
    "</div>"
    
    "<div id='fileInfo' style='display:none'>"
    "<h3>Selected File:</h3>"
    "<p><strong>Name:</strong> <span id='fileName'></span></p>"
    "<p><strong>Size:</strong> <span id='fileSize'></span></p>"
    "<p><strong>Type:</strong> <span id='fileType'></span></p>"
    "</div>"
    
    "<div style='margin:20px 0'>"
    "<label><input type='radio' name='updateMethod' value='web' checked> Web Upload (Current Device)</label><br>"
    "<label><input type='radio' name='updateMethod' value='espnow'> ESP-NOW Broadcast (All Devices)</label>"
    "</div>"
    
    "<button id='uploadBtn' onclick='startUpdate()' disabled>🚀 Start Update</button>"
    "<button onclick='abortUpdate()'>❌ Abort Update</button>"
    
    "<div class='progress' id='progressContainer' style='display:none'>"
    "<div class='progress-bar' id='progressBar'></div>"
    "</div>"
    
    "<div id='updateStatus'></div>"
    "</div>"
    
    "<!-- System Controls -->"
    "<div class='section'>"
    "<h2>⚙️ System Controls</h2>"
    "<button onclick='restartSystem()'>🔄 Restart System</button>"
    "<button onclick='factoryReset()'>🏭 Factory Reset</button>"
    "<button onclick='emergencyStop()'>🛑 Emergency Stop</button>"
    "<button onclick='exportLogs()'>📋 Export Logs</button>"
    "</div>"
    "</div>"
    
    "<script>"
    "let selectedFile = null;"
    "let updateInProgress = false;"
    
    "function updateSystemInfo() {"
    "fetch('/api/system/info').then(r=>r.json()).then(data=>{"
    "document.getElementById('firmwareVersion').textContent = data.version || 'Unknown';"
    "document.getElementById('freeMemory').textContent = (data.free_memory || 0) + ' bytes';"
    "document.getElementById('uptime').textContent = (data.uptime || 0) + ' seconds';"
    "document.getElementById('wifiStatus').textContent = data.wifi_connected ? 'Connected' : 'Disconnected';"
    "}).catch(e=>console.error('Failed to load system info:', e));}"
    
    "function updateDeviceList() {"
    "fetch('/api/espnow/devices').then(r=>r.json()).then(devices=>{"
    "const list = document.getElementById('deviceList');"
    "list.innerHTML = devices.map(d=>"
    "`<div class='device-item'><span>${d.name} (${d.mac})</span><span class='device-status ${d.connected?'connected':'disconnected'}'>${d.connected?'Connected':'Disconnected'}</span></div>`"
    ").join('') || '<p>No devices found</p>';"
    "}).catch(e=>console.error('Failed to load devices:', e));}"
    
    "function handleDragOver(e) {"
    "e.preventDefault();"
    "document.getElementById('uploadArea').classList.add('dragover');}"
    
    "function handleDragLeave(e) {"
    "document.getElementById('uploadArea').classList.remove('dragover');}"
    
    "function handleDrop(e) {"
    "e.preventDefault();"
    "document.getElementById('uploadArea').classList.remove('dragover');"
    "const files = e.dataTransfer.files;"
    "if (files.length > 0) handleFile(files[0]);}"
    
    "function handleFileSelect(e) {"
    "if (e.target.files.length > 0) handleFile(e.target.files[0]);}"
    
    "function handleFile(file) {"
    "if (!file.name.endsWith('.bin')) {"
    "showStatus('Please select a .bin firmware file', 'error');"
    "return;}"
    "selectedFile = file;"
    "document.getElementById('fileName').textContent = file.name;"
    "document.getElementById('fileSize').textContent = formatBytes(file.size);"
    "document.getElementById('fileType').textContent = file.type || 'application/octet-stream';"
    "document.getElementById('fileInfo').style.display = 'block';"
    "document.getElementById('uploadBtn').disabled = false;"
    "showStatus('Firmware file selected successfully', 'success');}"
    
    "function formatBytes(bytes) {"
    "if (bytes === 0) return '0 Bytes';"
    "const k = 1024;"
    "const sizes = ['Bytes', 'KB', 'MB', 'GB'];"
    "const i = Math.floor(Math.log(bytes) / Math.log(k));"
    "return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + ' ' + sizes[i];}"
    
    "function showStatus(message, type) {"
    "const statusDiv = document.getElementById('updateStatus');"
    "statusDiv.className = 'status ' + type;"
    "statusDiv.innerHTML = message;}"
    
    "function updateProgress(percent) {"
    "document.getElementById('progressContainer').style.display = 'block';"
    "document.getElementById('progressBar').style.width = percent + '%';}"
    
    "function startUpdate() {"
    "if (!selectedFile) return;"
    "updateInProgress = true;"
    "const method = document.querySelector('input[name=\"updateMethod\"]:checked').value;"
    "showStatus('Starting firmware update...', 'info');"
    "updateProgress(0);"
    
    "const formData = new FormData();"
    "formData.append('firmware', selectedFile);"
    "formData.append('method', method);"
    
    "fetch('/api/ota/upload', {method:'POST', body:formData})"
    ".then(response => response.json())"
    ".then(result => {"
    "if (result.success) {"
    "showStatus('Firmware update completed successfully!', 'success');"
    "updateProgress(100);"
    "} else {"
    "showStatus('Update failed: ' + result.error, 'error');"
    "}"
    "updateInProgress = false;"
    "})"
    ".catch(error => {"
    "showStatus('Update failed: ' + error.message, 'error');"
    "updateInProgress = false;"
    "});}"
    
    "function abortUpdate() {"
    "if (updateInProgress) {"
    "fetch('/api/ota/abort', {method:'POST'})"
    ".then(() => showStatus('Update aborted', 'warning'))"
    ".catch(e => console.error('Failed to abort:', e));"
    "updateInProgress = false;"
    "}}"
    
    "function restartSystem() {"
    "if (confirm('Are you sure you want to restart the system?')) {"
    "fetch('/api/system/restart', {method:'POST'})"
    ".then(() => showStatus('System restarting...', 'info'))"
    ".catch(e => console.error('Restart failed:', e));"
    "}}"
    
    "function factoryReset() {"
    "if (confirm('WARNING: This will reset all settings to factory defaults. Continue?')) {"
    "fetch('/api/system/factory-reset', {method:'POST'})"
    ".then(() => showStatus('Factory reset initiated...', 'warning'))"
    ".catch(e => console.error('Factory reset failed:', e));"
    "}}"
    
    "function emergencyStop() {"
    "fetch('/api/safety', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({action:'emergency_stop'})})"
    ".then(() => showStatus('Emergency stop activated!', 'error'))"
    ".catch(e => console.error('Emergency stop failed:', e));}"
    
    "function scanDevices() {"
    "showStatus('Scanning for ESP-NOW devices...', 'info');"
    "updateDeviceList();}"
    
    "function sendTestMessage() {"
    "fetch('/api/espnow/test', {method:'POST'})"
    ".then(() => showStatus('Test message sent to all devices', 'success'))"
    ".catch(e => console.error('Test message failed:', e));}"
    
    "function exportLogs() {"
    "window.open('/api/system/logs', '_blank');}"
    
    "// Initialize page"
    "document.addEventListener('DOMContentLoaded', function() {"
    "updateSystemInfo();"
    "updateDeviceList();"
    "setInterval(updateSystemInfo, 30000);"
    "setInterval(updateDeviceList, 10000);"
    "});"
    "</script>"
    "</body></html>";
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, admin_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t api_ota_upload_handler(httpd_req_t *req) {
    if (req->method == HTTP_POST) {
        ESP_LOGI(TAG, "OTA upload started");
        
        char buf[512];
        size_t total_received = 0;
        size_t content_length = req->content_len;
        
        // Initialize OTA
        esp_err_t err = ota_begin(content_length, 0, "web_upload");
        if (err != ESP_OK) {
            cJSON *response = cJSON_CreateObject();
            cJSON_AddBoolToObject(response, "success", false);
            cJSON_AddStringToObject(response, "error", "Failed to start OTA");
            
            char *response_str = cJSON_Print(response);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_send(req, response_str, HTTPD_RESP_USE_STRLEN);
            
            free(response_str);
            cJSON_Delete(response);
            return ESP_FAIL;
        }
        
        // Receive and write firmware data
        uint8_t chunk_id = 0;
        while (total_received < content_length) {
            int received = httpd_req_recv(req, buf, MIN(sizeof(buf), content_length - total_received));
            if (received <= 0) {
                ota_abort();
                ESP_LOGE(TAG, "Failed to receive OTA data");
                return ESP_FAIL;
            }
            
            err = ota_write_data(chunk_id++, (uint8_t*)buf, received);
            if (err != ESP_OK) {
                ota_abort();
                ESP_LOGE(TAG, "Failed to write OTA data");
                return ESP_FAIL;
            }
            
            total_received += received;
            ESP_LOGI(TAG, "OTA progress: %zu/%zu bytes", total_received, content_length);
        }
        
        // Finish OTA
        err = ota_end();
        
        cJSON *response = cJSON_CreateObject();
        cJSON_AddBoolToObject(response, "success", err == ESP_OK);
        if (err != ESP_OK) {
            cJSON_AddStringToObject(response, "error", "Failed to complete OTA");
        } else {
            cJSON_AddStringToObject(response, "message", "OTA completed successfully, restarting...");
        }
        
        char *response_str = cJSON_Print(response);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, response_str, HTTPD_RESP_USE_STRLEN);
        
        free(response_str);
        cJSON_Delete(response);
        
        return err;
    }
    
    return ESP_OK;
}