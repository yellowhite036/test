#include "web_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_app_desc.h"
#include "esp_ota_ops.h"
#include "esp_flash.h"
#include "motor_control.h"
#include "safety_system.h"
#include "ota_system.h"
#include "cJSON.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <string.h>
#include <inttypes.h>

static const char *TAG = "WEB_SERVER";

httpd_handle_t server = NULL;

esp_err_t index_handler(httpd_req_t *req) {
    const char* html_content = 
    "<!DOCTYPE html>"
    "<html><head><title>ESP32-C6 Position Reader - Robot Arm Wireless Control</title>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<style>"
    "body{font-family:Arial,sans-serif;margin:10px;background:#f5f5f5;font-size:14px}"
    ".container{max-width:1400px;margin:0 auto}"
    ".header{background:#2c3e50;color:white;padding:15px;border-radius:8px;margin-bottom:15px}"
    ".header h1{margin:0;font-size:20px}"
    ".header p{margin:5px 0 0;opacity:0.8}"
    ".emergency-stop{background:#e74c3c;color:white;border:none;padding:12px 25px;font-size:16px;font-weight:bold;border-radius:6px;cursor:pointer;width:100%;margin-bottom:15px}"
    ".status-bar{background:#34495e;color:white;padding:8px 12px;border-radius:4px;margin-bottom:15px;font-family:monospace;font-size:12px}"
    ".tab-container{display:flex;margin-bottom:15px;flex-wrap:wrap}"
    ".tab{background:#ecf0f1;border:none;padding:8px 16px;cursor:pointer;border-radius:6px 6px 0 0;margin-right:2px;font-size:12px;margin-bottom:2px}"
    ".tab.active{background:#3498db;color:white}"
    ".tab-content{background:white;padding:15px;border-radius:0 6px 6px 6px;box-shadow:0 2px 8px rgba(0,0,0,0.1);display:none}"
    ".tab-content.active{display:block}"
    ".section-grid{display:grid;grid-template-columns:1fr 1fr;gap:15px;margin-bottom:15px}"
    ".section{border:1px solid #ddd;border-radius:6px;padding:12px;background:white}"
    ".section h4{margin:0 0 10px;font-size:14px;color:#2c3e50;border-bottom:1px solid #eee;padding-bottom:5px}"
    ".device-list{border:1px solid #ddd;border-radius:4px;max-height:120px;overflow-y:auto;margin-bottom:10px}"
    ".device-item{padding:8px;border-bottom:1px solid #eee;font-size:12px;cursor:pointer}"
    ".device-item:hover{background:#f0f0f0}"
    ".device-item.selected{background:#3498db;color:white}"
    ".form-row{display:flex;gap:10px;margin-bottom:8px;align-items:center;flex-wrap:wrap}"
    ".form-row label{min-width:80px;font-size:12px}"
    ".form-row input,.form-row select{padding:4px 6px;border:1px solid #ddd;border-radius:3px;font-size:12px}"
    ".form-row input[type='number']{width:80px}"
    ".form-row select{width:120px}"
    ".btn{background:#3498db;color:white;border:none;padding:6px 12px;border-radius:4px;cursor:pointer;font-size:11px;margin:2px}"
    ".btn:hover{background:#2980b9}"
    ".btn-sm{padding:4px 8px;font-size:10px}"
    ".btn-danger{background:#e74c3c}.btn-danger:hover{background:#c0392b}"
    ".btn-success{background:#27ae60}.btn-success:hover{background:#229954}"
    ".btn-warning{background:#f39c12}.btn-warning:hover{background:#e67e22}"
    ".motor-table{width:100%;border-collapse:collapse;font-size:11px;margin-bottom:10px}"
    ".motor-table th,.motor-table td{padding:6px;border:1px solid #ddd;text-align:center}"
    ".motor-table th{background:#f8f9fa;font-weight:bold}"
    ".motor-table tbody tr:nth-child(even){background:#f9f9f9}"
    ".status-indicator{width:8px;height:8px;border-radius:50%;display:inline-block;margin-right:5px}"
    ".status-connected{background:#27ae60}"
    ".status-disconnected{background:#e74c3c}"
    ".status-paired{background:#f39c12}"
    ".mapping-item{display:flex;justify-content:space-between;align-items:center;padding:8px;border:1px solid #ddd;border-radius:4px;margin-bottom:5px;font-size:12px}"
    ".monitoring-data{font-family:monospace;font-size:11px;background:#f8f9fa;padding:10px;border-radius:4px;max-height:200px;overflow-y:auto}"
    ".config-actions{margin-top:15px;text-align:center}"
    ".safety-controls{background:#fff3cd;border:1px solid #ffeaa7;border-radius:4px;padding:10px;margin-bottom:15px}"
    ".alert{padding:10px;border-radius:4px;margin-bottom:10px}"
    ".alert-warning{background:#fff3cd;border:1px solid #ffeaa7;color:#856404}"
    ".alert-success{background:#d4edda;border:1px solid #c3e6cb;color:#155724}"
    ".alert-danger{background:#f8d7da;border:1px solid #f5c6cb;color:#721c24}"
    "@media (max-width:768px){.section-grid{grid-template-columns:1fr}.form-row{flex-direction:column;align-items:stretch}.form-row label{min-width:auto}}"
    "</style></head><body>"
    "<div class='container'>"
    "<div class='header'>"
    "<h1>🤖 ESP32-C6 Position Reader</h1>"
    "<p>Wireless Robot Arm Control via ESP-NOW + CAN Bus</p>"
    "</div>"
    
    "<button class='emergency-stop' onclick='emergencyStop()'>🛑 EMERGENCY STOP</button>"
    "<div class='status-bar' id='statusBar'>Status: Initializing...</div>"
    
    "<div class='tab-container'>"
    "<button class='tab active' onclick='showTab(\"config\")'>Configuration</button>"
    "<button class='tab' onclick='showTab(\"reader-test\")'>Motor Testing</button>"
    "<button class='tab' onclick='showTab(\"target-test\")'>Real-time Monitor</button>"
    "</div>"
    
    "<!-- Configuration Tab -->"
    "<div id='config' class='tab-content active'>"
    "<div class='section-grid'>"
    "<div class='section'>"
    "<h4>📡 ESP-NOW Devices</h4>"
    "<div class='device-list' id='espnowDevices'>"
    "<div class='alert alert-warning'>Scanning for ESP-NOW devices...</div>"
    "</div>"
    "<div class='form-row'>"
    "<button class='btn btn-sm' onclick='scanDevices()'>Scan Devices</button>"
    "<button class='btn btn-sm btn-success' onclick='pairSelected()'>Pair Selected</button>"
    "</div>"
    "</div>"
    "<div class='section'>"
    "<h4>⚡ System Status</h4>"
    "<div id='systemStatus'>"
    "<div>Local Device: <span id='localMAC'>Loading...</span></div>"
    "<div>Paired Device: <span id='pairedDevice'>None</span></div>"
    "<div>Connection: <span id='connectionStatus'>Disconnected</span></div>"
    "<div>Monitoring: <span id='monitoringStatus'>Stopped</span></div>"
    "</div>"
    "</div>"
    "</div>"
    
    "<div class='section-grid'>"
    "<div class='section'>"
    "<h4>📖 Reader Motors (Source)</h4>"
    "<table class='motor-table' id='readerMotorsTable'>"
    "<thead><tr><th>ID</th><th>Type</th><th>Master</th><th>Status</th><th>Position</th><th>Action</th></tr></thead>"
    "<tbody></tbody>"
    "</table>"
    "<div class='form-row'>"
    "<select id='readerMotorType'>"
    "<option value='0'>DM4310</option><option value='1'>DM4310_48V</option>"
    "<option value='2'>DM4340</option><option value='3'>DM4340_48V</option>"
    "<option value='4'>DM6006</option><option value='5'>DM8006</option>"
    "<option value='6'>DM8009</option><option value='7'>DM10010L</option>"
    "<option value='8'>DM10010</option><option value='9'>DMH3510</option>"
    "<option value='10'>DMH6215</option><option value='11'>DMG6220</option>"
    "</select>"
    "<input type='text' id='readerSlaveId' placeholder='0x01' pattern='0x[0-9A-Fa-f]{1,2}'>"
    "<input type='text' id='readerMasterId' placeholder='0x11' pattern='0x[0-9A-Fa-f]{1,2}'>"
    "<button class='btn btn-sm btn-success' onclick='addReaderMotor()'>Add Reader</button>"
    "</div>"
    "</div>"
    "<div class='section'>"
    "<h4>🎯 Target Motors (Destination)</h4>"
    "<table class='motor-table' id='targetMotorsTable'>"
    "<thead><tr><th>ID</th><th>Type</th><th>Master</th><th>Status</th><th>Position</th><th>Action</th></tr></thead>"
    "<tbody></tbody>"
    "</table>"
    "<div class='form-row'>"
    "<select id='targetMotorType'>"
    "<option value='0'>DM4310</option><option value='1'>DM4310_48V</option>"
    "<option value='2'>DM4340</option><option value='3'>DM4340_48V</option>"
    "<option value='4'>DM6006</option><option value='5'>DM8006</option>"
    "<option value='6'>DM8009</option><option value='7'>DM10010L</option>"
    "<option value='8'>DM10010</option><option value='9'>DMH3510</option>"
    "<option value='10'>DMH6215</option><option value='11'>DMG6220</option>"
    "</select>"
    "<input type='text' id='targetSlaveId' placeholder='0x01' pattern='0x[0-9A-Fa-f]{1,2}'>"
    "<input type='text' id='targetMasterId' placeholder='0x11' pattern='0x[0-9A-Fa-f]{1,2}'>"
    "<button class='btn btn-sm btn-success' onclick='addTargetMotor()'>Add Target</button>"
    "</div>"
    "</div>"
    "</div>"
    
    "<div class='section'>"
    "<h4>🔗 Motor Mapping (Reader → Target)</h4>"
    "<div id='motorMappings'></div>"
    "<div class='form-row'>"
    "<input type='text' id='mapReader' placeholder='0x01' pattern='0x[0-9A-Fa-f]{1,2}'>"
    "<span>→</span>"
    "<input type='text' id='mapTarget' placeholder='0x01' pattern='0x[0-9A-Fa-f]{1,2}'>"
    "<button class='btn btn-sm btn-success' onclick='addMapping()'>Add Mapping</button>"
    "</div>"
    "</div>"
    
    "<div class='config-actions'>"
    "<button class='btn btn-success' onclick='saveConfig()'>💾 Save Configuration</button>"
    "<button class='btn btn-warning' onclick='loadConfig()'>📁 Load Configuration</button>"
    "</div>"
    "</div>"
    
    "<!-- Motor Testing Tab -->"
    "<div id='reader-test' class='tab-content'>"
    "<div class='section'>"
    "<h4>🔧 DM Motor Testing & Control</h4>"
    
    "<div class='section-grid'>"
    "<div class='section'>"
    "<h4>Motor Selection & Basic Control</h4>"
    "<div class='form-row'>"
    "<label>Motor:</label>"
    "<select id='testMotorSelect'></select>"
    "<button class='btn btn-sm btn-success' onclick='enableMotor()'>Enable</button>"
    "<button class='btn btn-sm btn-danger' onclick='disableMotor()'>Disable</button>"
    "<button class='btn btn-sm btn-warning' onclick='setZeroPosition()'>Set Zero</button>"
    "</div>"
    "<div class='form-row'>"
    "<label>Control Mode:</label>"
    "<select id='controlModeSelect'>"
    "<option value='1'>MIT Control</option>"
    "<option value='2'>Position/Velocity</option>"
    "<option value='3'>Velocity Only</option>"
    "<option value='4'>Torque/Position</option>"
    "</select>"
    "<button class='btn btn-sm' onclick='switchControlMode()'>Switch Mode</button>"
    "</div>"
    "</div>"
    
    "<div class='section'>"
    "<h4>Motor Status</h4>"
    "<div id='motorStatus'>"
    "<div>Position: <span id='motorPosition'>0.000</span> rad</div>"
    "<div>Velocity: <span id='motorVelocity'>0.000</span> rad/s</div>"
    "<div>Torque: <span id='motorTorque'>0.000</span> Nm</div>"
    "<div>Status: <span id='motorConnectionStatus'>Disconnected</span></div>"
    "</div>"
    "<button class='btn btn-sm' onclick='refreshMotorStatus()'>Refresh Status</button>"
    "</div>"
    "</div>"
    
    "<div class='section'>"
    "<h4>MIT Control Mode</h4>"
    "<div class='form-row'>"
    "<label>Kp:</label><input type='number' id='mitKp' value='10' step='0.1' min='0' max='500'>"
    "<label>Kd:</label><input type='number' id='mitKd' value='1' step='0.1' min='0' max='5'>"
    "</div>"
    "<div class='form-row'>"
    "<label>Position (rad):</label><input type='number' id='mitPosition' value='0' step='0.1'>"
    "<label>Velocity (rad/s):</label><input type='number' id='mitVelocity' value='0' step='0.1'>"
    "<label>Torque (Nm):</label><input type='number' id='mitTorque' value='0' step='0.1'>"
    "</div>"
    "<div class='form-row'>"
    "<button class='btn btn-success' onclick='sendMITControl()'>Send MIT Control</button>"
    "</div>"
    "</div>"
    
    "<div class='section'>"
    "<h4>Position/Velocity Control</h4>"
    "<div class='form-row'>"
    "<label>Target Position (rad):</label><input type='number' id='posVelPosition' value='0' step='0.1'>"
    "<label>Target Velocity (rad/s):</label><input type='number' id='posVelVelocity' value='1' step='0.1'>"
    "</div>"
    "<div class='form-row'>"
    "<button class='btn btn-success' onclick='sendPosVelControl()'>Send Pos/Vel Control</button>"
    "</div>"
    "</div>"
    
    "<div class='section'>"
    "<h4>Velocity Only Control</h4>"
    "<div class='form-row'>"
    "<label>Target Velocity (rad/s):</label><input type='number' id='velOnlyVelocity' value='0' step='0.1'>"
    "<button class='btn btn-success' onclick='sendVelocityControl()'>Send Velocity Control</button>"
    "</div>"
    "</div>"
    
    "</div>"
    "</div>"
    
    "<!-- Real-time Monitor Tab -->"
    "<div id='target-test' class='tab-content'>"
    "<div class='section'>"
    "<h4>📊 Real-time Motor Monitoring</h4>"
    "<div class='form-row'>"
    "<button class='btn btn-success' onclick='startMonitoring()'>▶️ Start Monitoring</button>"
    "<button class='btn btn-danger' onclick='stopMonitoring()'>⏹️ Stop Monitoring</button>"
    "<label><input type='checkbox' id='autoRefresh'> Auto-refresh (1Hz)</label>"
    "</div>"
    "<div class='monitoring-data' id='monitoringOutput'></div>"
    "</div>"
    "</div>"
    
    "</div>"
    
    "<script>"
    "let socket;"
    "let config={readerMotors:[],targetMotors:[],mappings:{},pairedDevice:null,safety:{maxJump:2.0,posLimit:10.0}};"
    "let selectedDevice=null;"
    "let monitoring=false;"
    
    "function showTab(tabName){"
    "document.querySelectorAll('.tab').forEach(t=>t.classList.remove('active'));"
    "document.querySelectorAll('.tab-content').forEach(t=>t.classList.remove('active'));"
    "event.target.classList.add('active');"
    "document.getElementById(tabName).classList.add('active');}"
    
    "function updateStatus(msg){"
    "document.getElementById('statusBar').textContent='Status: '+msg;"
    "console.log('Status:',msg);}"
    
    "function emergencyStop(){"
    "if(confirm('Emergency stop will disable ALL motors and stop monitoring. Continue?')){"
    "fetch('/api/safety',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:'emergency_stop'})});"
    "stopMonitoring();"
    "updateStatus('EMERGENCY STOP ACTIVATED');"
    "}}"
    
    "async function loadData(){"
    "try{"
    "const res=await fetch('/api/config');"
    "if(res.ok){"
    "config=await res.json();"
    "renderAll();"
    "updateStatus('Configuration loaded');"
    "}else updateStatus('Failed to load configuration');"
    "}catch(e){updateStatus('Error loading data: '+e.message);}}"
    
    "function renderAll(){"
    "renderReaderMotors();"
    "renderTargetMotors();"
    "renderMappings();"
    "updateSystemStatus();}"
    
    "function renderReaderMotors(){"
    "const tbody=document.querySelector('#readerMotorsTable tbody');"
    "tbody.innerHTML='';"
    "config.readerMotors.forEach(m=>{"
    "const row=tbody.insertRow();"
    "row.innerHTML=`<td>0x${m.slave_id.toString(16).toUpperCase().padStart(2,'0')}</td><td>${getMotorTypeName(m.motor_type)}</td><td>0x${m.master_id.toString(16).toUpperCase().padStart(2,'0')}</td><td><span class='status-indicator ${m.connected?'status-connected':'status-disconnected'}'></span>${m.enabled?'Enabled':'Disabled'}</td><td>${m.position?.toFixed(3)||'0.000'}</td><td><button class='btn btn-sm btn-danger' onclick='removeReaderMotor(${m.slave_id})'>Remove</button></td>`;"
    "});}"
    
    "function renderTargetMotors(){"
    "const tbody=document.querySelector('#targetMotorsTable tbody');"
    "tbody.innerHTML='';"
    "const select=document.getElementById('testMotorSelect');"
    "select.innerHTML='<option value=\"\">Select Motor</option>';"
    "config.targetMotors.forEach(m=>{"
    "const row=tbody.insertRow();"
    "row.innerHTML=`<td>0x${m.slave_id.toString(16).toUpperCase().padStart(2,'0')}</td><td>${getMotorTypeName(m.motor_type)}</td><td>0x${m.master_id.toString(16).toUpperCase().padStart(2,'0')}</td><td><span class='status-indicator ${m.connected?'status-connected':'status-disconnected'}'></span>${m.enabled?'Enabled':'Disabled'}</td><td>${m.position?.toFixed(3)||'0.000'}</td><td><button class='btn btn-sm btn-danger' onclick='removeTargetMotor(${m.slave_id})'>Remove</button></td>`;"
    "select.innerHTML+=`<option value='${m.slave_id}'>Motor 0x${m.slave_id.toString(16).toUpperCase().padStart(2,'0')} (${getMotorTypeName(m.motor_type)})</option>`;"
    "});}"
    
    "function renderMappings(){"
    "const div=document.getElementById('motorMappings');"
    "div.innerHTML='';"
    "Object.entries(config.mappings).forEach(([r,t])=>{"
    "const item=document.createElement('div');"
    "item.className='mapping-item';"
    "item.innerHTML=`<span>Reader 0x${parseInt(r).toString(16).toUpperCase().padStart(2,'0')} → Target 0x${parseInt(t).toString(16).toUpperCase().padStart(2,'0')}</span><button class='btn btn-sm btn-danger' onclick='removeMapping(${r})'>Remove</button>`;"
    "div.appendChild(item);"
    "});}"
    
    "function updateSystemStatus(){"
    "document.getElementById('localMAC').textContent=config.localMAC||'Unknown';"
    "document.getElementById('pairedDevice').textContent=config.pairedDevice||'None';"
    "document.getElementById('connectionStatus').innerHTML=config.pairedDevice?'<span class=\"status-connected\"></span>Connected':'<span class=\"status-disconnected\"></span>Disconnected';"
    "document.getElementById('monitoringStatus').innerHTML=monitoring?'<span class=\"status-connected\"></span>Running':'<span class=\"status-disconnected\"></span>Stopped';}"
    
    "function getMotorTypeName(type){"
    "const types=['DM4310','DM4310_48V','DM4340','DM4340_48V','DM6006','DM8006','DM8009','DM10010L','DM10010','DMH3510','DMH6215','DMG6220'];"
    "return types[type]||'Unknown';}"
    
    "async function saveConfig(){"
    "try{"
    "const res=await fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(config)});"
    "if(res.ok){updateStatus('Configuration saved');}else{updateStatus('Failed to save configuration');}"
    "}catch(e){updateStatus('Error saving: '+e.message);}}"
    
    "async function loadConfig(){"
    "loadData();"
    "updateStatus('Configuration reloaded');}"
    
    "async function startMonitoring(){"
    "const persistent=document.getElementById('persistentMode').checked;"
    "try{"
    "const res=await fetch('/api/monitoring/start',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({persistent})});"
    "if(res.ok){monitoring=true;updateSystemStatus();updateStatus('Position monitoring started');}else{updateStatus('Failed to start monitoring');}"
    "}catch(e){updateStatus('Error: '+e.message);}}"
    
    "async function stopMonitoring(){"
    "try{"
    "const res=await fetch('/api/monitoring/stop',{method:'POST'});"
    "if(res.ok){monitoring=false;updateSystemStatus();updateStatus('Position monitoring stopped');}else{updateStatus('Failed to stop monitoring');}"
    "}catch(e){updateStatus('Error: '+e.message);}}"
    
    "function enableMotor(){"
    "const motorId=document.getElementById('testMotorSelect').value;"
    "if(!motorId){updateStatus('Please select a motor');return;}"
    "fetch('/api/motor/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({motor_id:parseInt(motorId),action:'enable'})}).then(()=>updateStatus('Motor enabled')).catch(()=>updateStatus('Failed to enable motor'));}"
    
    "function disableMotor(){"
    "const motorId=document.getElementById('testMotorSelect').value;"
    "if(!motorId){updateStatus('Please select a motor');return;}"
    "fetch('/api/motor/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({motor_id:parseInt(motorId),action:'disable'})}).then(()=>updateStatus('Motor disabled')).catch(()=>updateStatus('Failed to disable motor'));}"
    
    "function setZeroPosition(){"
    "const motorId=document.getElementById('testMotorSelect').value;"
    "if(!motorId){updateStatus('Please select a motor');return;}"
    "fetch('/api/motor/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({motor_id:parseInt(motorId),action:'zero'})}).then(()=>updateStatus('Zero position set')).catch(()=>updateStatus('Failed to set zero position'));}"
    
    "function switchControlMode(){"
    "const motorId=document.getElementById('testMotorSelect').value;"
    "const mode=parseInt(document.getElementById('controlModeSelect').value);"
    "if(!motorId){updateStatus('Please select a motor');return;}"
    "fetch('/api/motor/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({motor_id:parseInt(motorId),action:'switch_mode',control_mode:mode})}).then(()=>updateStatus('Control mode switched')).catch(()=>updateStatus('Failed to switch control mode'));}"
    
    "function sendMITControl(){"
    "const motorId=document.getElementById('testMotorSelect').value;"
    "const kp=parseFloat(document.getElementById('mitKp').value);"
    "const kd=parseFloat(document.getElementById('mitKd').value);"
    "const pos=parseFloat(document.getElementById('mitPosition').value);"
    "const vel=parseFloat(document.getElementById('mitVelocity').value);"
    "const torque=parseFloat(document.getElementById('mitTorque').value);"
    "if(!motorId){updateStatus('Please select a motor');return;}"
    "fetch('/api/motor/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({motor_id:parseInt(motorId),action:'mit_control',kp:kp,kd:kd,position:pos,velocity:vel,torque:torque})}).then(()=>updateStatus('MIT control sent')).catch(()=>updateStatus('Failed to send MIT control'));}"
    
    "function sendPosVelControl(){"
    "const motorId=document.getElementById('testMotorSelect').value;"
    "const pos=parseFloat(document.getElementById('posVelPosition').value);"
    "const vel=parseFloat(document.getElementById('posVelVelocity').value);"
    "if(!motorId){updateStatus('Please select a motor');return;}"
    "fetch('/api/motor/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({motor_id:parseInt(motorId),action:'move',position:pos,velocity:vel})}).then(()=>updateStatus('Position/velocity control sent')).catch(()=>updateStatus('Failed to send pos/vel control'));}"
    
    "function sendVelocityControl(){"
    "const motorId=document.getElementById('testMotorSelect').value;"
    "const vel=parseFloat(document.getElementById('velOnlyVelocity').value);"
    "if(!motorId){updateStatus('Please select a motor');return;}"
    "fetch('/api/motor/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({motor_id:parseInt(motorId),action:'velocity_control',velocity:vel})}).then(()=>updateStatus('Velocity control sent')).catch(()=>updateStatus('Failed to send velocity control'));}"
    
    "function refreshMotorStatus(){"
    "const motorId=document.getElementById('testMotorSelect').value;"
    "if(!motorId){updateStatus('Please select a motor');return;}"
    "fetch('/api/motor/status').then(res=>res.json()).then(data=>{"
    "const motor=data.motors.find(m=>m.slave_id==motorId);"
    "if(motor){"
    "document.getElementById('motorPosition').textContent=motor.position.toFixed(3);"
    "document.getElementById('motorVelocity').textContent=motor.velocity.toFixed(3);"
    "document.getElementById('motorTorque').textContent=motor.torque.toFixed(3);"
    "document.getElementById('motorConnectionStatus').textContent=motor.connected?'Connected':'Disconnected';"
    "updateStatus('Motor status refreshed');"
    "}}).catch(()=>updateStatus('Failed to refresh motor status'));}"
    
    "function scanDevices(){"
    "updateStatus('Scanning for ESP-NOW devices...');"
    "fetch('/api/espnow/scan',{method:'POST'});}"
    
    "function pairSelected(){"
    "if(selectedDevice){"
    "updateStatus('Pairing with selected device...');"
    "fetch('/api/espnow/pair',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({device:selectedDevice})});"
    "}else{updateStatus('No device selected for pairing');}}"
    
    "async function addReaderMotor(){"
    "const type=parseInt(document.getElementById('readerMotorType').value);"
    "const slaveIdStr=document.getElementById('readerSlaveId').value;"
    "const masterIdStr=document.getElementById('readerMasterId').value;"
    "if(!slaveIdStr.match(/^0x[0-9A-Fa-f]{1,2}$/)||!masterIdStr.match(/^0x[0-9A-Fa-f]{1,2}$/)){updateStatus('Please enter valid hex IDs (e.g., 0x01)');return;}"
    "const slaveId=parseInt(slaveIdStr,16);"
    "const masterId=parseInt(masterIdStr,16);"
    "try{"
    "const res=await fetch('/api/motors/reader',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({motor_type:type,slave_id:slaveId,master_id:masterId})});"
    "if(res.ok){loadData();updateStatus('Reader motor added');document.getElementById('readerSlaveId').value='';document.getElementById('readerMasterId').value='';}else{updateStatus('Failed to add reader motor');}"
    "}catch(e){updateStatus('Error: '+e.message);}}"
    
    "async function addTargetMotor(){"
    "const type=parseInt(document.getElementById('targetMotorType').value);"
    "const slaveIdStr=document.getElementById('targetSlaveId').value;"
    "const masterIdStr=document.getElementById('targetMasterId').value;"
    "if(!slaveIdStr.match(/^0x[0-9A-Fa-f]{1,2}$/)||!masterIdStr.match(/^0x[0-9A-Fa-f]{1,2}$/)){updateStatus('Please enter valid hex IDs (e.g., 0x01)');return;}"
    "const slaveId=parseInt(slaveIdStr,16);"
    "const masterId=parseInt(masterIdStr,16);"
    "try{"
    "const res=await fetch('/api/motors/target',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({motor_type:type,slave_id:slaveId,master_id:masterId})});"
    "if(res.ok){loadData();updateStatus('Target motor added');document.getElementById('targetSlaveId').value='';document.getElementById('targetMasterId').value='';}else{updateStatus('Failed to add target motor');}"
    "}catch(e){updateStatus('Error: '+e.message);}}"
    
    "async function removeReaderMotor(id){"
    "if(confirm('Remove reader motor '+id+'?')){"
    "try{"
    "const res=await fetch('/api/motors/reader/'+id,{method:'DELETE'});"
    "if(res.ok){loadData();updateStatus('Reader motor removed');}else{updateStatus('Failed to remove reader motor');}"
    "}catch(e){updateStatus('Error: '+e.message);}}}"
    
    "async function removeTargetMotor(id){"
    "if(confirm('Remove target motor '+id+'?')){"
    "try{"
    "const res=await fetch('/api/motors/target/'+id,{method:'DELETE'});"
    "if(res.ok){loadData();updateStatus('Target motor removed');}else{updateStatus('Failed to remove target motor');}"
    "}catch(e){updateStatus('Error: '+e.message);}}}"
    
    "async function addMapping(){"
    "const readerStr=document.getElementById('mapReader').value;"
    "const targetStr=document.getElementById('mapTarget').value;"
    "if(!readerStr.match(/^0x[0-9A-Fa-f]{1,2}$/)||!targetStr.match(/^0x[0-9A-Fa-f]{1,2}$/)){updateStatus('Please enter valid hex IDs (e.g., 0x01)');return;}"
    "const reader=parseInt(readerStr,16);"
    "const target=parseInt(targetStr,16);"
    "try{"
    "const res=await fetch('/api/mapping',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({reader_id:reader,target_id:target})});"
    "if(res.ok){loadData();updateStatus('Mapping added');document.getElementById('mapReader').value='';document.getElementById('mapTarget').value='';}else{updateStatus('Failed to add mapping');}"
    "}catch(e){updateStatus('Error: '+e.message);}}"
    
    "async function removeMapping(readerId){"
    "if(confirm('Remove mapping for reader '+readerId+'?')){"
    "try{"
    "const res=await fetch('/api/mapping/'+readerId,{method:'DELETE'});"
    "if(res.ok){loadData();updateStatus('Mapping removed');}else{updateStatus('Failed to remove mapping');}"
    "}catch(e){updateStatus('Error: '+e.message);}}}"
    
    "async function moveTestMotor(){"
    "const motorId=document.getElementById('testMotorSelect').value;"
    "const position=parseFloat(document.getElementById('testPosition').value);"
    "if(!motorId){updateStatus('Please select a motor');return;}"
    "if(isNaN(position)){updateStatus('Please enter a valid position');return;}"
    "try{"
    "const res=await fetch('/api/motor/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({motor_id:parseInt(motorId),action:'move',position:position,velocity:10.0})});"
    "if(res.ok){updateStatus('Motor '+motorId+' moving to position '+position);}else{updateStatus('Failed to move motor');}"
    "}catch(e){updateStatus('Error: '+e.message);}}"
    
    "async function setTestZero(){"
    "const motorId=document.getElementById('testMotorSelect').value;"
    "if(!motorId){updateStatus('Please select a motor');return;}"
    "try{"
    "const res=await fetch('/api/motor/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({motor_id:parseInt(motorId),action:'zero'})});"
    "if(res.ok){updateStatus('Motor '+motorId+' zero position set');}else{updateStatus('Failed to set zero position');}"
    "}catch(e){updateStatus('Error: '+e.message);}}"
    
    
    "document.addEventListener('DOMContentLoaded',function(){"
    "updateStatus('Initializing Position Reader GUI...');"
    "loadData();"
    "});"
    
    "</script>"
    "</body></html>";
    
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, html_content, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t admin_handler(httpd_req_t *req) {
    const char* admin_html = 
    "<!DOCTYPE html><html><head><title>Admin Panel</title>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<style>"
    "body{font-family:Arial,sans-serif;margin:20px;background:#f5f5f5}"
    ".container{max-width:600px;margin:0 auto;background:white;padding:30px;border-radius:10px;box-shadow:0 4px 12px rgba(0,0,0,0.15)}"
    ".header{text-align:center;margin-bottom:30px;padding-bottom:20px;border-bottom:2px solid #3498db}"
    ".header h1{margin:0;color:#2c3e50;font-size:28px}"
    ".nav-section{margin-bottom:25px}"
    ".nav-section h3{color:#2c3e50;margin-bottom:15px}"
    ".nav-links{display:flex;flex-direction:column;gap:15px}"
    ".nav-link{display:block;padding:15px 20px;background:#3498db;color:white;text-decoration:none;border-radius:8px;text-align:center;font-weight:bold;transition:background 0.3s}"
    ".nav-link:hover{background:#2980b9}"
    ".nav-link.ota{background:#e67e22}"
    ".nav-link.ota:hover{background:#d35400}"
    ".nav-link.home{background:#27ae60}"
    ".nav-link.home:hover{background:#229954}"
    "</style></head>"
    "<body><div class='container'>"
    "<div class='header'><h1>⚙️ Admin Panel</h1></div>"
    "<div class='nav-section'><h3>System Management</h3>"
    "<div class='nav-links'>"
    "<a href='/ota' class='nav-link ota'>🚀 OTA Firmware Update</a>"
    "<a href='/api/system/info' class='nav-link'>📊 System Information (JSON)</a>"
    "</div></div>"
    "<div class='nav-section'><h3>Motor Control</h3>"
    "<div class='nav-links'>"
    "<a href='/' class='nav-link home'>🏠 Main Control Interface</a>"
    "<a href='/api/motor/status' class='nav-link'>📈 Motor Status (JSON)</a>"
    "<a href='/api/config' class='nav-link'>⚙️ Configuration (JSON)</a>"
    "</div></div>"
    "</div></body></html>";
    
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, admin_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t api_motors_handler(httpd_req_t *req) {
    cJSON *json = cJSON_CreateArray();
    
    for (int i = 0; i < motor_count; i++) {
        cJSON *motor_obj = cJSON_CreateObject();
        cJSON_AddNumberToObject(motor_obj, "slave_id", motors[i].slave_id);
        cJSON_AddNumberToObject(motor_obj, "master_id", motors[i].master_id);
        cJSON_AddNumberToObject(motor_obj, "motor_type", motors[i].motor_type);
        cJSON_AddBoolToObject(motor_obj, "enabled", motors[i].enabled);
        cJSON_AddBoolToObject(motor_obj, "connected", motors[i].connected);
        cJSON_AddNumberToObject(motor_obj, "position", motors[i].position);
        cJSON_AddNumberToObject(motor_obj, "velocity", motors[i].velocity);
        cJSON_AddNumberToObject(motor_obj, "torque", motors[i].torque);
        cJSON_AddItemToArray(json, motor_obj);
    }
    
    char *json_str = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);
    
    free(json_str);
    cJSON_Delete(json);
    return ESP_OK;
}

esp_err_t api_motor_control_handler(httpd_req_t *req) {
    char buf[512];
    int ret = httpd_req_recv(req, buf, sizeof(buf));
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[ret] = 0;
    
    cJSON *json = cJSON_Parse(buf);
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad Request");
        return ESP_FAIL;
    }
    
    cJSON *motor_id_json = cJSON_GetObjectItem(json, "motor_id");
    cJSON *action_json = cJSON_GetObjectItem(json, "action");
    
    if (!cJSON_IsNumber(motor_id_json) || !cJSON_IsString(action_json)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad Request");
        return ESP_FAIL;
    }
    
    int motor_id = motor_id_json->valueint;
    char *action = action_json->valuestring;
    
    ESP_LOGI(TAG, "Motor control request: motor_id=%d, action=%s", motor_id, action);
    
    dm_motor_t *motor = find_motor_by_id(motor_id);
    if (motor == NULL) {
        ESP_LOGW(TAG, "Motor control failed: Motor ID %d not found. Available motors:", motor_id);
        for (int i = 0; i < motor_count; i++) {
            ESP_LOGW(TAG, "  Motor %d: slave_id=%d, master_id=%d", i, motors[i].slave_id, motors[i].master_id);
        }
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Motor not found - please add it in Configuration tab first");
        return ESP_FAIL;
    }
    
    esp_err_t result = ESP_OK;
    if (strcmp(action, "enable") == 0) {
        result = motor_enable(motor);
    } else if (strcmp(action, "disable") == 0) {
        result = motor_disable(motor);
    } else if (strcmp(action, "zero") == 0) {
        result = motor_set_zero_position(motor);
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set zero position for motor %d: %s", motor_id, esp_err_to_name(result));
        }
    } else if (strcmp(action, "move") == 0) {
        cJSON *pos_json = cJSON_GetObjectItem(json, "position");
        cJSON *vel_json = cJSON_GetObjectItem(json, "velocity");
        if (cJSON_IsNumber(pos_json) && cJSON_IsNumber(vel_json)) {
            float target_pos = pos_json->valuedouble;
            float target_vel = vel_json->valuedouble;
            
            // Safety validation
            if (safety_config.emergency_stop) {
                ESP_LOGW(TAG, "Motor control blocked - emergency stop active");
                result = ESP_FAIL;
            } else if (!validate_motor_position(motor, target_pos)) {
                ESP_LOGW(TAG, "Motor control blocked - position validation failed");
                result = ESP_FAIL;
            } else if (!validate_motor_velocity(motor, target_vel)) {
                ESP_LOGW(TAG, "Motor control blocked - velocity validation failed");
                result = ESP_FAIL;
            } else {
                result = motor_control_pos_vel(motor, target_pos, target_vel);
            }
        }
    } else if (strcmp(action, "mit_control") == 0) {
        // MIT control mode
        cJSON *kp_json = cJSON_GetObjectItem(json, "kp");
        cJSON *kd_json = cJSON_GetObjectItem(json, "kd");
        cJSON *pos_json = cJSON_GetObjectItem(json, "position");
        cJSON *vel_json = cJSON_GetObjectItem(json, "velocity");
        cJSON *torque_json = cJSON_GetObjectItem(json, "torque");
        
        if (cJSON_IsNumber(kp_json) && cJSON_IsNumber(kd_json) && 
            cJSON_IsNumber(pos_json) && cJSON_IsNumber(vel_json) && cJSON_IsNumber(torque_json)) {
            
            float kp = kp_json->valuedouble;
            float kd = kd_json->valuedouble;
            float pos = pos_json->valuedouble;
            float vel = vel_json->valuedouble;
            float torque = torque_json->valuedouble;
            
            if (!safety_config.emergency_stop) {
                result = motor_control_mit(motor, kp, kd, pos, vel, torque);
            } else {
                ESP_LOGW(TAG, "MIT control blocked - emergency stop active");
                result = ESP_FAIL;
            }
        }
    } else if (strcmp(action, "velocity_control") == 0) {
        // Velocity only control
        cJSON *vel_json = cJSON_GetObjectItem(json, "velocity");
        if (cJSON_IsNumber(vel_json)) {
            float velocity = vel_json->valuedouble;
            if (!safety_config.emergency_stop) {
                result = motor_control_vel(motor, velocity);
            } else {
                ESP_LOGW(TAG, "Velocity control blocked - emergency stop active");
                result = ESP_FAIL;
            }
        }
    } else if (strcmp(action, "switch_mode") == 0) {
        // Switch control mode
        cJSON *mode_json = cJSON_GetObjectItem(json, "control_mode");
        if (cJSON_IsNumber(mode_json)) {
            dm_control_type_t mode = (dm_control_type_t)mode_json->valueint;
            ESP_LOGI(TAG, "Switching motor %d to control mode %d", motor_id, mode);
            result = switch_control_mode(motor, mode);
            if (result != ESP_OK) {
                ESP_LOGE(TAG, "Failed to switch control mode for motor %d: %s", motor_id, esp_err_to_name(result));
            }
        } else {
            ESP_LOGE(TAG, "Invalid or missing control_mode parameter for motor %d", motor_id);
            result = ESP_ERR_INVALID_ARG;
        }
    }
    
    cJSON_Delete(json);
    
    if (result == ESP_OK) {
        httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    } else {
        // Send more detailed error response
        char error_msg[128];
        snprintf(error_msg, sizeof(error_msg), "Motor control failed: %s", esp_err_to_name(result));
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, error_msg);
    }
    return result;
}

// Configuration management
static bool monitoring_active = false;
static dm_motor_t reader_motors[MAX_MOTORS];
static int32_t reader_motor_count = 0;

esp_err_t api_config_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        cJSON *json = cJSON_CreateObject();
        cJSON *reader_motors_array = cJSON_CreateArray();
        cJSON *target_motors_array = cJSON_CreateArray();
        cJSON *motor_mapping = cJSON_CreateObject();
        
        // Add reader motors
        for (int i = 0; i < reader_motor_count; i++) {
            cJSON *motor_obj = cJSON_CreateObject();
            cJSON_AddNumberToObject(motor_obj, "slave_id", reader_motors[i].slave_id);
            cJSON_AddNumberToObject(motor_obj, "master_id", reader_motors[i].master_id);
            cJSON_AddNumberToObject(motor_obj, "motor_type", reader_motors[i].motor_type);
            cJSON_AddBoolToObject(motor_obj, "enabled", reader_motors[i].enabled);
            cJSON_AddBoolToObject(motor_obj, "connected", reader_motors[i].connected);
            cJSON_AddNumberToObject(motor_obj, "position", reader_motors[i].position);
            cJSON_AddNumberToObject(motor_obj, "velocity", reader_motors[i].velocity);
            cJSON_AddNumberToObject(motor_obj, "torque", reader_motors[i].torque);
            cJSON_AddItemToArray(reader_motors_array, motor_obj);
        }
        
        // Add target motors (sync with main motor array)
        for (int i = 0; i < motor_count; i++) {
            cJSON *motor_obj = cJSON_CreateObject();
            cJSON_AddNumberToObject(motor_obj, "slave_id", motors[i].slave_id);
            cJSON_AddNumberToObject(motor_obj, "master_id", motors[i].master_id);
            cJSON_AddNumberToObject(motor_obj, "motor_type", motors[i].motor_type);
            cJSON_AddBoolToObject(motor_obj, "enabled", motors[i].enabled);
            cJSON_AddBoolToObject(motor_obj, "connected", motors[i].connected);
            cJSON_AddNumberToObject(motor_obj, "position", motors[i].position);
            cJSON_AddNumberToObject(motor_obj, "velocity", motors[i].velocity);
            cJSON_AddNumberToObject(motor_obj, "torque", motors[i].torque);
            cJSON_AddItemToArray(target_motors_array, motor_obj);
        }
        
        // Simple 1:1 mapping for existing motors
        for (int i = 0; i < reader_motor_count && i < motor_count; i++) {
            char id_str[10];
            snprintf(id_str, sizeof(id_str), "%d", reader_motors[i].slave_id);
            cJSON_AddNumberToObject(motor_mapping, id_str, motors[i].slave_id);
        }
        
        cJSON_AddItemToObject(json, "readerMotors", reader_motors_array);
        cJSON_AddItemToObject(json, "targetMotors", target_motors_array);
        cJSON_AddItemToObject(json, "mappings", motor_mapping);
        
        char *json_str = cJSON_Print(json);
        httpd_resp_set_type(req, "application/json; charset=utf-8");
        httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);
        
        free(json_str);
        cJSON_Delete(json);
    } else if (req->method == HTTP_POST) {
        char buf[1024];
        int ret = httpd_req_recv(req, buf, sizeof(buf));
        if (ret <= 0) {
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        buf[ret] = 0;
        
        cJSON *json = cJSON_Parse(buf);
        if (json == NULL) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad Request");
            return ESP_FAIL;
        }
        
        // Save configuration to NVS
        nvs_handle_t nvs_handle;
        esp_err_t err = nvs_open("motor_config", NVS_READWRITE, &nvs_handle);
        if (err == ESP_OK) {
            // Save motor count and motor data
            nvs_set_i32(nvs_handle, "motor_count", motor_count);
            for (int i = 0; i < motor_count; i++) {
                char key[32];
                snprintf(key, sizeof(key), "motor_%d", i);
                nvs_set_blob(nvs_handle, key, &motors[i], sizeof(dm_motor_t));
            }
            nvs_commit(nvs_handle);
            nvs_close(nvs_handle);
            ESP_LOGI(TAG, "Saved motor configuration to NVS: %" PRId32 " motors", motor_count);
        }
        
        cJSON_Delete(json);
        
        httpd_resp_send(req, "Configuration saved", HTTPD_RESP_USE_STRLEN);
    }
    return ESP_OK;
}


esp_err_t api_monitoring_handler(httpd_req_t *req) {
    if (req->method == HTTP_POST) {
        char buf[100];
        int ret = httpd_req_recv(req, buf, sizeof(buf));
        if (ret <= 0) {
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        buf[ret] = 0;
        
        // Extract the action from the URL path
        const char *uri = req->uri;
        if (strstr(uri, "/start")) {
            monitoring_active = true;
            ESP_LOGI(TAG, "Starting position monitoring");
        } else if (strstr(uri, "/stop")) {
            monitoring_active = false;
            ESP_LOGI(TAG, "Stopping position monitoring");
        }
        
        httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    }
    return ESP_OK;
}

esp_err_t api_motor_test_handler(httpd_req_t *req) {
    if (req->method == HTTP_POST) {
        char buf[256];
        int ret = httpd_req_recv(req, buf, sizeof(buf));
        if (ret <= 0) {
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        buf[ret] = 0;
        
        cJSON *json = cJSON_Parse(buf);
        if (json == NULL) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad Request");
            return ESP_FAIL;
        }
        
        cJSON *motor_id_json = cJSON_GetObjectItem(json, "motor_id");
        cJSON *test_type_json = cJSON_GetObjectItem(json, "test_type");
        
        if (!cJSON_IsNumber(motor_id_json) || !cJSON_IsString(test_type_json)) {
            cJSON_Delete(json);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad Request");
            return ESP_FAIL;
        }
        
        int motor_id = motor_id_json->valueint;
        char *test_type = test_type_json->valuestring;
        
        dm_motor_t *motor = find_motor_by_id(motor_id);
        if (motor == NULL) {
            cJSON_Delete(json);
            httpd_resp_send_404(req);
            return ESP_FAIL;
        }
        
        esp_err_t result = ESP_OK;
        cJSON *response = cJSON_CreateObject();
        
        if (strcmp(test_type, "connection_test") == 0) {
            motor_refresh_status(motor);
            result = motor->connected ? ESP_OK : ESP_FAIL;
            cJSON_AddStringToObject(response, "message", motor->connected ? "Motor connected" : "Motor not responding");
            cJSON_AddBoolToObject(response, "connected", motor->connected);
        } else {
            cJSON_AddStringToObject(response, "error", "Unknown test type");
            result = ESP_FAIL;
        }
        
        cJSON_AddBoolToObject(response, "success", result == ESP_OK);
        cJSON_AddNumberToObject(response, "motor_id", motor_id);
        cJSON_AddStringToObject(response, "test_type", test_type);
        
        char *response_str = cJSON_Print(response);
        httpd_resp_set_type(req, "application/json; charset=utf-8");
        
        if (result == ESP_OK) {
            httpd_resp_send(req, response_str, HTTPD_RESP_USE_STRLEN);
        } else {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, response_str);
        }
        
        free(response_str);
        cJSON_Delete(response);
        cJSON_Delete(json);
        
        return result;
    }
    
    return ESP_OK;
}

esp_err_t api_motor_status_handler(httpd_req_t *req) {
    cJSON *json = cJSON_CreateObject();
    cJSON *motors_array = cJSON_CreateArray();
    
    for (int i = 0; i < motor_count; i++) {
        // Refresh motor status
        motor_refresh_status(&motors[i]);
        
        cJSON *motor_obj = cJSON_CreateObject();
        cJSON_AddNumberToObject(motor_obj, "slave_id", motors[i].slave_id);
        cJSON_AddNumberToObject(motor_obj, "master_id", motors[i].master_id);
        cJSON_AddNumberToObject(motor_obj, "motor_type", motors[i].motor_type);
        cJSON_AddBoolToObject(motor_obj, "enabled", motors[i].enabled);
        cJSON_AddBoolToObject(motor_obj, "connected", motors[i].connected);
        cJSON_AddNumberToObject(motor_obj, "position", motors[i].position);
        cJSON_AddNumberToObject(motor_obj, "velocity", motors[i].velocity);
        cJSON_AddNumberToObject(motor_obj, "torque", motors[i].torque);
        
        // Add motor type string for UI display
        const char* type_names[] = {"DM4310", "DM4310_48V", "DM4340", "DM4340_48V", "DM6006", "DM8006", "DM8009", "DM10010L", "DM10010", "DMH3510", "DMH6215", "DMG6220"};
        if (motors[i].motor_type < sizeof(type_names) / sizeof(type_names[0])) {
            cJSON_AddStringToObject(motor_obj, "type_name", type_names[motors[i].motor_type]);
        }
        
        cJSON_AddItemToArray(motors_array, motor_obj);
    }
    
    cJSON_AddItemToObject(json, "motors", motors_array);
    cJSON_AddNumberToObject(json, "motor_count", motor_count);
    cJSON_AddBoolToObject(json, "monitoring_active", monitoring_active);
    
    char *json_str = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);
    
    free(json_str);
    cJSON_Delete(json);
    return ESP_OK;
}

esp_err_t api_espnow_devices_handler(httpd_req_t *req) {
    cJSON *json = cJSON_CreateArray();
    
    // For demonstration, add some mock devices
    cJSON *device1 = cJSON_CreateObject();
    cJSON_AddStringToObject(device1, "mac", "AA:BB:CC:DD:EE:F1");
    cJSON_AddStringToObject(device1, "name", "ESP32-C6 Reader");
    cJSON_AddBoolToObject(device1, "connected", true);
    cJSON_AddItemToArray(json, device1);
    
    cJSON *device2 = cJSON_CreateObject();
    cJSON_AddStringToObject(device2, "mac", "AA:BB:CC:DD:EE:F2");
    cJSON_AddStringToObject(device2, "name", "ESP32-C6 Target");
    cJSON_AddBoolToObject(device2, "connected", false);
    cJSON_AddItemToArray(json, device2);
    
    char *json_str = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);
    
    free(json_str);
    cJSON_Delete(json);
    return ESP_OK;
}

esp_err_t ota_page_handler(httpd_req_t *req) {
    const char* ota_html = 
    "<!DOCTYPE html>"
    "<html><head><title>OTA Firmware Update</title>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<style>"
    "body{font-family:Arial,sans-serif;margin:20px;background:#f5f5f5}"
    ".container{max-width:800px;margin:0 auto;background:white;padding:30px;border-radius:10px;box-shadow:0 4px 12px rgba(0,0,0,0.15)}"
    ".header{text-align:center;margin-bottom:30px;padding-bottom:20px;border-bottom:2px solid #3498db}"
    ".header h1{margin:0;color:#2c3e50;font-size:28px}"
    ".header p{margin:10px 0 0;color:#7f8c8d;font-size:16px}"
    ".warning{background:#fff3cd;border:1px solid #ffeaa7;color:#856404;padding:15px;border-radius:6px;margin-bottom:20px}"
    ".warning h3{margin:0 0 10px;color:#b8860b}"
    ".upload-section{background:#f8f9fa;padding:25px;border-radius:8px;border:2px dashed #dee2e6;margin-bottom:20px}"
    ".upload-section h3{margin:0 0 15px;color:#2c3e50}"
    ".file-input{width:100%;padding:12px;border:2px solid #ddd;border-radius:6px;font-size:16px;margin-bottom:15px}"
    ".file-input:focus{border-color:#3498db;outline:none}"
    ".upload-btn{background:#27ae60;color:white;border:none;padding:15px 30px;font-size:16px;font-weight:bold;border-radius:6px;cursor:pointer;width:100%;transition:background 0.3s}"
    ".upload-btn:hover{background:#229954}"
    ".upload-btn:disabled{background:#95a5a6;cursor:not-allowed}"
    ".progress-container{margin-top:20px;display:none}"
    ".progress-bar{width:100%;height:20px;background:#ecf0f1;border-radius:10px;overflow:hidden}"
    ".progress-fill{height:100%;background:#3498db;width:0%;transition:width 0.3s;border-radius:10px}"
    ".progress-text{text-align:center;margin-top:10px;font-weight:bold}"
    ".status{padding:15px;border-radius:6px;margin-top:20px;display:none}"
    ".status.success{background:#d4edda;border:1px solid #c3e6cb;color:#155724}"
    ".status.error{background:#f8d7da;border:1px solid #f5c6cb;color:#721c24}"
    ".status.info{background:#d1ecf1;border:1px solid #bee5eb;color:#0c5460}"
    ".info-section{margin-bottom:20px}"
    ".info-item{display:flex;justify-content:space-between;padding:8px 0;border-bottom:1px solid #eee}"
    ".info-item:last-child{border-bottom:none}"
    ".info-label{font-weight:bold;color:#2c3e50}"
    ".nav-links{text-align:center;margin-top:30px;padding-top:20px;border-top:1px solid #eee}"
    ".nav-links a{color:#3498db;text-decoration:none;margin:0 15px;font-weight:bold}"
    ".nav-links a:hover{text-decoration:underline}"
    "</style></head><body>"
    "<div class='container'>"
    "<div class='header'>"
    "<h1>🚀 OTA Firmware Update</h1>"
    "<p>Upload new firmware to update your ESP32-C6 device wirelessly</p>"
    "</div>"
    
    "<div class='warning'>"
    "<h3>⚠️ Important Safety Information</h3>"
    "<ul>"
    "<li>Ensure the firmware file is compatible with ESP32-C6</li>"
    "<li>Do not power off or disconnect during update process</li>"
    "<li>Keep the device connected to a stable power source</li>"
    "<li>The device will automatically restart after successful update</li>"
    "<li>Have a backup plan to recover if update fails</li>"
    "</ul>"
    "</div>"
    
    "<div class='info-section'>"
    "<h3>📊 Current System Information</h3>"
    "<div class='info-item'>"
    "<span class='info-label'>Device:</span>"
    "<span>ESP32-C6 Motor Control System</span>"
    "</div>"
    "<div class='info-item'>"
    "<span class='info-label'>Current Version:</span>"
    "<span id='currentVersion'>Loading...</span>"
    "</div>"
    "<div class='info-item'>"
    "<span class='info-label'>Available Flash:</span>"
    "<span id='flashInfo'>Loading...</span>"
    "</div>"
    "<div class='info-item'>"
    "<span class='info-label'>Last Update:</span>"
    "<span id='lastUpdate'>Loading...</span>"
    "</div>"
    "</div>"
    
    "<div class='upload-section'>"
    "<h3>📁 Select Firmware File</h3>"
    "<input type='file' id='firmwareFile' class='file-input' accept='.bin' />"
    "<div id='fileInfo' style='margin-top:10px;color:#666;display:none'></div>"
    "<button id='uploadBtn' class='upload-btn' onclick='startUpload()'>Upload Firmware</button>"
    
    "<div id='progressContainer' class='progress-container'>"
    "<div class='progress-bar'>"
    "<div id='progressFill' class='progress-fill'></div>"
    "</div>"
    "<div id='progressText' class='progress-text'>0%</div>"
    "</div>"
    "</div>"
    
    "<div id='statusMessage' class='status'></div>"
    
    "<div class='nav-links'>"
    "<a href='/'>← Back to Home</a>"
    "<a href='/admin'>Admin Panel</a>"
    "<a href='javascript:location.reload()'>🔄 Refresh</a>"
    "</div>"
    "</div>"
    
    "<script>"
    "let uploadInProgress = false;"
    
    "document.getElementById('firmwareFile').addEventListener('change', function(e) {"
    "    const file = e.target.files[0];"
    "    const fileInfo = document.getElementById('fileInfo');"
    "    const uploadBtn = document.getElementById('uploadBtn');"
    "    "
    "    if (file) {"
    "        const sizeKB = (file.size / 1024).toFixed(2);"
    "        const sizeMB = (file.size / 1024 / 1024).toFixed(2);"
    "        fileInfo.innerHTML = `<strong>Selected:</strong> ${file.name}<br><strong>Size:</strong> ${sizeKB} KB (${sizeMB} MB)<br><strong>Type:</strong> ${file.type || 'application/octet-stream'}`;"
    "        fileInfo.style.display = 'block';"
    "        uploadBtn.disabled = false;"
    "        uploadBtn.textContent = 'Upload Firmware';"
    "    } else {"
    "        fileInfo.style.display = 'none';"
    "        uploadBtn.disabled = true;"
    "        uploadBtn.textContent = 'Select a file first';"
    "    }"
    "});"
    
    "function showStatus(message, type) {"
    "    const status = document.getElementById('statusMessage');"
    "    status.textContent = message;"
    "    status.className = `status ${type}`;"
    "    status.style.display = 'block';"
    "}"
    
    "function updateProgress(percent) {"
    "    document.getElementById('progressFill').style.width = percent + '%';"
    "    document.getElementById('progressText').textContent = Math.round(percent) + '%';"
    "}"
    
    "function startUpload() {"
    "    const fileInput = document.getElementById('firmwareFile');"
    "    const file = fileInput.files[0];"
    "    "
    "    if (!file) {"
    "        showStatus('Please select a firmware file first!', 'error');"
    "        return;"
    "    }"
    "    "
    "    if (uploadInProgress) {"
    "        showStatus('Upload already in progress!', 'error');"
    "        return;"
    "    }"
    "    "
    "    if (!file.name.endsWith('.bin')) {"
    "        if (!confirm('File does not have .bin extension. Continue anyway?')) {"
    "            return;"
    "        }"
    "    }"
    "    "
    "    if (!confirm('Start firmware update? Device will restart after completion.')) {"
    "        return;"
    "    }"
    "    "
    "    uploadInProgress = true;"
    "    document.getElementById('uploadBtn').disabled = true;"
    "    document.getElementById('uploadBtn').textContent = 'Uploading...';"
    "    document.getElementById('progressContainer').style.display = 'block';"
    "    showStatus('Starting firmware upload...', 'info');"
    "    "
    "    const xhr = new XMLHttpRequest();"
    "    "
    "    xhr.upload.onprogress = function(e) {"
    "        if (e.lengthComputable) {"
    "            const percent = (e.loaded / e.total) * 100;"
    "            updateProgress(percent);"
    "            showStatus(`Uploading firmware... ${Math.round(percent)}%`, 'info');"
    "        }"
    "    };"
    "    "
    "    xhr.onload = function() {"
    "        uploadInProgress = false;"
    "        if (xhr.status === 200) {"
    "            updateProgress(100);"
    "            showStatus('✅ Firmware uploaded successfully! Device is restarting...', 'success');"
    "            setTimeout(() => {"
    "                showStatus('Device should restart in a few seconds. You may need to refresh this page.', 'info');"
    "            }, 3000);"
    "        } else {"
    "            showStatus(`❌ Upload failed: ${xhr.status} ${xhr.statusText}`, 'error');"
    "            document.getElementById('uploadBtn').disabled = false;"
    "            document.getElementById('uploadBtn').textContent = 'Upload Firmware';"
    "        }"
    "    };"
    "    "
    "    xhr.onerror = function() {"
    "        uploadInProgress = false;"
    "        showStatus('❌ Network error during upload. Please try again.', 'error');"
    "        document.getElementById('uploadBtn').disabled = false;"
    "        document.getElementById('uploadBtn').textContent = 'Upload Firmware';"
    "    };"
    "    "
    "    xhr.ontimeout = function() {"
    "        uploadInProgress = false;"
    "        showStatus('❌ Upload timeout. Please try again with a stable connection.', 'error');"
    "        document.getElementById('uploadBtn').disabled = false;"
    "        document.getElementById('uploadBtn').textContent = 'Upload Firmware';"
    "    };"
    "    "
    "    xhr.open('POST', '/api/ota/upload', true);"
    "    xhr.timeout = 300000; // 5 minute timeout"
    "    xhr.send(file);"
    "}"
    
    "function loadSystemInfo() {"
    "    fetch('/api/system/info')"
    "    .then(response => response.json())"
    "    .then(data => {"
    "        document.getElementById('currentVersion').textContent = data.version || 'Unknown';"
    "        document.getElementById('flashInfo').textContent = data.flash_info || 'Unknown';"
    "        document.getElementById('lastUpdate').textContent = data.last_update || 'Unknown';"
    "    })"
    "    .catch(() => {"
    "        document.getElementById('currentVersion').textContent = 'Error loading';"
    "        document.getElementById('flashInfo').textContent = 'Error loading';"
    "        document.getElementById('lastUpdate').textContent = 'Error loading';"
    "    });"
    "}"
    
    "document.addEventListener('DOMContentLoaded', function() {"
    "    loadSystemInfo();"
    "    document.getElementById('uploadBtn').disabled = true;"
    "    document.getElementById('uploadBtn').textContent = 'Select a file first';"
    "});"
    
    "</script>"
    "</body></html>";
    
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, ota_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t api_system_info_handler(httpd_req_t *req) {
    cJSON *json = cJSON_CreateObject();
    
    // Get app description
    const esp_app_desc_t *app_desc = esp_app_get_description();
    cJSON_AddStringToObject(json, "version", app_desc->version);
    cJSON_AddStringToObject(json, "project_name", app_desc->project_name);
    cJSON_AddStringToObject(json, "compile_time", app_desc->time);
    cJSON_AddStringToObject(json, "compile_date", app_desc->date);
    cJSON_AddStringToObject(json, "idf_version", app_desc->idf_ver);
    
    // Get flash information
    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);
    char flash_info[64];
    snprintf(flash_info, sizeof(flash_info), "%.1f MB total", (float)flash_size / (1024 * 1024));
    cJSON_AddStringToObject(json, "flash_info", flash_info);
    
    // Get current partition info
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running) {
        char partition_info[128];
        snprintf(partition_info, sizeof(partition_info), "%s (0x%lx, %lu bytes)", 
                running->label, running->address, running->size);
        cJSON_AddStringToObject(json, "current_partition", partition_info);
    }
    
    // Get next OTA partition info
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition) {
        char next_partition_info[128];
        snprintf(next_partition_info, sizeof(next_partition_info), "%s (0x%lx, %lu bytes)", 
                update_partition->label, update_partition->address, update_partition->size);
        cJSON_AddStringToObject(json, "next_partition", next_partition_info);
    }
    
    // Add boot count from NVS if available
    cJSON_AddStringToObject(json, "last_update", "System startup");
    
    char *json_str = cJSON_Print(json);
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);
    
    free(json_str);
    cJSON_Delete(json);
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
            httpd_resp_set_type(req, "application/json; charset=utf-8");
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
        httpd_resp_set_type(req, "application/json; charset=utf-8");
        httpd_resp_send(req, response_str, HTTPD_RESP_USE_STRLEN);
        
        free(response_str);
        cJSON_Delete(response);
        
        return err;
    }
    
    return ESP_OK;
}

esp_err_t api_motor_delete_handler(httpd_req_t *req) {
    const char *uri = req->uri;
    ESP_LOGI(TAG, "DELETE request for URI: %s", uri);
    
    // Extract motor ID from URI path like /api/motors/target/4 or /api/motors/reader/3
    const char *id_start = strrchr(uri, '/');
    if (id_start == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid motor ID in URL");
        return ESP_FAIL;
    }
    
    int motor_id = atoi(id_start + 1);
    ESP_LOGI(TAG, "Attempting to remove motor with ID: %d", motor_id);
    
    if (strstr(uri, "/motors/target/")) {
        // Remove target motor from main motor array
        bool found = false;
        for (int i = 0; i < motor_count; i++) {
            if (motors[i].slave_id == motor_id) {
                // Shift remaining motors down
                for (int j = i; j < motor_count - 1; j++) {
                    motors[j] = motors[j + 1];
                }
                motor_count--;
                found = true;
                ESP_LOGI(TAG, "Removed target motor with slave_id=%d", motor_id);
                break;
            }
        }
        
        if (!found) {
            httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Target motor not found");
            return ESP_FAIL;
        }
    } 
    else if (strstr(uri, "/motors/reader/")) {
        // Remove reader motor from reader motor array
        bool found = false;
        for (int i = 0; i < reader_motor_count; i++) {
            if (reader_motors[i].slave_id == motor_id) {
                // Shift remaining motors down
                for (int j = i; j < reader_motor_count - 1; j++) {
                    reader_motors[j] = reader_motors[j + 1];
                }
                reader_motor_count--;
                found = true;
                ESP_LOGI(TAG, "Removed reader motor with slave_id=%d", motor_id);
                break;
            }
        }
        
        if (!found) {
            httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Reader motor not found");
            return ESP_FAIL;
        }
    }
    else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid endpoint");
        return ESP_FAIL;
    }
    
    httpd_resp_send(req, "Motor removed successfully", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t api_mapping_delete_handler(httpd_req_t *req) {
    const char *uri = req->uri;
    ESP_LOGI(TAG, "DELETE mapping request for URI: %s", uri);
    
    // Extract reader ID from URI path like /api/mapping/1
    const char *id_start = strrchr(uri, '/');
    if (id_start == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid reader ID in URL");
        return ESP_FAIL;
    }
    
    int reader_id = atoi(id_start + 1);
    ESP_LOGI(TAG, "Removed mapping for reader ID: %d (mappings not persistently stored yet)", reader_id);
    
    httpd_resp_send(req, "Mapping removed successfully", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t api_motor_management_handler(httpd_req_t *req) {
    if (req->method == HTTP_POST) {
        char buf[512];
        int ret = httpd_req_recv(req, buf, sizeof(buf));
        if (ret <= 0) {
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        buf[ret] = 0;
        
        cJSON *json = cJSON_Parse(buf);
        if (json == NULL) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad Request");
            return ESP_FAIL;
        }
        
        const char *uri = req->uri;
        
        if (strstr(uri, "/motors/reader")) {
            // Add reader motor functionality
            cJSON *motor_type_json = cJSON_GetObjectItem(json, "motor_type");
            cJSON *slave_id_json = cJSON_GetObjectItem(json, "slave_id");
            cJSON *master_id_json = cJSON_GetObjectItem(json, "master_id");
            
            if (cJSON_IsNumber(motor_type_json) && cJSON_IsNumber(slave_id_json) && cJSON_IsNumber(master_id_json)) {
                // Check if we have space for more reader motors
                if (reader_motor_count >= MAX_MOTORS) {
                    cJSON_Delete(json);
                    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Maximum reader motors reached");
                    return ESP_FAIL;
                }
                
                // Add new reader motor
                reader_motors[reader_motor_count].slave_id = slave_id_json->valueint;
                reader_motors[reader_motor_count].master_id = master_id_json->valueint;
                reader_motors[reader_motor_count].motor_type = motor_type_json->valueint;
                reader_motors[reader_motor_count].control_mode = CTRL_POS_VEL;
                reader_motors[reader_motor_count].position = 0.0f;
                reader_motors[reader_motor_count].velocity = 0.0f;
                reader_motors[reader_motor_count].torque = 0.0f;
                reader_motors[reader_motor_count].enabled = false;
                reader_motors[reader_motor_count].connected = false;
                reader_motor_count++;
                
                ESP_LOGI(TAG, "Added reader motor: slave_id=%d, master_id=%d, type=%d", 
                        slave_id_json->valueint, master_id_json->valueint, motor_type_json->valueint);
            }
        } else if (strstr(uri, "/motors/target")) {
            // Add target motor functionality (add to main motor array)
            cJSON *motor_type_json = cJSON_GetObjectItem(json, "motor_type");
            cJSON *slave_id_json = cJSON_GetObjectItem(json, "slave_id");
            cJSON *master_id_json = cJSON_GetObjectItem(json, "master_id");
            
            if (cJSON_IsNumber(motor_type_json) && cJSON_IsNumber(slave_id_json) && cJSON_IsNumber(master_id_json)) {
                // Check if we have space for more motors
                if (motor_count >= MAX_MOTORS) {
                    cJSON_Delete(json);
                    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Maximum motors reached");
                    return ESP_FAIL;
                }
                
                // Add new motor to main array
                motors[motor_count].slave_id = slave_id_json->valueint;
                motors[motor_count].master_id = master_id_json->valueint;
                motors[motor_count].motor_type = motor_type_json->valueint;
                motors[motor_count].control_mode = CTRL_MIT;
                motors[motor_count].position = 0.0f;
                motors[motor_count].velocity = 0.0f;
                motors[motor_count].torque = 0.0f;
                motors[motor_count].enabled = false;
                motors[motor_count].connected = false;
                motor_count++;
                
                ESP_LOGI(TAG, "Added motor: slave_id=%d, master_id=%d, type=%d", 
                        slave_id_json->valueint, master_id_json->valueint, motor_type_json->valueint);
            }
        } else if (strstr(uri, "/mapping")) {
            // Add mapping functionality
            cJSON *reader_id_json = cJSON_GetObjectItem(json, "reader_id");
            cJSON *target_id_json = cJSON_GetObjectItem(json, "target_id");
            
            if (cJSON_IsNumber(reader_id_json) && cJSON_IsNumber(target_id_json)) {
                // For now, just log the mapping (could store in NVS later)
                ESP_LOGI(TAG, "Added mapping: reader %d -> target %d", 
                        reader_id_json->valueint, target_id_json->valueint);
            }
        }
        
        cJSON_Delete(json);
        httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    
    return ESP_OK;
}

esp_err_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.max_uri_handlers = 20;

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Registering URI handlers");
        
        // Main pages
        httpd_uri_t index_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = index_handler,
        };
        httpd_register_uri_handler(server, &index_uri);

        httpd_uri_t admin_uri = {
            .uri       = "/admin",
            .method    = HTTP_GET,
            .handler   = admin_handler,
        };
        httpd_register_uri_handler(server, &admin_uri);

        // API endpoints
        httpd_uri_t api_motors_uri = {
            .uri       = "/api/motors",
            .method    = HTTP_GET,
            .handler   = api_motors_handler,
        };
        httpd_register_uri_handler(server, &api_motors_uri);

        httpd_uri_t api_motor_control_uri = {
            .uri       = "/api/motor/control",
            .method    = HTTP_POST,
            .handler   = api_motor_control_handler,
        };
        httpd_register_uri_handler(server, &api_motor_control_uri);

        httpd_uri_t api_config_get_uri = {
            .uri       = "/api/config",
            .method    = HTTP_GET,
            .handler   = api_config_handler,
        };
        httpd_register_uri_handler(server, &api_config_get_uri);

        httpd_uri_t api_config_post_uri = {
            .uri       = "/api/config",
            .method    = HTTP_POST,
            .handler   = api_config_handler,
        };
        httpd_register_uri_handler(server, &api_config_post_uri);

        httpd_uri_t api_monitoring_start_uri = {
            .uri       = "/api/monitoring/start",
            .method    = HTTP_POST,
            .handler   = api_monitoring_handler,
        };
        httpd_register_uri_handler(server, &api_monitoring_start_uri);

        httpd_uri_t api_monitoring_stop_uri = {
            .uri       = "/api/monitoring/stop",
            .method    = HTTP_POST,
            .handler   = api_monitoring_handler,
        };
        httpd_register_uri_handler(server, &api_monitoring_stop_uri);

        httpd_uri_t api_motor_test_uri = {
            .uri       = "/api/motor/test",
            .method    = HTTP_POST,
            .handler   = api_motor_test_handler,
        };
        httpd_register_uri_handler(server, &api_motor_test_uri);

        httpd_uri_t api_motor_status_uri = {
            .uri       = "/api/motor/status",
            .method    = HTTP_GET,
            .handler   = api_motor_status_handler,
        };
        httpd_register_uri_handler(server, &api_motor_status_uri);

        httpd_uri_t api_espnow_devices_uri = {
            .uri       = "/api/espnow/devices",
            .method    = HTTP_GET,
            .handler   = api_espnow_devices_handler,
        };
        httpd_register_uri_handler(server, &api_espnow_devices_uri);

        httpd_uri_t api_espnow_scan_uri = {
            .uri       = "/api/espnow/scan",
            .method    = HTTP_POST,
            .handler   = api_espnow_devices_handler,
        };
        httpd_register_uri_handler(server, &api_espnow_scan_uri);

        httpd_uri_t api_ota_upload_uri = {
            .uri       = "/api/ota/upload",
            .method    = HTTP_POST,
            .handler   = api_ota_upload_handler,
        };
        httpd_register_uri_handler(server, &api_ota_upload_uri);

        // Additional motor management endpoints
        httpd_uri_t api_motors_reader_post_uri = {
            .uri       = "/api/motors/reader",
            .method    = HTTP_POST,
            .handler   = api_motor_management_handler,
        };
        httpd_register_uri_handler(server, &api_motors_reader_post_uri);

        httpd_uri_t api_motors_target_post_uri = {
            .uri       = "/api/motors/target",
            .method    = HTTP_POST,
            .handler   = api_motor_management_handler,
        };
        httpd_register_uri_handler(server, &api_motors_target_post_uri);

        httpd_uri_t api_mapping_post_uri = {
            .uri       = "/api/mapping",
            .method    = HTTP_POST,
            .handler   = api_motor_management_handler,
        };
        httpd_register_uri_handler(server, &api_mapping_post_uri);

        // OTA page and system info endpoints
        httpd_uri_t ota_page_uri = {
            .uri       = "/ota",
            .method    = HTTP_GET,
            .handler   = ota_page_handler,
        };
        httpd_register_uri_handler(server, &ota_page_uri);

        httpd_uri_t api_system_info_uri = {
            .uri       = "/api/system/info",
            .method    = HTTP_GET,
            .handler   = api_system_info_handler,
        };
        httpd_register_uri_handler(server, &api_system_info_uri);

        // DELETE handlers for motor management
        httpd_uri_t api_motors_target_delete_uri = {
            .uri       = "/api/motors/target/*",
            .method    = HTTP_DELETE,
            .handler   = api_motor_delete_handler,
        };
        httpd_register_uri_handler(server, &api_motors_target_delete_uri);

        httpd_uri_t api_motors_reader_delete_uri = {
            .uri       = "/api/motors/reader/*",
            .method    = HTTP_DELETE,
            .handler   = api_motor_delete_handler,
        };
        httpd_register_uri_handler(server, &api_motors_reader_delete_uri);

        httpd_uri_t api_mapping_delete_uri = {
            .uri       = "/api/mapping/*",
            .method    = HTTP_DELETE,
            .handler   = api_mapping_delete_handler,
        };
        httpd_register_uri_handler(server, &api_mapping_delete_uri);

        ESP_LOGI(TAG, "Started HTTP server on port %d", config.server_port);
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return ESP_FAIL;
}

esp_err_t stop_webserver(void) {
    if (server) {
        httpd_stop(server);
        server = NULL;
    }
    return ESP_OK;
}