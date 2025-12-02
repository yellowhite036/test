#!/bin/bash

# ESP32-C6 OTA Firmware Build Script
# This script builds the firmware and creates an uploadable OTA binary

set -e

echo "🔧 ESP32-C6 OTA Firmware Build Script"
echo "======================================"

# Check if ESP-IDF is sourced
if [ -z "$IDF_PATH" ]; then
    echo "⚠️  ESP-IDF environment not found. Trying to source..."
    if [ -f "$HOME/esp/esp-idf/export.sh" ]; then
        source "$HOME/esp/esp-idf/export.sh"
    elif [ -f "$HOME/esp/v5.4.2/esp-idf/export.sh" ]; then
        source "$HOME/esp/v5.4.2/esp-idf/export.sh"
    else
        echo "❌ ESP-IDF not found. Please source the ESP-IDF environment first."
        exit 1
    fi
fi

# Set target
echo "🎯 Setting target to ESP32-C6..."
idf.py set-target esp32c6

# Configure custom partition table for OTA
echo "📋 Configuring partition table for OTA..."
cat > partitions_ota.csv << EOF
# Name,   Type, SubType, Offset,  Size, Flags
nvs,      data, nvs,     0x9000,  0x4000,
otadata,  data, ota,     0xd000,  0x2000,
phy_init, data, phy,     0xf000,  0x1000,
ota_0,    app,  ota_0,   0x10000, 0x1E0000,
ota_1,    app,  ota_1,   0x1F0000,0x1E0000,
EOF

# Update sdkconfig for OTA
echo "⚙️  Configuring OTA settings..."
echo "CONFIG_PARTITION_TABLE_CUSTOM=y" >> sdkconfig
echo "CONFIG_PARTITION_TABLE_CUSTOM_FILENAME=\"partitions_ota.csv\"" >> sdkconfig
echo "CONFIG_APP_ROLLBACK_ENABLE=y" >> sdkconfig
echo "CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y" >> sdkconfig

# Build the project
echo "🔨 Building firmware..."
idf.py build

# Check if build was successful
if [ ! -f "build/esp32c6mini-canbus.bin" ]; then
    echo "❌ Build failed - binary not found"
    exit 1
fi

# Create OTA directory
OTA_DIR="build/ota"
mkdir -p "$OTA_DIR"

# Copy the main application binary
APP_BIN="build/esp32c6mini-canbus.bin"
OTA_BIN="$OTA_DIR/firmware_ota.bin"

echo "📦 Creating OTA binary..."
cp "$APP_BIN" "$OTA_BIN"

# Get file information
FILE_SIZE=$(stat -c%s "$OTA_BIN")
FILE_HASH=$(sha256sum "$OTA_BIN" | cut -d' ' -f1)
BUILD_DATE=$(date -u +"%Y-%m-%d %H:%M:%S UTC")
VERSION=$(git describe --tags --dirty --always 2>/dev/null || echo "unknown")

# Create firmware info file
INFO_FILE="$OTA_DIR/firmware_info.json"
cat > "$INFO_FILE" << EOF
{
    "version": "$VERSION",
    "build_date": "$BUILD_DATE",
    "target": "esp32c6",
    "file_size": $FILE_SIZE,
    "sha256": "$FILE_HASH",
    "filename": "firmware_ota.bin",
    "description": "ESP32-C6 Motor Control System with OTA Support",
    "features": [
        "Motor Control via CAN Bus",
        "ESP-NOW Wireless Communication", 
        "Web-based GUI",
        "OTA Updates",
        "Safety Systems",
        "Position Monitoring"
    ]
}
EOF

# Create upload script
UPLOAD_SCRIPT="$OTA_DIR/upload_firmware.py"
cat > "$UPLOAD_SCRIPT" << 'EOF'
#!/usr/bin/env python3
"""
ESP32-C6 OTA Upload Script
Uploads firmware to ESP32-C6 via web interface
"""

import requests
import json
import sys
import os
from pathlib import Path

def upload_firmware(host, firmware_path, info_path):
    print(f"🚀 Uploading firmware to {host}")
    
    # Load firmware info
    with open(info_path, 'r') as f:
        info = json.load(f)
    
    print(f"📋 Firmware Info:")
    print(f"   Version: {info['version']}")
    print(f"   Size: {info['file_size']} bytes")
    print(f"   SHA256: {info['sha256']}")
    
    # Upload firmware
    url = f"http://{host}/api/ota/upload"
    
    with open(firmware_path, 'rb') as f:
        files = {'firmware': f}
        data = {'method': 'web'}
        
        print("📤 Uploading...")
        try:
            response = requests.post(url, files=files, data=data, timeout=300)
            
            if response.status_code == 200:
                result = response.json()
                if result.get('success'):
                    print("✅ Upload successful!")
                    print("🔄 Device will restart automatically")
                else:
                    print(f"❌ Upload failed: {result.get('error', 'Unknown error')}")
                    return False
            else:
                print(f"❌ HTTP Error: {response.status_code}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"❌ Network error: {e}")
            return False
    
    return True

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 upload_firmware.py <esp32_ip_address>")
        print("Example: python3 upload_firmware.py 192.168.1.100")
        sys.exit(1)
    
    host = sys.argv[1]
    script_dir = Path(__file__).parent
    firmware_path = script_dir / "firmware_ota.bin"
    info_path = script_dir / "firmware_info.json"
    
    if not firmware_path.exists():
        print("❌ Firmware file not found!")
        sys.exit(1)
    
    if not info_path.exists():
        print("❌ Firmware info file not found!")
        sys.exit(1)
    
    success = upload_firmware(host, firmware_path, info_path)
    sys.exit(0 if success else 1)
EOF

chmod +x "$UPLOAD_SCRIPT"

# Create ESP-NOW broadcast script
ESPNOW_SCRIPT="$OTA_DIR/broadcast_firmware.py"
cat > "$ESPNOW_SCRIPT" << 'EOF'
#!/usr/bin/env python3
"""
ESP-NOW OTA Broadcast Script
Sends firmware update via ESP-NOW broadcast
"""

import requests
import json
import sys
from pathlib import Path

def broadcast_firmware(host, firmware_path, info_path):
    print(f"📡 Broadcasting firmware via ESP-NOW from {host}")
    
    # Load firmware info
    with open(info_path, 'r') as f:
        info = json.load(f)
    
    print(f"📋 Firmware Info:")
    print(f"   Version: {info['version']}")
    print(f"   Size: {info['file_size']} bytes")
    
    # Upload and broadcast via ESP-NOW
    url = f"http://{host}/api/ota/upload"
    
    with open(firmware_path, 'rb') as f:
        files = {'firmware': f}
        data = {'method': 'espnow'}
        
        print("📡 Broadcasting to all ESP-NOW devices...")
        try:
            response = requests.post(url, files=files, data=data, timeout=600)
            
            if response.status_code == 200:
                result = response.json()
                if result.get('success'):
                    print("✅ Broadcast successful!")
                    print("🔄 All devices will update and restart")
                else:
                    print(f"❌ Broadcast failed: {result.get('error', 'Unknown error')}")
                    return False
            else:
                print(f"❌ HTTP Error: {response.status_code}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"❌ Network error: {e}")
            return False
    
    return True

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 broadcast_firmware.py <esp32_ip_address>")
        print("Example: python3 broadcast_firmware.py 192.168.1.100")
        sys.exit(1)
    
    host = sys.argv[1]
    script_dir = Path(__file__).parent
    firmware_path = script_dir / "firmware_ota.bin"
    info_path = script_dir / "firmware_info.json"
    
    if not firmware_path.exists():
        print("❌ Firmware file not found!")
        sys.exit(1)
    
    if not info_path.exists():
        print("❌ Firmware info file not found!")
        sys.exit(1)
    
    success = broadcast_firmware(host, firmware_path, info_path)
    sys.exit(0 if success else 1)
EOF

chmod +x "$ESPNOW_SCRIPT"

# Create README
README_FILE="$OTA_DIR/README.md"
cat > "$README_FILE" << EOF
# ESP32-C6 OTA Firmware Package

This package contains the firmware and tools for OTA updates.

## Files

- **firmware_ota.bin** - Main firmware binary for OTA update
- **firmware_info.json** - Firmware information and metadata
- **upload_firmware.py** - Python script for web upload
- **broadcast_firmware.py** - Python script for ESP-NOW broadcast
- **README.md** - This file

## Firmware Info

- **Version**: $VERSION
- **Build Date**: $BUILD_DATE
- **Size**: $FILE_SIZE bytes
- **SHA256**: $FILE_HASH

## Upload Methods

### Method 1: Web Interface Upload

1. Open the admin panel: http://[ESP32_IP]/admin
2. Select the firmware_ota.bin file
3. Choose "Web Upload (Current Device)"
4. Click "Start Update"

### Method 2: Python Script Upload

\`\`\`bash
python3 upload_firmware.py [ESP32_IP_ADDRESS]
\`\`\`

### Method 3: ESP-NOW Broadcast

\`\`\`bash
python3 broadcast_firmware.py [ESP32_IP_ADDRESS]
\`\`\`

## Notes

- Ensure the target device has sufficient flash memory for OTA
- The device will automatically restart after successful update
- Keep the original firmware backup for recovery
- ESP-NOW broadcast will update ALL connected devices simultaneously

## Safety

- Always test firmware in a safe environment first
- Have a recovery method ready (JTAG/USB)
- Verify firmware integrity before deployment
EOF

echo ""
echo "✅ Build Complete!"
echo "==================="
echo "📦 OTA Package Location: $OTA_DIR"
echo "📄 Firmware Binary: $(basename "$OTA_BIN")"
echo "📊 Size: $(numfmt --to=iec $FILE_SIZE)"
echo "🔢 Version: $VERSION"
echo "🔐 SHA256: $FILE_HASH"
echo ""
echo "🚀 Upload Methods:"
echo "   1. Web Interface: Open http://[ESP32_IP]/admin"
echo "   2. Python Script: python3 $OTA_DIR/upload_firmware.py [IP]"
echo "   3. ESP-NOW Broadcast: python3 $OTA_DIR/broadcast_firmware.py [IP]"
echo ""
echo "📚 See $OTA_DIR/README.md for detailed instructions"