#!/bin/bash

# Source ESP-IDF environment
source $HOME/esp/esp-idf/export.sh

# Set target
idf.py set-target esp32c6

# Configure custom partition table
echo "CONFIG_PARTITION_TABLE_CUSTOM=y" >> sdkconfig
echo "CONFIG_PARTITION_TABLE_CUSTOM_FILENAME=\"partitions.csv\"" >> sdkconfig

# Build project
idf.py build