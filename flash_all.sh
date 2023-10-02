#!/bin/bash

echo "UART port: $1"
echo "UART port: $2"

DEV_TYPE=$1
DEV_PORT=$2

# Write the certs and device id data in the device flash
python $IDF_PATH/components/esptool_py/esptool/esptool.py --chip $DEV_TYPE -p $DEV_PORT -b 460800 --before=default_reset --after=hard_reset write_flash --flash_mode dio --flash_freq 80m --flash_size 4MB 0x1000 build/bootloader/bootloader.bin 0x9000 build/nvs.bin 0x8000 build/partition_table/partition-table.bin 0x11000 build/ota_data_initial.bin 0x20000 build/wit102.bin