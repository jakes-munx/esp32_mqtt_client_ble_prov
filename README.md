# esp32_mqtt_client_ble_prov
An ESP32 project which provisions its WiFi credentials over BLE and connects to a personal MQTT server with SSL verification. Test data published periodically.

# Make client certificate binary
python C:\Users\JacquesvdM\esp\esp-idf\components\nvs_flash\nvs_partition_generator\nvs_partition_gen.py generate certs_partition.csv certs.bin 0x3000

# Flash client certificate binary
python C:\Users\JacquesvdM\esp\esp-idf\components\partition_table\parttool.py -p COM15 -b 60800 write_partition --partition-name=certs --input=certs.bin

read back:
python C:\Users\JacquesvdM\esp\esp-idf\components\partition_table\parttool.py -p COM15 -b 60800 read_partition --partition-name=certs --output=certs_check.bin
python C:\Users\JacquesvdM\esp\esp-idf\components\partition_table\parttool.py -p COM15 -b 60800 get_partition_info  --partition-name=certs --info name, type, subtype, offset, size, encrypted


# openOCD
openocd -f board/esp32c3-ftdi.cfg
openocd -f board/esp32c3-ftdi.cfg -c "gdb_memory_map disable"
openocd -f $OPENOCD_SCRIPTS/interface/ftdi/esp32_devkitj_v1.cfg  -f $OPENOCD_SCRIPTS/target/esp32c3.cfg 
path: C:\Users\JacquesvdM\.espressif\tools\openocd-esp32\v0.12.0-esp32-20230419\openocd-esp32\share\openocd\scripts\board

# burn fuse for C3
python -m espefuse --port COM15 burn_efuse DIS_USB_JTAG