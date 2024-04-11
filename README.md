# esp32_mqtt_client_ble_prov
An ESP32 project which provisions its WiFi credentials over BLE and connects to a personal MQTT server with SSL verification. Test data published periodically.

# Make client certificate binary
python C:\Users\JacquesvdM\esp\esp-idf\components\nvs_flash\nvs_partition_generator\nvs_partition_gen.py generate certs_partition.csv certs.bin 0x3000

# Flash client certificate binary
python C:\Users\JacquesvdM\esp\esp-idf\components\partition_table\parttool.py -p COM5 -b 60800 write_partition --partition-name=certs --input=certs.bin

read back:
python C:\Users\JacquesvdM\esp\esp-idf\components\partition_table\parttool.py -p COM5 -b 60800 read_partition --partition-name=certs --output=certs_check.bin
python C:\Users\JacquesvdM\esp\esp-idf\components\partition_table\parttool.py -p COM5 -b 60800 get_partition_info  --partition-name=certs --info name, type, subtype, offset, size, encrypted
