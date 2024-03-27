idf_component_register(SRCS 
                        "app_ble.c"
                    INCLUDE_DIRS 
                        "include"

                    PRIV_REQUIRES
                        "qrcode"
                        
                    REQUIRES esp_wifi
                    REQUIRES nvs_flash
                    REQUIRES wifi_provisioning)
