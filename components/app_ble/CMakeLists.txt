idf_component_register(SRCS 
                        "app_ble.c"
                    INCLUDE_DIRS 
                        "include"

                    PRIV_REQUIRES
                        "qrcode"
                        "app_globals"
                        
                    REQUIRES esp_wifi
                    REQUIRES nvs_flash
                    REQUIRES wifi_provisioning)
