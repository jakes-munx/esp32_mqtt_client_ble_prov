idf_component_register(SRCS 
                        "app_time.c"
                    INCLUDE_DIRS 
                        "include"
                    
                    PRIV_REQUIRES
                        "app_globals"

                    REQUIRES esp_event
                    REQUIRES nvs_flash
                    REQUIRES esp_netif)