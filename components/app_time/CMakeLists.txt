idf_component_register(SRCS 
                        "app_time.c"
                    INCLUDE_DIRS 
                        "include"
                        
                    REQUIRES esp_event
                    REQUIRES nvs_flash
                    REQUIRES esp_netif)