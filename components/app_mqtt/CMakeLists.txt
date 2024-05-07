idf_component_register(SRCS 
                        "app_mqtt.c"
                    INCLUDE_DIRS 
                        "include"
                    
                    PRIV_REQUIRES
                        "app_time"
                        "app_globals"
                    REQUIRES esp_event
                    REQUIRES mqtt
                    REQUIRES esp_netif
                    REQUIRES app_update)