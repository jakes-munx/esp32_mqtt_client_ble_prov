idf_component_register(SRCS 
                        "app_uart.c"
                    INCLUDE_DIRS 
                        "include"
                    
                    PRIV_REQUIRES
                        "app_globals"
                        "driver"
                    
                        )