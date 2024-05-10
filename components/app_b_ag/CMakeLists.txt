idf_component_register(SRCS 
                        "app_b_ag.c"
                    INCLUDE_DIRS 
                        "include"
                    
                    PRIV_REQUIRES
                        "app_globals"
                        "app_uart")