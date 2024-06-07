idf_component_register(SRCS "sensor.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES "driver"
                    PRIV_REQUIRES "dht"
                    REQUIRES "esp_adc")
