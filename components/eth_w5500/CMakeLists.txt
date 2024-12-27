idf_component_register(SRCS "eth_w5500.c"
                       PRIV_REQUIRES esp_driver_gpio esp_eth
                       INCLUDE_DIRS ".")
