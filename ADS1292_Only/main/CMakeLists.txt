idf_component_register(SRCS "main.c"
                             "ads1292.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver
                                esp_driver_gpio
                                esp_driver_spi)
