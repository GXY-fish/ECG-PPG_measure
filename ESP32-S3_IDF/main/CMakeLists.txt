idf_component_register(SRCS "main.c"
                             "max30102.c"
                             "oled.c"
                             "ads1292.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver
                                esp_driver_i2c
                                esp_driver_gpio
                                esp_driver_spi
                                esp_driver_usb_serial_jtag)
