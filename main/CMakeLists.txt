idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_spi esp_lcd esp_lcd_panel_st7789 esp_tinyusb
)
