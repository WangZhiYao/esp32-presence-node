idf_component_register(
    SRCS "app_sensor.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_driver_i2c
)
