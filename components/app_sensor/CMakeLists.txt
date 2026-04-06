idf_component_register(
    SRCS "app_sensor.c" "ld2412.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_driver_uart esp_event
    PRIV_REQUIRES app_event app_protocol
)
