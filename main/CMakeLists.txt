idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES app_storage app_protocol app_network app_event app_espnow app_sensor
)