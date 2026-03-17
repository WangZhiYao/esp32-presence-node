idf_component_register(
    SRCS "app_espnow.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_timer app_storage app_protocol app_event
)