idf_component_register(
    SRCS "ble_esp.c"
    INCLUDE_DIRS "include"
    REQUIRES bt nvs_flash
)