idf_component_register(
    SRCS "pwm_esp.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_driver_mcpwm
        esp_driver_gpio
)