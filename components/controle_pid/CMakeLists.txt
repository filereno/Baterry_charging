idf_component_register(
    SRCS "controle_pid.c"
    INCLUDE_DIRS "include"
    REQUIRES pid_ctrl   # espressif/pid_ctrl^0.2.0 (gerenciado via idf_component.yml)
)
