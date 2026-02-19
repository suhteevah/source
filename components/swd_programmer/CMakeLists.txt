# components/swd_programmer/CMakeLists.txt

idf_component_register(
    SRCS "src/swd_host.c"
    INCLUDE_DIRS "include"
    REQUIRES fixture_config
    PRIV_REQUIRES esp_timer
)
