# main/CMakeLists.txt

idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES fixture_config test_sequencer result_logger swd_programmer
    PRIV_REQUIRES esp_app_format nvs_flash
)
