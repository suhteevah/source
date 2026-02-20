# components/test_sequencer/CMakeLists.txt

idf_component_register(
    SRCS "src/test_logic.c"
    INCLUDE_DIRS "include"
    REQUIRES fixture_config swd_programmer esp_timer
    PRIV_REQUIRES esp_system
)
