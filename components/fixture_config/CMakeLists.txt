# components/fixture_config/CMakeLists.txt

idf_component_register(
    SRCS "mock_hardware.c"
    INCLUDE_DIRS "."
    REQUIRES driver
)
