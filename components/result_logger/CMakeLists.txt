# components/result_logger/CMakeLists.txt

idf_component_register(
    SRCS "src/csv_logger.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_timer
)
