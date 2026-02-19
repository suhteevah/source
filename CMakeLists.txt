# LatchPac Validator 3000 -- Root CMakeLists.txt (ESP-IDF v5.x)

cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(latchpac_fixture)
