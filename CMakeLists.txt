
# (Automatically converted from project Makefile by convert_to_cmake.py.)

# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)
include(FetchContent)

set(HOMEKIT_PATH $ENV{IDF_PATH}/../esp-homekit-sdk/)

# if(DEFINED ENV{HOMEKIT_PATH})
#   set(HOMEKIT_PATH $ENV{HOMEKIT_PATH})
# else()
#   set(HOMEKIT_PATH ${CMAKE_CURRENT_LIST_DIR}/../..)
# endif(DEFINED ENV{HOMEKIT_PATH})

set(COMMON_COMPONENT_PATH ${HOMEKIT_PATH}/examples/common)

# FetchContent_Declare(
#   espidflib
#   GIT_REPOSITORY https://github.com/UncleRus/esp-idf-lib.git
# )
# FetchContent_MakeAvailable(espidflib)
#set(EXTRA_COMPONENT_DIRS /home/user/myprojects/esp/esp-idf-lib/components)

set(EXTRA_COMPONENT_DIRS ${HOMEKIT_PATH}/../esp-idf-lib/components ${HOMEKIT_PATH}/components ${HOMEKIT_PATH}/components/homekit ${COMMON_COMPONENT_PATH} ${CMAKE_CURRENT_LIST_DIR}/components )

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(air-quality-sensor)
