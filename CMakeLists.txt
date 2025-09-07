# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Include ESP-IDF's project.cmake first
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Set the path to your custom LVGL configuration file
idf_build_set_property(C_COMPILE_OPTIONS "-DLV_CONF_PATH= ${CMAKE_SOURCE_DIR}/main/inc/lv_conf.h" APPEND)

# Define the project name
project(touch_screen_test)
