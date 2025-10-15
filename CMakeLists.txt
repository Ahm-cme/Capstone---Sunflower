# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# Add compiler definition for hardcoded location mode
add_compile_definitions(USE_HARDCODED_LOCATION=1)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(Sunflower)