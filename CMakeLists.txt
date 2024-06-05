cmake_minimum_required(VERSION 3.22)
project(RP2040Audio)

add_library(RP2040Audio src/RP2040Audio.cpp)

#needed?
target_sources(RP2040Audio PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src/RP2040Audio.cpp")

target_link_arduino_libraries(RP2040Audio PUBLIC core)
