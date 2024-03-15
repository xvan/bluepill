cmake_minimum_required(VERSION 3.10)
set(CMAKE_BUILD_TYPE Debug)

project(Tests)

include_directories(
    ${CMAKE_SOURCE_DIR}/Arduino-Interpolation/src
    ${CMAKE_SOURCE_DIR}/BasicLinearAlgebra
    ${CMAKE_SOURCE_DIR}/Arduino
)

message("${CMAKE_SOURCE_DIR}/BasicLinearAlgebra")


add_library( ARDUINO_INTERPOLATION OBJECT
    ${CMAKE_SOURCE_DIR}/Arduino-Interpolation/src/InterpolationLib.cpp        
)

add_executable(Tests 
    ${CMAKE_SOURCE_DIR}/test_original.cpp     
    ${CMAKE_SOURCE_DIR}/estimacion_original.cpp
)
target_link_libraries(Tests ARDUINO_INTERPOLATION)
