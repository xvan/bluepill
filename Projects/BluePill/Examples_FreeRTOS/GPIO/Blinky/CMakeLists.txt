message(STATUS "Current Source Dir: ${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "Main Source Dir: ${CMAKE_SOURCE_DIR}")
get_filename_component(ProjectId ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId}.elf)


include_directories(${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/CMSIS/RTOS2/FreeRTOS/Include)
set(FREERTOS_CONFIG_FILE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Inc CACHE STRING "Path where FreeRTOSConfig.h is found")
set(FREERTOS_PORT GCC_ARM_CM3 CACHE STRING "FreeRTOS PORT")

add_subdirectory(${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source ${CMAKE_CURRENT_BINARY_DIR}/FreeRTOS)

FILE(GLOB_RECURSE PROJECT_SRC "Src/*.c")
set(TARGET ${ProjectId})
add_executable( ${ProjectId} ${PROJECT_SRC})
target_link_libraries(${ProjectId} $<TARGET_OBJECTS:STARTUP> $<TARGET_OBJECTS:DRIVERS> freertos_kernel)
target_include_directories(${ProjectId} PRIVATE Inc)

