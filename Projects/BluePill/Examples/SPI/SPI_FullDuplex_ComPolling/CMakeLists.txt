get_filename_component(ProjectId ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})

include_directories(Inc)

FILE(GLOB_RECURSE HAL_SRC "${CMAKE_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/*.c")
list(FILTER HAL_SRC EXCLUDE REGEX ".*template\\.c$")
add_library(DRIVERS_HAL_${ProjectId} OBJECT ${HAL_SRC})

# add_library(USB_DEVICE OBJECT
#     ${CMAKE_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
#     ${CMAKE_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
#     ${CMAKE_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
# )

# add_library(USB_CDC OBJECT ${CMAKE_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c )

FILE(GLOB_RECURSE PROJECT_SRC "Src/*.c")
set(TARGET ${ProjectId}.elf)
add_executable( ${TARGET} ${PROJECT_SRC})
target_link_libraries(${TARGET} STARTUP DRIVERS_HAL_${ProjectId})