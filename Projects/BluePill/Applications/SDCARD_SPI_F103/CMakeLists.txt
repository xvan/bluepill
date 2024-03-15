get_filename_component(ProjectId ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId}.elf)

include_directories(
    Middlewares/Third_Party/FatFs/src
    Core/Inc
    FATFS/App
    FATFS/Target
    )


set(DRIVERS_HAL_LIB ${ProjectId}_DRIVERS_HAL)
FILE(GLOB_RECURSE HAL_SRC "${CMAKE_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/*.c")
list(FILTER HAL_SRC EXCLUDE REGEX ".*template\\.c$")
add_library(${DRIVERS_HAL_LIB} OBJECT ${HAL_SRC})
#target_link_libraries(${DRIVERS_HAL_LIB} DRIVERS)


# set(USB_DEVICE_LIB ${ProjectId}_USB_DEVICE)
# add_library( ${USB_DEVICE_LIB} OBJECT
#     ${CMAKE_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
#     ${CMAKE_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
#     ${CMAKE_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
# )

# set(USB_CDC_LIB ${ProjectId}_USB_CDC)
# add_library(${USB_CDC_LIB} OBJECT ${CMAKE_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c )

set(FATFS_LIB ${ProjectId}_FATFS_LIB)
add_library(${FATFS_LIB} OBJECT 
    Middlewares/Third_Party/FatFs/src/diskio.c  
    Middlewares/Third_Party/FatFs/src/ff_gen_drv.c
    Middlewares/Third_Party/FatFs/src/ff.c
    FATFS/App/fatfs.c
    FATFS/Target/user_diskio.c
    )

FILE(GLOB_RECURSE PROJECT_SRC "Core/Src/*.c")
set(TARGET ${ProjectId})
add_executable( ${ProjectId} ${PROJECT_SRC})
target_link_libraries(${ProjectId} STARTUP CMSIS_DSP ${DRIVERS_HAL_LIB} ${FATFS_LIB})
#${USB_DEVICE_LIB} ${USB_CDC_LIB}