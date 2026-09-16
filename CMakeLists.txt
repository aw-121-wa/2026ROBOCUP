cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME chassis_motor)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Release-only experiment; keep OFF until hardware regression has passed.
option(CHASSIS_RELEASE_LTO "Enable experimental Release link-time optimization" OFF)
option(CHASSIS_TELEMETRY_ENABLE "Enable binary VOFA transmission (host RX stays enabled)" ON)
option(CHASSIS_TELEMETRY_FULL "Include full IMU diagnostics in VOFA frames" ON)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    CHASSIS_TELEMETRY_ENABLE=$<BOOL:${CHASSIS_TELEMETRY_ENABLE}>
    CHASSIS_TELEMETRY_FULL=$<BOOL:${CHASSIS_TELEMETRY_FULL}>)
if(CHASSIS_RELEASE_LTO)
    add_compile_options($<$<AND:$<CONFIG:Release>,$<COMPILE_LANGUAGE:C>>:-flto>)
    target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE $<$<CONFIG:Release>:-flto>)
    target_link_options(${CMAKE_PROJECT_NAME} PRIVATE $<$<CONFIG:Release>:-flto>)
    # Referenced by inline assembly in the Cortex-M7 PendSV port.
    target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
        $<$<CONFIG:Release>:-Wl,--undefined=vTaskSwitchContext>)
endif()

# Add STM32CubeMX generated sources
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE_NAME:${CMAKE_PROJECT_NAME}> chassis_motor.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE_NAME:${CMAKE_PROJECT_NAME}> chassis_motor.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE_NAME:${CMAKE_PROJECT_NAME}>
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    VERBATIM
)
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/bsp_dwt.c
    Core/Src/pin_config.c
    Algorithm/planner.c
    Algorithm/imu_health.c
    Algorithm/heading.c
    App/chassis_odom.c
    DRIVER/src/jy60.c
    DRIVER/src/zdt_x42s.c
    App/chassis_control.c
    App/host_command.c
    App/host_uart.c
    App/path_chassis.c
    App/path_mission.c
    App/path_ports.c
    App/rdk_link.c
    Algorithm/motion.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    DRIVER/inc App Algorithm
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
