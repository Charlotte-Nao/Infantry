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
set(CMAKE_PROJECT_NAME Infantry_01)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Task/Main.c
        Task/Main.h
        Application/struct_typedef.h
        Algorithm/pid/pid.c
        Algorithm/pid/pid.h
        Driver/can/can.c
        Driver/can/can.h
        Driver/LED/LED.c
        Driver/LED/LED.h
        Driver/uart/dvc_uart.c
        Driver/uart/dvc_uart.h
        Device/motor/motor.c
        Device/motor/motor.h
        Driver/remote/remote.c
        Driver/remote/remote.h
        Driver/motor_pwm/fire_control.c
        Driver/motor_pwm/fire_control.h
        Driver/ins/ist8310_i2c/ist8310_i2c.c
        Driver/ins/ist8310_i2c/ist8310_i2c.h
        Task/Auto.c
        Task/Auto.h
        Driver/ins/BMI088/BMI088driver.c
        Driver/ins/BMI088/BMI088Middleware.c
        Algorithm/MahonyAHRS/MahonyAHRS.c
        Application/mahony.c
        Application/mahony.h
        Application/motor_control.c
        Application/motor_control.h
        Application/auto_aim.c
        Application/auto_aim.h
        Task/Auto_Aim.c
        Task/Auto_Aim.h
        Driver/usb_cdc/usb_cdc.c
        Driver/usb_cdc/usb_cdc.h
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
