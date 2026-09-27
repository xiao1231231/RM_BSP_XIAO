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
set(CMAKE_PROJECT_NAME RM_F407)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        ${CMAKE_CURRENT_LIST_DIR}/User_File/System/Init/system_init.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/System/Timestamp/sys_timestamp.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/System/debug/sys_debug.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/System/callback/callback.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Device/Onboard/Buzzer/buzzer.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Device/Onboard/LED/led.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Middleware/BSP/UART/bsp_uart.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Middleware/BSP/SPI/bsp_spi.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Task/TIM_1ms_Task.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Task/BMI088_Task.cpp

        # ── BMI088 驱动（移植自 H7_BSP / USTC-RoboWalker）──
        # 结构：Gyro（FIFO 读取 + 时间戳重建 + 样本队列）
        #       Accel（数据 + 温度读取，温控本轮未移植）
        #       顶层 bsp_bmi088（三路状态机 + SPI 恢复 + VQF 姿态解算）
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Device/Onboard/BMI088/Gyro/bsp_bmi088_gyro.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Device/Onboard/BMI088/Accel/bsp_bmi088_accel.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Device/Onboard/BMI088/bsp_bmi088.cpp

        # ── 姿态解算层 ──
        # sys_attitude: BMI088 初始化 + VQF 姿态滤波（含在线零偏估计）
        # alg_matrix_cmsis_compat: 只实现被用到的 8 个矩阵/三角函数，
        #                          替代整个 CMSIS-DSP —— VQF 依赖它
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Middleware/Algorithm/Matrix/alg_matrix_cmsis_compat.c
        ${CMAKE_CURRENT_LIST_DIR}/User_File/System/INS/sys_attitude.cpp

        # ── VQF 姿态滤波器 ──
        # 算法本体来自 TU Berlin 的 Daniel Laidig（2021，MIT，见同目录 LICENSE），
        # C++ 类化封装与数学库来自 H7_BSP / yssickjgd。
        # ★ VQF 自带在线零偏估计 —— 这是 6 轴（无磁力计）方案下
        #   抑制偏航漂移的关键。
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Middleware/Algorithm/VQF/alg_basic.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Middleware/Algorithm/VQF/alg_matrix.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Middleware/Algorithm/VQF/alg_quaternion.cpp
        ${CMAKE_CURRENT_LIST_DIR}/User_File/Middleware/Algorithm/VQF/alg_filter_vqf.cpp
)


# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        User_File/System/Init
        User_File/System/Timestamp
        User_File/System/debug
        User_File/System/callback
        User_File/Device/Onboard/LED
        User_File/Device/Onboard/Buzzer
        User_File/Middleware/BSP/UART
        User_File/Middleware/BSP/SPI

        # ── BMI088 + 姿态解算 ──
        User_File/Device/Onboard/BMI088
        User_File/Device/Onboard/BMI088/Gyro
        User_File/Device/Onboard/BMI088/Accel
        User_File/Middleware/Algorithm/Matrix
        User_File/Middleware/Algorithm/VQF
        User_File/System/INS
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
# 让 printf 家族支持浮点格式化（nano.specs 下必须显式开启）
# 不加的话 %.3f 会打印出空值/乱码，而且不报任何错
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
        -u _printf_float
)