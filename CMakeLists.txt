cmake_minimum_required(VERSION 3.5)

# project name
project(STM32WB07 C ASM)

# toolchain
set(CMAKE_TOOLCHAIN_FILE "cmake/arm-none-eabi.cmake")

# select linker script
set(LINKER_SCRIPT "lnk/STM32WB07.ld")
set(BLE_STACK "ble_stack.a")

# define CPU OPTIONS
set(CPU_OPTIONS -mthumb -mcpu=cortex-m0plus)

# optimizations (-O0 -O1 -O2 -O3 -Os -Ofast -Og -flto)
set(CMAKE_C_FLAGS_DEBUG "-O0 -g -DDEBUG")
set(CMAKE_C_FLAGS_RELEASE "-Ofast -flto")
set(CMAKE_C_FLAGS_MINSIZEREL "-Os -flto")

add_compile_options(
    ${CPU_OPTIONS}
    $<$<COMPILE_LANGUAGE:C>:-std=c11>
    $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>
    $<$<COMPILE_LANGUAGE:CXX>:-fms-extensions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>
    $<$<COMPILE_LANGUAGE:CXX>:-Wold-style-cast>
    $<$<COMPILE_LANGUAGE:CXX>:-Wsuggest-override>
    -fstrict-volatile-bitfields
    -ffunction-sections

    #        -Werror
    #        -Wall
    #        -Wextra
    #        -Wcast-align
    #        -Wconversion
    #        -Wsign-conversion
    #        -Wshadow
    #        -Wlogical-op
    #        -Wsuggest-final-types
    #        -Wsuggest-final-methods
    #        -pedantic
)

include_directories(
    "inc/"
    "lib/RF/inc"
)

link_directories(
    ${CMAKE_SOURCE_DIR}
    "lib/RF/bin/"
)

# list of modules to build final firmware
add_executable(${PROJECT_NAME}
    src/vector.s
    src/startup.s

    src/GPIO.S

    src/NVIC.c
    src/sys.c
    src/WDG.c
    src/RTC.c
    src/tim.c
    src/GPIO.c
    src/EXTI.c
    src/USART.c
    src/CRC.c
    src/RNG.c
    src/SPI.c
    src/main.c

    lib/RF/src/RF.s
    lib/RF/src/cpu_context_switch.s
    lib/RF/src/init.c
    lib/RF/src/stm32wb0x_hal_radio.c
    lib/RF/src/stm32wb0x_hal_msp.c
    lib/RF/src/stm32wb0x_hal_radio_timer.c
    lib/RF/src/stm32wb0x_hal_cortex.c
    lib/RF/src/stm32wb0x_hal_rcc.c
    lib/RF/src/stm32wb0x_hal_rcc_ex.c
    lib/RF/src/stm32wb0x_hal_pwr.c
    lib/RF/src/stm32wb0x_hal_pwr_ex.c
    lib/RF/src/stm32wb0x_hal_uart.c
    lib/RF/src/stm32wb0x_hal_uart_ex.c
    lib/RF/src/stm32wb0x_hal_dma.c
    lib/RF/src/stm32wb0x_hal.c
    lib/RF/src/system_stm32wb0x.c
    lib/RF/src/stm32wb0x_hal_gpio.c
    lib/RF/src/app_entry.c
    lib/RF/src/stm32_lpm.c
    lib/RF/src/stm32wb0x_nucleo.c
    lib/RF/src/stm32_lpm_if.c
    lib/RF/src/device_context_switch.c
    lib/RF/src/osal.c
)
target_link_libraries(${PROJECT_NAME}
    ${CPU_OPTIONS}
    -T${LINKER_SCRIPT}
    -nostartfiles
    -nolibc
    m
    #-nostdlib
)
target_link_libraries(${PROJECT_NAME} ${BLE_STACK})

set_property(TARGET ${PROJECT_NAME} PROPERTY LINK_DEPENDS ${CMAKE_SOURCE_DIR}/${LINKER_SCRIPT})

include("cmake/flash.cmake")
