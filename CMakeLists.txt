cmake_minimum_required(VERSION 3.13)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(pico_sdk_import.cmake)

project(imulib C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "src/*.[ch]")
file(GLOB_RECURSE SH2 RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "sh2/*.[ch]")

pico_sdk_init()
add_library(imulib
    src/imulib.cpp
    ${SOURCES}
    ${SH2}
)

target_include_directories(imulib PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        src
        sh2
        # add other folders as needed
)

target_link_libraries(imulib
	pico_stdlib
    hardware_irq
    hardware_i2c
	# other libs as needed
)

