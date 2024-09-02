cmake_minimum_required(VERSION 3.13)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(pico_sdk_import.cmake)

project(pico-imulib C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "src/*.[ch]")


pico_sdk_init()

#pico_enable_stdio_usb(main 0)
#pico_enable_stdio_uart(main 1)


#pico_add_extra_outputs(main)
add_library(pico-imulib
    ${SOURCES}
)

target_include_directories(pico-imulib PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        # add other folders as needed
)

target_link_libraries(pico-imulib
	pico_stdlib
	# other libs as needed
)

add_custom_target(flash
    COMMAND /bin/bash ../flash
    DEPENDS pico-imulib  
)
add_custom_target(reset
    COMMAND /bin/bash ../reset
    DEPENDS pico-imulib  
)
