cmake_minimum_required(VERSION 3.12)

# Change your executable name to something creative!
set(NAME dotmatrix-timer) # <-- Name your project/executable here!

include(pimoroni_pico_import.cmake)
include(pico_sdk_import.cmake)

# Gooey boilerplate
project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(SKIP_PICO_RUNTIME 1)

# Initialize the SDK
pico_sdk_init()

# Add your source files
add_executable(${NAME}
    main.cpp # <-- Add source files here!
    clocks.c
)

add_subdirectory(pico_runtime)

set_target_properties(${NAME} PROPERTIES PICO_TARGET_LINKER_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/lpbadger.ld)
pico_add_link_depend(${NAME} ${CMAKE_CURRENT_LIST_DIR}/lpbadger.ld)

#target_compile_definitions(${NAME} PRIVATE PICO_TIME_SLEEP_OVERHEAD_ADJUST_US=60)

pico_enable_stdio_uart(${NAME} 0)
pico_enable_stdio_usb(${NAME} 0)

# Include required libraries
# This assumes `pimoroni-pico` is stored alongside your project
include(common/pimoroni_i2c)
include(drivers/ltp305/ltp305)
include(libraries/breakout_dotmatrix/breakout_dotmatrix)

# Don't forget to link the libraries you need!
target_link_libraries(${NAME}
    pimoroni_i2c
    breakout_dotmatrix
    hardware_pwm
)

# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})

# Set up files for the release packages
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.uf2
    ${CMAKE_CURRENT_LIST_DIR}/README.md
    DESTINATION .
)

set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set(CPACK_GENERATOR "ZIP" "TGZ")
include(CPack)
