add_executable(pico-test
               ${PROJECT_SOURCE_DIR}/src/pico-test.cpp
               ${PROJECT_SOURCE_DIR}/src/ms5607.cpp
               ${PROJECT_SOURCE_DIR}/src/adxl375.cpp
               ${PROJECT_SOURCE_DIR}/src/iim42653.cpp
               ${PROJECT_SOURCE_DIR}/src/mmc5983ma.cpp
               ${PROJECT_SOURCE_DIR}/src/bno055.cpp
               ${PROJECT_SOURCE_DIR}/src/pa1616d.cpp
               ${PROJECT_SOURCE_DIR}/src/pwm.cpp
               ${PROJECT_SOURCE_DIR}/src/log_format.cpp
               ${PROJECT_SOURCE_DIR}/src/heartbeat.cpp
               ${PROJECT_SOURCE_DIR}/src/serial.cpp
)

pico_set_binary_type(pico-test copy_to_ram)

target_link_libraries(pico-test pico_stdlib pico_logger pico_flash pico_rand pico_multicore pico_sync hardware_i2c hardware_adc hardware_timer hardware_pwm FreeRTOS-Kernel FreeRTOS-Kernel-Heap4 libfixmath libfixmatrix libfixkalman)
target_include_directories(pico-test PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_compile_definitions(pico-test PRIVATE
        USE_FREERTOS=1
        #        DEBUG=1
        PICO_STDIO_STACK_BUFFER_SIZE=64 # use a small printf on stack buffer
)

pico_enable_stdio_usb(pico-test 1)
pico_enable_stdio_uart(pico-test 0)

pico_add_extra_outputs(pico-test)

