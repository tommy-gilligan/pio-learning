add_executable(pio_i2c_bus_scan)

pico_generate_pio_header(pio_i2c_bus_scan ${CMAKE_CURRENT_LIST_DIR}/i2c.pio)

target_sources(pio_i2c_bus_scan PRIVATE i2c_bus_scan.c)

target_link_libraries(pio_i2c_bus_scan PRIVATE pico_stdlib hardware_pio json_maker)
pico_enable_stdio_usb(pio_i2c_bus_scan 1)
pico_enable_stdio_uart(pio_i2c_bus_scan 0)
pico_add_extra_outputs(pio_i2c_bus_scan)

# add url via pico_set_program_url
example_auto_set_url(pio_i2c_bus_scan)

