add_executable(pio_addition)

pico_generate_pio_header(pio_addition ${CMAKE_CURRENT_LIST_DIR}/addition.pio)

target_sources(pio_addition PRIVATE addition.c)

target_link_libraries(pio_addition PRIVATE pico_stdlib hardware_pio json_maker)
pico_enable_stdio_usb(pio_addition 1)
pico_enable_stdio_uart(pio_addition 0)
pico_add_extra_outputs(pio_addition)

# add url via pico_set_program_url
example_auto_set_url(pio_addition)
