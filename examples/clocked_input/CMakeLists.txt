add_executable(pio_clocked_input)

pico_generate_pio_header(pio_clocked_input ${CMAKE_CURRENT_LIST_DIR}/clocked_input.pio)

target_sources(pio_clocked_input PRIVATE clocked_input.c)

target_link_libraries(pio_clocked_input PRIVATE
        pico_stdlib
        hardware_pio
        hardware_spi
	json_maker
        )

pico_enable_stdio_usb(pio_clocked_input 1)
pico_enable_stdio_uart(pio_clocked_input 0)
pico_add_extra_outputs(pio_clocked_input)

# add url via pico_set_program_url
example_auto_set_url(pio_clocked_input)
