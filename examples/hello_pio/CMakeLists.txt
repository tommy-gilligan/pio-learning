add_executable(hello_pio)

pico_generate_pio_header(hello_pio ${CMAKE_CURRENT_LIST_DIR}/hello.pio)

target_sources(hello_pio PRIVATE hello.c)

target_link_libraries(hello_pio PRIVATE
        pico_stdlib
        hardware_pio
	json_maker
        )

pico_enable_stdio_usb(hello_pio 1)
pico_enable_stdio_uart(hello_pio 0)

pico_add_extra_outputs(hello_pio)

# add url via pico_set_program_url
example_auto_set_url(hello_pio)
