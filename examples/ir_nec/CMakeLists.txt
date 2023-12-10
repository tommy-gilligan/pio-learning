add_executable (pio_ir_loopback ir_loopback.c)
pico_generate_pio_header(pio_ir_loopback ${CMAKE_CURRENT_LIST_DIR}/nec_receive.pio)
pico_generate_pio_header(pio_ir_loopback ${CMAKE_CURRENT_LIST_DIR}/nec_carrier_burst.pio)
pico_generate_pio_header(pio_ir_loopback ${CMAKE_CURRENT_LIST_DIR}/nec_carrier_control.pio)
target_link_libraries(pio_ir_loopback LINK_PUBLIC
  pico_stdlib
  hardware_pio
  json_maker
  )
pico_enable_stdio_usb(pio_ir_loopback 1)
pico_enable_stdio_uart(pio_ir_loopback 0)
pico_add_extra_outputs(pio_ir_loopback)
example_auto_set_url(pio_ir_loopback)
