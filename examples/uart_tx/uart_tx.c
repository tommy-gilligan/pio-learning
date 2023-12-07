/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "uart_tx.pio.h"

#include "../debug.h"

int main() {
    stdio_init_all();
    // We're going to use PIO to print "Hello, world!" on the same GPIO which we
    // normally attach UART0 to.
    const uint PIN_TX = 16;
    // This is the same as the default UART baud rate on Pico
    const uint SERIAL_BAUD = 115200;

    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &uart_tx_program);
    sleep_ms(2000);

    // Tell PIO to initially drive output-high on the selected pin, then map PIO
    // onto that pin with the IO muxes.
    pio_sm_set_pins_with_mask(pio, sm, 1u << PIN_TX, 1u << PIN_TX);
    pio_sm_set_pindirs_with_mask(pio, sm, 1u << PIN_TX, 1u << PIN_TX);
    pio_gpio_init(pio, PIN_TX);

    pio_sm_config c = uart_tx_program_get_default_config(offset);

    // OUT shifts to right, no autopull
    sm_config_set_out_shift(&c, true, false, 32);

    // We are mapping both OUT and side-set to the same pin, because sometimes
    // we need to assert user data onto the pin (with OUT) and sometimes
    // assert constant values (start/stop bit)
    sm_config_set_out_pins(&c, PIN_TX, 1);
    sm_config_set_sideset_pins(&c, PIN_TX);

    // We only need TX, so get an 8-deep FIFO!
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    // SM transmits 1 bit per 8 execution cycles.
    float div = (float)clock_get_hz(clk_sys) / (8 * SERIAL_BAUD);
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(pio, sm, offset, &c);

    print_pio_state();
    print_sm_state(SM0_BASE);

    pio_sm_set_enabled(pio, sm, true);

    while (true) {
	char *s = "Hello, world! (from PIO!)\n";

        while (*s) {
            pio_sm_put_blocking(pio, sm, (uint32_t)*s++);
        }

        sleep_ms(1000);
    }
}
