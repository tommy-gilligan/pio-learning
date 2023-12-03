/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
// Our assembled program:
#include "hello.pio.h"
#include "../debug.h"

int main() {
    stdio_init_all();
#ifndef PICO_DEFAULT_LED_PIN
#warning pio/hello_pio example requires a board with a regular LED
#else
    // Choose which PIO instance to use (there are two instances)
    PIO pio = pio0;

    // Our assembled program needs to be loaded into this PIO's instruction
    // memory. This SDK function will find a location (offset) in the
    // instruction memory where there is enough space for our program. We need
    // to remember this location!
    uint offset = pio_add_program(pio, &hello_program);

    // Find a free state machine on our chosen PIO (erroring if there are
    // none). Configure it to run our program, and start it, using the
    // helper function we included in our .pio file.
    uint sm = pio_claim_unused_sm(pio, true);
    pio_sm_config c = hello_program_get_default_config(offset);

    // Map the state machine's OUT pin group to one pin, namely the `pin`
    // parameter to this function.
    sm_config_set_out_pins(&c, PICO_DEFAULT_LED_PIN, 1);
    // Set this pin's GPIO function (connect PIO to the pad)
    pio_gpio_init(pio, PICO_DEFAULT_LED_PIN);
    // Set the pin direction to output at the PIO
    pio_sm_set_consecutive_pindirs(pio, sm, PICO_DEFAULT_LED_PIN, 1, true);

    // Load our configuration, and jump to the start of the program
    pio_sm_init(pio, sm, offset, &c);
    // Set the state machine running
    pio_sm_set_enabled(pio, sm, true);

    sleep_ms(5000);

    print_pio_state();

    // The state machine is now running. Any value we push to its TX FIFO will
    // appear on the LED pin.
    while (true) {
        // Blink
        pio_sm_put_blocking(pio, sm, 1);
        sleep_ms(500);
        // Blonk
        pio_sm_put_blocking(pio, sm, 0);
        sleep_ms(500);
    }
#endif
}
