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
#include "json-maker/json-maker.h"

struct pio_state_copy {
    int ctrl;
    int fstat;
    int fdebug;
    int flevel;
    int irq;
    int dbg_padout;
    int dbg_padoe;
    int dbg_cfginfo;
    int sm0_clkdiv;
    int sm0_execctrl;
    int sm0_shiftctrl;
    int sm0_addr;
    int sm0_instr;
    int sm0_pinctrl;
};

void print_pio_state() {
    char buff[300];
    size_t len = sizeof(buff);

    char* p = buff;
    p = json_objOpen( p, NULL, &len );
    p = json_int( p, "ctrl", *((volatile int *) 0x50200000), &len );
    p = json_int( p, "fstat", *((volatile int *) 0x50200004), &len );
    p = json_int( p, "fdebug", *((volatile int *) 0x50200008), &len );
    p = json_int( p, "flevel", *((volatile int *) 0x5020000c), &len );
    p = json_int( p, "irq", *((volatile int *) 0x50200030), &len );
    p = json_int( p, "dbg_padout", *((volatile int *) 0x5020003c), &len );
    p = json_int( p, "dbg_padoe", *((volatile int *) 0x50200040), &len );
    p = json_int( p, "dbg_cfginfo", *((volatile int *) 0x50200044), &len );
    p = json_int( p, "sm0_clkdiv", *((volatile int *) 0x502000c8), &len );
    p = json_int( p, "sm0_execctrl", *((volatile int *) 0x502000cc), &len );
    p = json_int( p, "sm0_shiftctrl", *((volatile int *) 0x502000d0), &len );
    p = json_int( p, "sm0_addr", *((volatile int *) 0x502000d4), &len );
    p = json_int( p, "sm0_instr", *((volatile int *) 0x502000d8), &len );
    p = json_int( p, "sm0_pinctrl", *((volatile int *) 0x502000dc), &len );
    p = json_objClose( p, &len );
    p = json_end( p, &len );

    printf("%s\n", buff);
}

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
