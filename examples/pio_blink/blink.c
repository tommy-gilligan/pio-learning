/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "blink.pio.h"
#include "../debug.h"

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq);

int main() {
    stdio_init_all();

    // todo get free sm
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &blink_program);
    printf("Loaded program at %d\n", offset);

    sleep_ms(5000);
    blink_pin_forever(pio, 0, offset, 2, 3);
    blink_pin_forever(pio, 1, offset, 6, 4);
    blink_pin_forever(pio, 2, offset, 11, 1);
    print_sm_state(SM0_BASE);
    print_sm_state(SM1_BASE);
    print_sm_state(SM2_BASE);
    print_pio_state();
}

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
    pio_sm_config c = blink_program_get_default_config(offset);
    sm_config_set_set_pins(&c, pin, 1);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (clock_get_hz(clk_sys) / (2 * freq)) - 3;
}
