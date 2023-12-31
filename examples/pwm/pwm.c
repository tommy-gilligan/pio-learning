/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "pwm.pio.h"
#include "../debug.h"

// Write `period` to the input shift register
void pio_pwm_set_period(PIO pio, uint sm, uint32_t period) {
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_put_blocking(pio, sm, period);
    pio_sm_exec(pio, sm, pio_encode_pull(false, false));
    pio_sm_exec(pio, sm, pio_encode_out(pio_isr, 32));

    sleep_ms(5000);
    print_sm_state(SM0_BASE);
    print_pio_state();

    pio_sm_set_enabled(pio, sm, true);
}

// Write `level` to TX FIFO. State machine will copy this into X.
void pio_pwm_set_level(PIO pio, uint sm, uint32_t level) {
    pio_sm_put_blocking(pio, sm, level);
}

int main() {
    stdio_init_all();
#ifndef PICO_DEFAULT_LED_PIN
#warning pio/pwm example requires a board with a regular LED
    puts("Default LED pin was not defined");
#else

    // todo get free sm
    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &pwm_program);
    printf("Loaded program at %d\n", offset);

    pio_gpio_init(pio, PICO_DEFAULT_LED_PIN);
    pio_sm_set_consecutive_pindirs(pio, sm, PICO_DEFAULT_LED_PIN, 1, true);
    pio_sm_config c = pwm_program_get_default_config(offset);
    sm_config_set_sideset_pins(&c, PICO_DEFAULT_LED_PIN);
    pio_sm_init(pio, sm, offset, &c);
    pio_pwm_set_period(pio, sm, (1u << 16) - 1);

    int level = 0;
    while (true) {
        // printf("Level = %d\n", level);
        pio_pwm_set_level(pio, sm, level * level);
        level = (level + 1) % 256;
        sleep_ms(10);
    }
#endif
}
