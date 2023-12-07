/**
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

#include "quadrature_encoder.pio.h"
#include "../debug.h"

// ---- quadrature encoder interface example
//
// the PIO program reads phase A/B of a quadrature encoder and increments or
// decrements an internal counter to keep the current absolute step count
// updated. At any point, the main code can query the current count by using
// the quadrature_encoder_*_count functions. The counter is kept in a full
// 32 bit register that just wraps around. Two's complement arithmetic means
// that it can be interpreted as a 32-bit signed or unsigned value, and it will
// work anyway.
//
// As an example, a two wheel robot being controlled at 100Hz, can use two
// state machines to read the two encoders and in the main control loop it can
// simply ask for the current encoder counts to get the absolute step count. It
// can also subtract the values from the last sample to check how many steps
// each wheel as done since the last sample period.
//
// One advantage of this approach is that it requires zero CPU time to keep the
// encoder count updated and because of that it supports very high step rates.

// max_step_rate is used to lower the clock of the state machine to save power
// if the application doesn't require a very high sampling rate. Passing zero
// will set the clock to the maximum
//

int main() {
    int new_value, delta, old_value = 0;

    // Base pin to connect the A phase of the encoder.
    // The B phase must be connected to the next pin
    const uint PIN_AB = 10;

    stdio_init_all();

    PIO pio = pio0;
    const uint sm = 0;

    // we don't really need to keep the offset, as this program must be loaded
    // at offset 0
    pio_add_program(pio, &quadrature_encoder_program);
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_AB, 2, 0);
    gpio_pull_up(PIN_AB);
    gpio_pull_up(PIN_AB + 1);

    pio_sm_config c = quadrature_encoder_program_get_default_config(0);

    sm_config_set_in_pins(&c, PIN_AB); // for WAIT, IN
    sm_config_set_jmp_pin(&c, PIN_AB); // for JMP
    // shift to left, autopull disabled
    sm_config_set_in_shift(&c, false, false, 32);
    // don't join FIFO's
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE);

    sm_config_set_clkdiv(&c, 1.0);

    pio_sm_init(pio, sm, 0, &c);
    pio_sm_set_enabled(pio, sm, true);

    while (1) {
        // note: thanks to two's complement arithmetic delta will always
        // be correct even when new_value wraps around MAXINT / MININT
        uint ret;
        int n;
    
        // if the FIFO has N entries, we fetch them to drain the FIFO,
        // plus one entry which will be guaranteed to not be stale
        n = pio_sm_get_rx_fifo_level(pio, sm) + 1;
        while (n > 0) {
            ret = pio_sm_get_blocking(pio, sm);
            n--;
        }
        new_value = ret;

        delta = new_value - old_value;
        old_value = new_value;

        printf("position %8d, delta %6d\n", new_value, delta);
        sleep_ms(100);
    }
}
