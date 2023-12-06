/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "differential_manchester.pio.h"
#include "../debug.h"

// Differential serial transmit/receive example
// Need to connect a wire from GPIO2 -> GPIO3

const uint pin_tx = 2;
const uint pin_rx = 3;

int main() {
    stdio_init_all();
    sleep_ms(5000);

    PIO pio = pio0;
    uint sm_tx = 0;
    uint sm_rx = 1;

    uint offset_tx = pio_add_program(pio, &differential_manchester_tx_program);
    uint offset_rx = pio_add_program(pio, &differential_manchester_rx_program);
    printf("Transmit program loaded at %d\n", offset_tx);
    printf("Receive program loaded at %d\n", offset_rx);

    // Configure state machines, set bit rate at 5 Mbps
    pio_sm_set_pins_with_mask(pio, sm_tx, 0, 1u << pin_tx);
    pio_sm_set_consecutive_pindirs(pio, sm_tx, pin_tx, 1, true);
    pio_gpio_init(pio, pin_tx);

    pio_sm_config c = differential_manchester_tx_program_get_default_config(offset_tx);
    sm_config_set_sideset_pins(&c, pin_tx);
    sm_config_set_out_shift(&c, true, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv(&c, 125.f / (16 * 5));
    pio_sm_init(pio, sm_tx, offset_tx + differential_manchester_tx_offset_start, &c);

    // Execute a blocking pull so that we maintain the initial line state until data is available
    pio_sm_exec(pio, sm_tx, pio_encode_pull(false, true));
    print_sm_state(SM0_BASE);

    pio_sm_set_enabled(pio, sm_tx, true);
    pio_sm_set_consecutive_pindirs(pio, sm_rx, pin_rx, 1, false);
    pio_gpio_init(pio, pin_rx);

    pio_sm_config d = differential_manchester_rx_program_get_default_config(offset_rx);
    sm_config_set_in_pins(&d, pin_rx); // for WAIT
    sm_config_set_jmp_pin(&d, pin_rx); // for JMP
    sm_config_set_in_shift(&d, true, true, 32);
    sm_config_set_fifo_join(&d, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv(&d, 125.f / (16 * 5));
    pio_sm_init(pio, sm_rx, offset_rx, &d);

    // X and Y are set to 0 and 1, to conveniently emit these to ISR/FIFO.
    pio_sm_exec(pio, sm_rx, pio_encode_set(pio_x, 1));
    pio_sm_exec(pio, sm_rx, pio_encode_set(pio_y, 0));
    print_sm_state(SM1_BASE);
    print_pio_state();
    pio_sm_set_enabled(pio, sm_rx, true);

    pio_sm_set_enabled(pio, sm_tx, false);
    pio_sm_put_blocking(pio, sm_tx, 0);
    pio_sm_put_blocking(pio, sm_tx, 0x0ff0a55a);
    pio_sm_put_blocking(pio, sm_tx, 0x12345678);
    pio_sm_set_enabled(pio, sm_tx, true);

    for (int i = 0; i < 3; ++i)
        printf("%08x\n", pio_sm_get_blocking(pio, sm_rx));
}
