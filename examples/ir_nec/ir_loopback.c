/**
 * Copyright (c) 2021 mjcross
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/pio.h"
#include "hardware/clocks.h"    // for clock_get_hz()

// import the assembled PIO state machine programs
#include "nec_carrier_burst.pio.h"
#include "nec_carrier_control.pio.h"

// import the assembled PIO state machine program
#include "nec_receive.pio.h"
#include "../debug.h"

static inline void nec_carrier_burst_program_init(PIO pio, uint sm, uint offset, uint pin, float freq) {
    // Create a new state machine configuration
    //
    pio_sm_config c = nec_carrier_burst_program_get_default_config (offset);

    // Map the SET pin group to one pin, namely the `pin`
    // parameter to this function.
    //
    sm_config_set_set_pins (&c, pin, 1);

    // Set the GPIO function of the pin (connect the PIO to the pad)
    //
    pio_gpio_init (pio, pin);

    // Set the pin direction to output at the PIO
    //
    pio_sm_set_consecutive_pindirs (pio, sm, pin, 1, true);

    // Set the clock divider to generate the required frequency
    //
    float div = clock_get_hz (clk_sys) / (freq * nec_carrier_burst_TICKS_PER_LOOP);
    sm_config_set_clkdiv (&c, div);

    // Apply the configuration to the state machine
    //
    pio_sm_init (pio, sm, offset, &c);

    print_pio_state();
    print_sm_state(SM0_BASE);
    // Set the state machine running
    //
    pio_sm_set_enabled (pio, sm, true);
}

static inline void nec_carrier_control_program_init (PIO pio, uint sm, uint offset, float tick_rate, int bits_per_frame) {

    // create a new state machine configuration
    //
    pio_sm_config c = nec_carrier_control_program_get_default_config(offset);

    // configure the output shift register
    //
    sm_config_set_out_shift (&c,
                             true,       // shift right
                             false,      // disable autopull
                             bits_per_frame);

    // join the FIFOs to make a single large transmit FIFO
    //
    sm_config_set_fifo_join (&c, PIO_FIFO_JOIN_TX);

    // configure the clock divider
    //
    float div = clock_get_hz (clk_sys) / tick_rate;
    sm_config_set_clkdiv (&c, div);

    // apply the configuration to the state machine
    //
    pio_sm_init(pio, sm, offset, &c);
    print_pio_state();
    print_sm_state(SM1_BASE);

    // set the state machine running
    //
    pio_sm_set_enabled(pio, sm, true);
}

static inline void nec_receive_program_init (PIO pio, uint sm, uint offset, uint pin) {
    // Set the GPIO function of the pin (connect the PIO to the pad)
    //
    pio_gpio_init(pio, pin);

    // Set the pin direction to `input` at the PIO
    //
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, false);

    // Create a new state machine configuration
    //
    pio_sm_config c = nec_receive_program_get_default_config (offset);

    // configure the Input Shift Register
    //
    sm_config_set_in_shift (&c,
                            true,       // shift right
                            true,       // enable autopush
                            32);        // autopush after 32 bits

    // join the FIFOs to make a single large receive FIFO
    //
    sm_config_set_fifo_join (&c, PIO_FIFO_JOIN_RX);

    // Map the IN pin group to one pin, namely the `pin`
    // parameter to this function.
    //
    sm_config_set_in_pins (&c, pin);

    // Map the JMP pin to the `pin` parameter of this function.
    //
    sm_config_set_jmp_pin (&c, pin);

    // Set the clock divider to 10 ticks per 562.5us burst period
    //
    float div = clock_get_hz (clk_sys) / (10.0 / 562.5e-6);
    sm_config_set_clkdiv (&c, div);

    // Apply the configuration to the state machine
    //
    pio_sm_init (pio, sm, offset, &c);
    print_pio_state();
    print_sm_state(SM2_BASE);

    // Set the state machine running
    //
    pio_sm_set_enabled (pio, sm, true);
}

// Claim an unused state machine on the specified PIO and configure it
// to transmit NEC IR frames on the specificied GPIO pin.
//
// Returns: on success, the number of the carrier_control state machine
// otherwise -1
int nec_tx_init(PIO pio, uint pin_num) {

    // install the carrier_burst program in the PIO shared instruction space
    uint carrier_burst_offset;
    if (pio_can_add_program(pio, &nec_carrier_burst_program)) {
        carrier_burst_offset = pio_add_program(pio, &nec_carrier_burst_program);
    } else {
        return -1;
    }

    // claim an unused state machine on this PIO
    int carrier_burst_sm = pio_claim_unused_sm(pio, true);
    if (carrier_burst_sm == -1) {
        return -1;
    }

    // configure and enable the state machine
    nec_carrier_burst_program_init(pio,
                                   0,
                                   carrier_burst_offset,
                                   pin_num,
                                   38.222e3);                   // 38.222 kHz carrier

    // install the carrier_control program in the PIO shared instruction space
    uint carrier_control_offset;
    if (pio_can_add_program(pio, &nec_carrier_control_program)) {
        carrier_control_offset = pio_add_program(pio, &nec_carrier_control_program);
    } else {
        return -1;
    }

    // claim an unused state machine on this PIO
    int carrier_control_sm = pio_claim_unused_sm(pio, true);
    if (carrier_control_sm == -1) {
        return -1;
    }

    // configure and enable the state machine
    nec_carrier_control_program_init(pio,
                                     1,
                                     carrier_control_offset,
                                     2 * (1 / 562.5e-6f),        // 2 ticks per 562.5us carrier burst
                                     32);                       // 32 bits per frame

    return carrier_control_sm;
}


// Create a frame in `NEC` format from the provided 8-bit address and data
//
// Returns: a 32-bit encoded frame

// Claim an unused state machine on the specified PIO and configure it
// to receive NEC IR frames on the given GPIO pin.
//
// Returns: the state machine number on success, otherwise -1
int nec_rx_init(PIO pio, uint pin_num) {

    // disable pull-up and pull-down on gpio pin
    gpio_disable_pulls(pin_num);

    // install the program in the PIO shared instruction space
    uint offset;
    if (pio_can_add_program(pio, &nec_receive_program)) {
        offset = pio_add_program(pio, &nec_receive_program);
    } else {
        return -1;      // the program could not be added
    }

    // claim an unused state machine on this PIO
    int sm = pio_claim_unused_sm(pio, true);
    if (sm == -1) {
        return -1;      // we were unable to claim a state machine
    }

    // configure and enable the state machine
    nec_receive_program_init(pio, 2, offset, pin_num);

    return sm;
}


// Validate a 32-bit frame and store the address and data at the locations
// provided.
//
// Returns: `true` if the frame was valid, otherwise `false`
bool nec_decode_frame(uint32_t frame, uint8_t *p_address, uint8_t *p_data) {

    // access the frame data as four 8-bit fields
    //
    union {
        uint32_t raw;
        struct {
            uint8_t address;
            uint8_t inverted_address;
            uint8_t data;
            uint8_t inverted_data;
        };
    } f;

    f.raw = frame;

    // a valid (non-extended) 'NEC' frame should contain 8 bit
    // address, inverted address, data and inverted data
    if (f.address != (f.inverted_address ^ 0xff) ||
        f.data != (f.inverted_data ^ 0xff)) {
        return false;
    }

    // store the validated address and data
    *p_address = f.address;
    *p_data = f.data;

    return true;
}

// Infrared loopback example ('NEC' format)
//
// Need to connect an IR LED to GPIO 14 via a suitable series resistor (e.g. 1.5k)
// and an active-low IR detector to GPIO 15 (e.g. VS1838b)
//
// Output is sent to stdout

int main() {
    stdio_init_all();
    sleep_ms(5000);

    PIO pio = pio0;                                 // choose which PIO block to use (RP2040 has two: pio0 and pio1)
    uint tx_gpio = 14;                              // choose which GPIO pin is connected to the IR LED
    uint rx_gpio = 15;                              // choose which GPIO pin is connected to the IR detector

    // configure and enable the state machines
    int tx_sm = nec_tx_init(pio, tx_gpio);         // uses two state machines, 16 instructions and one IRQ
    int rx_sm = nec_rx_init(pio, rx_gpio);         // uses one state machine and 9 instructions

    if (tx_sm == -1 || rx_sm == -1) {
        printf("could not configure PIO\n");
        return -1;
    }

    // transmit and receive frames
    uint8_t tx_address = 0x00, tx_data = 0x00, rx_address, rx_data;
    while (true) {
        // create a 32-bit frame and add it to the transmit FIFO
        uint32_t tx_frame = tx_address | (tx_address ^ 0xff) << 8 | tx_data << 16 | (tx_data ^ 0xff) << 24;
        pio_sm_put(pio, tx_sm, tx_frame);
        printf("\nsent: %02x, %02x", tx_address, tx_data);

        // allow time for the frame to be transmitted (optional)
        sleep_ms(100);

        // display any frames in the receive FIFO
        while (!pio_sm_is_rx_fifo_empty(pio, rx_sm)) {
            uint32_t rx_frame = pio_sm_get(pio, rx_sm);

            if (nec_decode_frame(rx_frame, &rx_address, &rx_data)) {
                printf("\treceived: %02x, %02x", rx_address, rx_data);
            } else {
                printf("\treceived: %08x", rx_frame);
            }
        }

        sleep_ms(900);
        tx_data += 1;
    }
}
