/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdlib.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "../debug.h"

#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "spi.pio.h"

// This program instantiates a PIO SPI with each of the four possible
// CPOL/CPHA combinations, with the serial input and output pin mapped to the
// same GPIO. Any data written into the state machine's TX FIFO should then be
// serialised, deserialised, and reappear in the state machine's RX FIFO.

#define PIN_SCK 18
#define PIN_MOSI 16
#define PIN_MISO 16 // same as MOSI, so we get loopback

#define BUF_SIZE 20

int main() {
    stdio_init_all();
    sleep_ms(5000);

    float clkdiv = 31.25f;  // 1 MHz @ 125 clk_sys
    uint cpha0_prog_offs = pio_add_program(pio0, &spi_cpha0_program);
    uint cpha1_prog_offs = pio_add_program(pio0, &spi_cpha1_program);

    for (int cpha = 0; cpha <= 1; ++cpha) {
        for (int cpol = 0; cpol <= 1; ++cpol) {
            printf("CPHA = %d, CPOL = %d\n", cpha, cpol);

	    uint prog_offs = cpha ? cpha1_prog_offs : cpha0_prog_offs;
            pio_sm_config c = cpha ? spi_cpha1_program_get_default_config(prog_offs) : spi_cpha0_program_get_default_config(prog_offs);

            pio_sm_set_enabled(pio0, 0, false);

            sm_config_set_out_pins(&c, PIN_MOSI, 1);
            sm_config_set_in_pins(&c, PIN_MISO);
            sm_config_set_sideset_pins(&c, PIN_SCK);
            // Only support MSB-first in this example code (shift to left, auto push/pull, threshold=nbits)
            sm_config_set_out_shift(&c, false, true, 8);
            sm_config_set_in_shift(&c, false, true, 8);
            sm_config_set_clkdiv(&c, clkdiv);
        
            // MOSI, SCK output are low, MISO is input
            pio_sm_set_pins_with_mask(pio0, 0, 0, (1u << PIN_SCK) | (1u << PIN_MOSI));
            pio_sm_set_pindirs_with_mask(pio0, 0, (1u << PIN_SCK) | (1u << PIN_MOSI), (1u << PIN_SCK) | (1u << PIN_MOSI) | (1u << PIN_MISO));
            pio_gpio_init(pio0, PIN_MOSI);
            pio_gpio_init(pio0, PIN_MISO);
            pio_gpio_init(pio0, PIN_SCK);
        
            // The pin muxes can be configured to invert the output (among other things
            // and this is a cheesy way to get CPOL=1
            gpio_set_outover(PIN_SCK, cpol ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL);
            // SPI is synchronous, so bypass input synchroniser to reduce input delay.
            hw_set_bits(&pio0->input_sync_bypass, 1u << PIN_MISO);
            pio_sm_init(pio0, 0, prog_offs, &c);

	    print_pio_state();
	    print_sm_state(SM0_BASE);

            pio_sm_set_enabled(pio0, 0, true);

            static uint8_t txbuf[BUF_SIZE];
            static uint8_t rxbuf[BUF_SIZE];
            printf("TX:");
            for (int i = 0; i < BUF_SIZE; ++i) {
                txbuf[i] = rand() >> 16;
                rxbuf[i] = 0;
                printf(" %02x", (int) txbuf[i]);
            }
            printf("\n");
     
            uint8_t *src = txbuf;
            uint8_t *dst = rxbuf;
            size_t tx_remain = BUF_SIZE, rx_remain = BUF_SIZE;
            io_rw_8 *txfifo = (io_rw_8 *) &pio0->txf[0];
            io_rw_8 *rxfifo = (io_rw_8 *) &pio0->rxf[0];
            while (tx_remain || rx_remain) {
                if (tx_remain && !pio_sm_is_tx_fifo_full(pio0, 0)) {
                    *txfifo = *src++;
                    --tx_remain;
                }
                if (rx_remain && !pio_sm_is_rx_fifo_empty(pio0, 0)) {
                    *dst++ = *rxfifo;
                    --rx_remain;
                }
            }
     
            printf("RX:");
            bool mismatch = false;
            for (int i = 0; i < BUF_SIZE; ++i) {
                printf(" %02x", (int) rxbuf[i]);
                mismatch = mismatch || rxbuf[i] != txbuf[i];
            }
            if (mismatch)
                printf("\nNope\n");
            else
                printf("\nOK\n");

            sleep_ms(10);
        }
    }
}
