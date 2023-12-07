# Rust Ports of C SDK PIO Examples

This is really just for my own learning.  From a brief gander, this seems to be
a good order in which to approach these examples.

1. [x] Hello
1. [x] Addition
1. [x] Blink (simplified to single SM)
1. [x] PWM
1. [o] Square wave (fails config assertions, fast & wrap not implemented)
1. [x] WS2812 (parallel not implemented, patterns could use some work)
1. [x] Manchester
1. [o] Differential Manchester (fails config assertions)
1. [o] Logic Analyzer (fails config assertions, resolution is wrong)
1. [x] Clocked Input
1. [x] UART TX
1. [x] UART RX (have not looked at 'mini' example)
1. [ ] IR NEC
1. [ ] I2C
1. [ ] SPI

## Resources

- `rp2040-hal` PIO examples
    - <https://github.com/rp-rs/rp-hal/blob/main/rp2040-hal/examples/pio_blink.rs>
    - <https://github.com/rp-rs/rp-hal/blob/main/rp2040-hal/examples/pio_dma.rs>
    - <https://github.com/rp-rs/rp-hal/blob/main/rp2040-hal/examples/pio_proc_blink.rs>
    - <https://github.com/rp-rs/rp-hal/blob/main/rp2040-hal/examples/pio_side_set.rs>
    - <https://github.com/rp-rs/rp-hal/blob/main/rp2040-hal/examples/pio_synchronized.rs>
- [RP2040 Datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)

## Examples for which I don't have hardware
- apa102
- hub75
- onewire
- st7789_lcd
- quadrature encoder
