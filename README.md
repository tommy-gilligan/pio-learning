# Rust Ports of C SDK PIO Examples

This is really just for my own learning.  From a brief gander, this seems to be
a good order in which to approach these examples.

1. [x] Hello
1. [x] Addition
1. [o] Blink (fails config assertions)
1. [o] PWM (fails config assertions)
1. [o] Square wave (fails config assertions, fast & wrap not implemented)
1. [o] WS2812 (clock wrong? parallel not implemented)
1. [ ] Manchester
1. [ ] Differential Manchester
1. [ ] Logic Analyzer
1. [ ] Clocked Input
1. [ ] Quadrature Encoder
1. [ ] UART TX
1. [ ] UART RX
1. [ ] IR NEC
1. [ ] I2C
1. [ ] SPI

## Resources

- <https://github.com/rp-rs/rp-hal/blob/main/rp2040-hal/examples/pio_blink.rs>
- <https://github.com/rp-rs/rp-hal/blob/main/rp2040-hal/examples/pio_dma.rs>
- <https://github.com/rp-rs/rp-hal/blob/main/rp2040-hal/examples/pio_proc_blink.rs>
- <https://github.com/rp-rs/rp-hal/blob/main/rp2040-hal/examples/pio_side_set.rs>
- <https://github.com/rp-rs/rp-hal/blob/main/rp2040-hal/examples/pio_synchronized.rs>

## Examples for which I don't have hardware
- apa102
- hub75
- onewire
- st7789_lcd
