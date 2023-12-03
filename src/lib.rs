#![no_std]

use defmt::Format;
use serde::Deserialize;
use serde::Serialize;

#[derive(Serialize, Deserialize, Format, Debug, Clone, Copy, PartialEq, Eq)]
pub struct PioStateCopy {
    ctrl: u32,
    fstat: u32,
    fdebug: u32,
    flevel: u32,
    irq: u32,
    dbg_padout: u32,
    dbg_padoe: u32,
    dbg_cfginfo: u32,
    sm0_clkdiv: u32,
    sm0_execctrl: u32,
    sm0_shiftctrl: u32,
    sm0_addr: u32,
    sm0_instr: u32,
    sm0_pinctrl: u32,
}

impl PioStateCopy {
    pub unsafe fn new() -> Self {
        Self {
            ctrl: core::ptr::read_volatile(0x50200000 as *const u32),
            fstat: core::ptr::read_volatile(0x50200004 as *const u32),
            fdebug: core::ptr::read_volatile(0x50200008 as *const u32),
            flevel: core::ptr::read_volatile(0x5020000c as *const u32),
            irq: core::ptr::read_volatile(0x50200030 as *const u32),
            dbg_padout: core::ptr::read_volatile(0x5020003c as *const u32),
            dbg_padoe: core::ptr::read_volatile(0x50200040 as *const u32),
            dbg_cfginfo: core::ptr::read_volatile(0x50200044 as *const u32),
            sm0_clkdiv: core::ptr::read_volatile(0x502000c8 as *const u32),
            sm0_execctrl: core::ptr::read_volatile(0x502000cc as *const u32),
            sm0_shiftctrl: core::ptr::read_volatile(0x502000d0 as *const u32),
            sm0_addr: core::ptr::read_volatile(0x502000d4 as *const u32),
            sm0_instr: core::ptr::read_volatile(0x502000d8 as *const u32),
            sm0_pinctrl: core::ptr::read_volatile(0x502000dc as *const u32),
        }
    }

    pub fn assert_eq(expected_json: &str) {
        let expected: PioStateCopy = serde_json_core::from_str(expected_json).unwrap().0;
        defmt::assert_eq!(unsafe { PioStateCopy::new() }, expected);
    }
}
