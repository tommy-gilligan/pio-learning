#![no_std]

use defmt::Format;
use serde::Deserialize;
use serde::Serialize;

#[derive(Serialize, Deserialize, Format, Debug, Clone, Copy, PartialEq, Eq)]
pub struct SmStateCopy {
    sm_clkdiv: u32,
    sm_execctrl: u32,
    sm_shiftctrl: u32,
    sm_addr: u32,
    sm_instr: u32,
    sm_pinctrl: u32,
}

pub const SM0_BASE: u32 = 0x502000c8;
pub const SM1_BASE: u32 = 0x502000e0;
pub const SM2_BASE: u32 = 0x502000f8;
pub const SM3_BASE: u32 = 0x50200110;

impl SmStateCopy {
    pub unsafe fn new(base: u32) -> Self {
        Self {
            sm_clkdiv: core::ptr::read_volatile(base as *const u32),
            sm_execctrl: core::ptr::read_volatile((base + 4) as *const u32),
            sm_shiftctrl: core::ptr::read_volatile((base + 8) as *const u32),
            sm_addr: core::ptr::read_volatile((base + 12) as *const u32),
            sm_instr: core::ptr::read_volatile((base + 16) as *const u32),
            sm_pinctrl: core::ptr::read_volatile((base + 20) as *const u32),
        }
    }

    pub fn print(base: u32) {
        defmt::info!("{:?}", unsafe { Self::new(base) });
    }

    pub fn assert_eq(base: u32, expected_json: &str) {
        let expected: Self = serde_json_core::from_str(expected_json).unwrap().0;
        defmt::assert_eq!(unsafe { Self::new(base) }, expected);
    }
}

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
        }
    }

    pub fn print() {
        defmt::info!("{:?}", unsafe { PioStateCopy::new() });
    }

    pub fn assert_eq(expected_json: &str) {
        let expected: PioStateCopy = serde_json_core::from_str(expected_json).unwrap().0;
        defmt::assert_eq!(unsafe { PioStateCopy::new() }, expected);
    }
}
