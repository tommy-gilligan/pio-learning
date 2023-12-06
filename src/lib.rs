#![no_std]

use core::fmt;
use defmt::Format;
use serde::{Deserialize, Deserializer, Serialize, Serializer, de::{self, Visitor}};

#[derive(Format, Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct SmStateCopy {
    #[serde(serialize_with = "bitstring_serialize", deserialize_with = "bitstring_deserialize")]
    sm_clkdiv: u32,
    #[serde(serialize_with = "bitstring_serialize", deserialize_with = "bitstring_deserialize")]
    sm_execctrl: u32,
    #[serde(serialize_with = "bitstring_serialize", deserialize_with = "bitstring_deserialize")]
    sm_shiftctrl: u32,
    #[serde(serialize_with = "bitstring_serialize", deserialize_with = "bitstring_deserialize")]
    sm_addr: u32,
    #[serde(serialize_with = "bitstring_serialize", deserialize_with = "bitstring_deserialize")]
    sm_instr: u32,
    #[serde(serialize_with = "bitstring_serialize", deserialize_with = "bitstring_deserialize")]
    sm_pinctrl: u32,
}

#[derive(Format, Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct PioStateCopy {
    #[serde(serialize_with = "bitstring_serialize", deserialize_with = "bitstring_deserialize")]
    ctrl: u32,
    #[serde(serialize_with = "bitstring_serialize", deserialize_with = "bitstring_deserialize")]
    fstat: u32,
    #[serde(serialize_with = "bitstring_serialize", deserialize_with = "bitstring_deserialize")]
    fdebug: u32,
    #[serde(serialize_with = "bitstring_serialize", deserialize_with = "bitstring_deserialize")]
    flevel: u32,
    #[serde(serialize_with = "bitstring_serialize", deserialize_with = "bitstring_deserialize")]
    irq: u32,
    #[serde(serialize_with = "bitstring_serialize", deserialize_with = "bitstring_deserialize")]
    dbg_padout: u32,
    #[serde(serialize_with = "bitstring_serialize", deserialize_with = "bitstring_deserialize")]
    dbg_padoe: u32,
    #[serde(serialize_with = "bitstring_serialize", deserialize_with = "bitstring_deserialize")]
    dbg_cfginfo: u32,
}

unsafe fn value(address: usize) -> u32 {
    core::ptr::read_volatile(address as *const u32)
}

unsafe fn bitstring(word: u32) -> serde_json_core::heapless::String<32> {
    let mut result: serde_json_core::heapless::String<32> = serde_json_core::heapless::String::new();

    for i in 0..32 {
        let bit = if (((word << i) >> 31) & 1) == 1 {
            '1'
        } else {
            '0'
        };
        result.push(bit).unwrap();
    }

    result
}

pub const SM0_BASE: usize = 0x502000c8;
pub const SM1_BASE: usize = 0x502000e0;
pub const SM2_BASE: usize = 0x502000f8;
pub const SM3_BASE: usize = 0x50200110;

impl SmStateCopy {
    /// # Safety
    /// Reads directly from registers to build up state
    pub unsafe fn new(base: usize) -> Self {
        Self {
            sm_clkdiv: value(base),
            sm_execctrl: value(base + 4),
            sm_shiftctrl: value(base + 8),
            sm_addr: value(base + 12),
            sm_instr: value(base + 16),
            sm_pinctrl: value(base + 20),
        }
    }

    pub fn print(base: usize) {
        defmt::info!("{:?}", unsafe { Self::new(base) });
    }

    pub fn assert_eq(base: usize, expected_json: &str) {
        let expected: SmStateCopy = serde_json_core::from_str(expected_json).unwrap().0;
        defmt::assert_eq!(unsafe { Self::new(base) }, expected);
    }
}

impl PioStateCopy {
    /// # Safety
    /// Reads directly from registers to build up state
    pub unsafe fn new() -> Self {
        Self {
            ctrl: value(0x50200000),
            fstat: value(0x50200004),
            fdebug: value(0x50200008),
            flevel: value(0x5020000c),
            irq: value(0x50200030),
            dbg_padout: value(0x5020003c),
            dbg_padoe: value(0x50200040),
            dbg_cfginfo: value(0x50200044),
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

struct U32Visitor;

impl<'de> Visitor<'de> for U32Visitor {
    type Value = u32;

    fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        formatter.write_str("an integer between -2^31 and 2^31")
    }

    fn visit_borrowed_str<E>(self, v: &'de str) -> Result<Self::Value, E> where E: de::Error {
        let mut i: u32 = 0;

        for c in v.chars() {
            if c == '1' {
                i |= 1;
            }
            i <<= 1;
        }
        Ok(i >> 1)
    }
}

fn bitstring_serialize<S>(t: &u32, s: S) -> Result<S::Ok, S::Error> where S: Serializer {
    unsafe {
        s.serialize_str(&bitstring(*t))
    }
}

fn bitstring_deserialize<'de, D>(d: D) -> Result<u32, D::Error> where D: Deserializer<'de> {
    d.deserialize_str(U32Visitor)
}
