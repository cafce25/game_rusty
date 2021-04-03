use bitfield::bitfield;
use std::ops::{Index, IndexMut};

pub enum R8 {
    A = 1,
    C,
    B,
    E,
    D,
    L,
    H,
}

pub enum R16 {
    AF = 0,
    BC,
    DE,
    HL,
    SP,
    PC,
}

#[repr(C)]
pub struct Cpu {
    pub r: Registers,
}

impl IndexMut<R16> for Cpu {
    fn index_mut(&mut self, idx: R16) -> &mut u16 {
        unsafe { &mut self.r.r16a[idx as usize] }
    }
}

impl Index<R16> for Cpu {
    type Output = u16;
    fn index(&self, idx: R16) -> &u16 {
        unsafe { &self.r.r16a[idx as usize] }
    }
}

impl IndexMut<R8> for Cpu {
    fn index_mut(&mut self, idx: R8) -> &mut u8 {
        unsafe { &mut self.r.r8a[idx as usize] }
    }
}

impl Index<R8> for Cpu {
    type Output = u8;
    fn index(&self, idx: R8) -> &u8 {
        unsafe { &self.r.r8a[idx as usize] }
    }
}

#[repr(C)]
#[derive(Copy, Clone)]
pub union Registers {
    pub r8: Registers8,
    pub r16: Registers16,
    pub f: Flags,
    pub r8a: [u8; 8],
    pub r16a: [u16; 6],
}

impl std::fmt::Debug for Registers {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        unsafe {
            //write!(f, "{:?}{:?}", self.r8s, self.r16s)?;
            let r16 = &self.r16;
            write!(f, "Registers: {{\n")?;
            write!(
                f,
                "    r16: {{af: {:04x}, bc: {:04x}, de: {:04x}, \
                hl: {:04x}, sp: {:04x}, pc: {:04x}}},\n",
                r16.af, r16.bc, r16.de, r16.hl, r16.sp, r16.pc
            )?;
            let r8 = &self.r8;
            write!(
                f,
                "    r8: {{a: {:02x}, b: {:02x}, c: {:02x}, \
                d: {:02x}, e: {:02x}, h: {:02x}, l: {:02x}}},\n",
                r8.a, r8.b, r8.c, r8.d, r8.e, r8.h, r8.l
            )?;
            write!(
                f,
                "    z: {}, n: {}, h: {}, c: {},\n",
                &self.f.z(),
                &self.f.n(),
                &self.f.h(),
                &self.f.c()
            )?;
            write!(f, "}}")
        }
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Registers8 {
    pub _f: u8,
    pub a: u8,
    pub c: u8,
    pub b: u8,
    pub e: u8,
    pub d: u8,
    pub l: u8,
    pub h: u8,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Registers16 {
    pub af: u16,
    pub bc: u16,
    pub de: u16,
    pub hl: u16,
    pub sp: u16,
    pub pc: u16,
}

bitfield! {
    #[repr(C)]
    #[derive(Copy, Clone, Debug)]
    pub struct Flags(u16);
    z, set_z: 7;
    n, set_n: 6;
    h, set_h: 5;
    c, set_c: 4;
}

pub struct Memory<const SIZE: usize> {
    m: [u8; SIZE],
}

impl<const SIZE: usize> Memory<SIZE> {
    pub fn new() -> Self {
        Self {
            m: [0; SIZE],
        }
    }
}

impl<const SIZE: usize> Index<u16> for Memory<SIZE> {
    type Output = u8;
    fn index(&self, idx: u16) -> &u8 {
        &self.m[idx as usize]
    }
}

impl<const SIZE: usize> IndexMut<u16> for Memory<SIZE> {
    fn index_mut(&mut self, idx: u16) -> &mut u8 {
        &mut self.m[idx as usize]
    }
}
