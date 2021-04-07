use bitfield::bitfield;
use std::ops::{Index, IndexMut};

#[derive(Debug, Copy, Clone)]
pub enum R8 {
    A = 1,
    C,
    B,
    E,
    D,
    L,
    H,
}

#[derive(Debug, Copy, Clone)]
pub enum R16 {
    AF = 0,
    BC,
    DE,
    HL,
    SP,
    PC,
}

/// A struct to contain all the registers and handle the access to different combinations of bytes
///
/// ```
/// use game_rusty::peripherals::{ Cpu, Registers, Registers16, R8, R16 };
/// let mut cpu = Cpu::new();
/// cpu[R16::AF] =  0x01a0;
///
/// assert_eq!(cpu[R8::A], 0x01);
/// assert_eq!(cpu.z(), true);
/// assert_eq!(cpu.n(), false);
/// # assert_eq!(cpu.h(), true);
/// # assert_eq!(cpu.c(), false);
///
/// # cpu[R16::BC] =  0x0203;
/// # assert_eq!(cpu[R8::B], 0x02);
/// # assert_eq!(cpu[R8::C], 0x03);
///
/// # cpu[R16::DE] =  0x0405;
/// # assert_eq!(cpu[R8::D], 0x04);
/// # assert_eq!(cpu[R8::E], 0x05);
///
/// # cpu[R16::HL] =  0x0607;
/// # assert_eq!(cpu[R8::H], 0x06);
/// # assert_eq!(cpu[R8::L], 0x07);
///
/// # cpu[R8::A] = 0x1f;
/// cpu[R8::B] = 0xff;
/// cpu[R8::C] = 0x2f;
/// # cpu[R8::D] = 0x3f;
/// # cpu[R8::E] = 0x4f;
/// # cpu[R8::H] = 0x5f;
/// # cpu[R8::L] = 0x6f;
/// # assert_eq!(cpu[R16::AF], 0x1fa0);
/// assert_eq!(cpu[R16::BC], 0xff2f);
/// # assert_eq!(cpu[R16::DE], 0x3f4f);
/// # assert_eq!(cpu[R16::HL], 0x5f6f);
/// ```                         
//// cpu[R16::AF] = 0xffff;
//// assert_eq!(cpu[R16::AF], 0xfff0);
//// this is not really an issue since only pop can write to AF and we work around it there
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Cpu {
    pub r: Registers,
    pub i: bool,
}

impl Cpu {
    pub fn new() -> Self {
        Self {
            i: false,
            r: Registers {
                r16: Registers16 {
                    af: 0x0,
                    bc: 0x0,
                    de: 0x0,
                    hl: 0x0,
                    sp: 0x0,
                    pc: 0x0,
                },
            },
        }
    }

    ///and setting the flags
    pub fn and(&mut self, b: u8) {
        self[R8::A] &= b;
        self.set_z(self[R8::A] == 0);
        self.set_n(false);
        self.set_h(true);
        self.set_c(false);
    }

    ///xor setting the flags
    pub fn xor(&mut self, b: u8) {
        self[R8::A] ^= b;
        self.set_z(self[R8::A] == 0);
        self.set_n(false);
        self.set_h(false);
        self.set_c(false);
    }

    ///or setting the flags
    pub fn or(&mut self, b: u8) {
        self[R8::A] |= b;
        self.set_z(self[R8::A] == 0);
        self.set_n(false);
        self.set_h(false);
        self.set_c(false);
    }

    ///sub setting the flags
    pub fn sub(&mut self, b: u8, respect_carry: bool) {
        let a = self[R8::A];
        let respect_carry = respect_carry && self.c();
        let (res, borrow) = {
            let (mut res, mut borrow) = a.overflowing_sub(b);
            if respect_carry {
                let (r, c1) = res.overflowing_sub(1);
                res = r;
                borrow |= c1;
            }
            (res, borrow)
        };
        let h = (a & 0xf) < ((b & 0xf) + if respect_carry { 1 } else { 0 });

        self[R8::A] = res;
        self.set_z(res == 0);
        self.set_n(true);
        self.set_c(borrow);
        self.set_h(h);
    }

    ///add setting the flags
    pub fn add(&mut self, b: u8, respect_carry: bool) {
        let a = self[R8::A];
        let respect_carry = respect_carry && self.c();
        let (res, carry) = {
            let (mut res, mut carry) = a.overflowing_add(b);
            if respect_carry {
                let (r, c1) = res.overflowing_add(1);
                res = r;
                carry |= c1;
            }
            (res, carry)
        };
        let h = ((a & 0xf) + (b & 0xf) + if respect_carry { 1 } else { 0 }) & 0x10 != 0;

        self[R8::A] = res;
        self.set_z(res == 0);
        self.set_n(false);
        self.set_c(carry);
        self.set_h(h);
    }

    pub fn z(&self) -> bool {
        unsafe { self.r.f.z() }
    }
    pub fn n(&self) -> bool {
        unsafe { self.r.f.n() }
    }
    pub fn h(&self) -> bool {
        unsafe { self.r.f.h() }
    }
    pub fn c(&self) -> bool {
        unsafe { self.r.f.c() }
    }
    pub fn i(&self) -> bool {
        self.i
    }

    pub fn set_z(&mut self, val: bool) {
        unsafe { self.r.f.set_z(val) }
    }
    pub fn set_n(&mut self, val: bool) {
        unsafe { self.r.f.set_n(val) }
    }
    pub fn set_h(&mut self, val: bool) {
        unsafe { self.r.f.set_h(val) }
    }
    pub fn set_c(&mut self, val: bool) {
        unsafe { self.r.f.set_c(val) }
    }
    pub fn set_i(&mut self, val: bool) {
        self.i = val;
    }
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
            //write!(f, "{:?}{:?}", self.r8s, self.r16a)?;
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
        Self { m: [0; SIZE] }
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
