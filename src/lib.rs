pub mod instruction;
pub mod peripherals;

use instruction::{Condition, Instruction as I};
use peripherals::{Cpu, Memory, R16, R8};
use std::ops::{Index, IndexMut};

pub struct GameRusty {
    cpu: Cpu,
    mem: Memory,
}

impl Index<R8> for GameRusty {
    type Output = u8;
    fn index(&self, idx: R8) -> &u8 {
        &self.cpu[idx]
    }
}
impl IndexMut<R8> for GameRusty {
    fn index_mut(&mut self, idx: R8) -> &mut u8 {
        &mut self.cpu[idx]
    }
}

impl Index<R16> for GameRusty {
    type Output = u16;
    fn index(&self, idx: R16) -> &u16 {
        &self.cpu[idx]
    }
}
impl IndexMut<R16> for GameRusty {
    fn index_mut(&mut self, idx: R16) -> &mut u16 {
        &mut self.cpu[idx]
    }
}

impl GameRusty {
    pub fn new() -> Self {
        Self {
            cpu: Cpu::new(),
            mem: Memory::new(),
        }
    }

    /// advance the machine one step
    pub fn update(&mut self) {
        let instruction = self.next_instruction();
        match instruction {
            //8-bit load
            I::LD_R_R(t, s) => self[t] = self[s],
            I::LD_R_N(t, s) => self[t] = s,
            I::LD_iR_R(t, s) => self.indirect_set(t, self[s]),
            I::LD_iR_N(t, s) => self.indirect_set(t, s),
            I::LD_R_iR(t, s) => self[t] = self.indirect(s),
            I::LD_R_iN(t, s) => self[t] = self.mem.get(s),
            I::LD_iN_R(t, s) => self.mem.set(t, self[s]),
            I::LDI_iR_R(t, s) => {
                self.indirect_set(t, self[s]);
                self[t] += 1;
            }
            I::LDI_R_iR(t, s) => {
                self[t] = self.indirect(s);
                self[s] += 1;
            }
            I::LDD_iR_R(t, s) => {
                self.indirect_set(t, self[s]);
                self[t] -= 1;
            }
            I::LDD_R_iR(t, s) => {
                self[t] = self.indirect(s);
                self[s] -= 1;
            }

            //16-bit load
            I::LD_R16_N(t, s) => self[t] = s,
            I::LD_iN_R16(t, s) => self.write_u16(t, self[s]),
            I::LD_R16_R16(t, s) => self[t] = self[s],
            I::PUSH(r) => {
                self[R16::SP] -= 2;
                self.write_u16(self[R16::SP], self[r]);
            }
            I::POP(r) => {
                self[r] = self.read_u16(self[R16::SP]);
                // fix the low nibble of AF which always should read as 0
                if let R16::AF = r {
                    self[r] &= 0xfff0;
                }
                self[R16::SP] += 2;
            }

            //8-bit arithmetic/logic
            I::ADD_R(r) => self.cpu.add(self[r], false),
            I::ADD_N(n) => self.cpu.add(n, false),
            I::ADD_iR(r) => self.cpu.add(self.indirect(r), false),
            I::ADC_R(r) => self.cpu.add(self[r], true),
            I::ADC_N(n) => self.cpu.add(n, true),
            I::ADC_iR(r) => self.cpu.add(self.indirect(r), true),

            I::SUB_R(r) => self.cpu.sub(self[r], false),
            I::SUB_N(n) => self.cpu.sub(n, false),
            I::SUB_iR(r) => self.cpu.sub(self.indirect(r), false),
            I::SBC_R(r) => self.cpu.sub(self[r], true),
            I::SBC_N(n) => self.cpu.sub(n, true),
            I::SBC_iR(r) => self.cpu.sub(self.indirect(r), true),

            I::AND_R(r) => self.cpu.and(self[r]),
            I::AND_N(n) => self.cpu.and(n),
            I::AND_iR(r) => self.cpu.and(self.indirect(r)),

            I::OR_R(r) => self.cpu.or(self[r]),
            I::OR_N(n) => self.cpu.or(n),
            I::OR_iR(r) => self.cpu.or(self.indirect(r)),

            I::XOR_R(r) => self.cpu.xor(self[r]),
            I::XOR_N(n) => self.cpu.xor(n),
            I::XOR_iR(r) => self.cpu.xor(self.indirect(r)),
            I::CMP_R(r) => {
                let tmp = self[R8::A];
                self.cpu.sub(self[r], false);
                self[R8::A] = tmp;
            }
            I::CMP_N(n) => {
                let tmp = self[R8::A];
                self.cpu.sub(n, false);
                self[R8::A] = tmp;
            }
            I::CMP_iR(r) => {
                let tmp = self[R8::A];
                self.cpu.sub(self.indirect(r), false);
                self[R8::A] = tmp;
            }

            I::INC_R(r) => {
                let a = self[r];
                self.set_h((a & 0xf) == 0xf);
                let res = a.wrapping_add(1);
                self.set_z(res == 0);
                self.set_n(false);
                self[r] = res;
            }
            I::INC_iR(r) => {
                let a = self.indirect(r);
                self.set_h((a & 0xf) == 0xf);
                let res = a.wrapping_add(1);
                self.set_z(res == 0);
                self.set_n(false);
                self.indirect_set(r, res);
            }

            I::DEC_R(r) => {
                let a = self[r];
                self.set_h((a & 0xf) == 0);
                let res = a.wrapping_sub(1);
                self.set_z(res == 0);
                self.set_n(true);
                self[r] = res;
            }
            I::DEC_iR(r) => {
                let a = self.indirect(r);
                self.set_h((a & 0xf) == 0);
                let res = a.wrapping_sub(1);
                self.set_z(res == 0);
                self.set_n(true);
                self.indirect_set(r, res);
            }

            I::DAA => {
                let n = self.n();
                if self[R8::A] & 0xf > 9 || self.h() {
                    self.cpu.add(0x6, false);
                }
                if self[R8::A] >> 4 & 0xf > 9 || self.c() {
                    self.cpu.add(0x60, false);
                }
                self.set_z(self[R8::A] == 0);
                self.set_h(false);
                self.set_n(n);
            }
            I::CPL => {
                self[R8::A] = !self[R8::A];
                self.set_n(true);
                self.set_h(true);
            }

            //16-bit arithmetic/logic
            I::ADD_R16(r) => {
                let a = self[R16::HL];
                let b = self[r];
                let h = ((a & 0xfff) + (b & 0xfff)) & 0x1000 != 0;
                let (res, c) = a.overflowing_add(b);
                self[R16::HL] = res;
                self.set_n(false);
                self.set_h(h);
                self.set_c(c);
            }
            I::INC_R16(r) => self[r] = self[r].wrapping_add(1),
            I::DEC_R16(r) => self[r] = self[r].wrapping_sub(1),
            I::ADD_SP_D(d) => {
                let a_sav = self[R8::A];
                self[R8::A] = (self[R16::SP] & 0xff) as u8;
                if d < 0 {
                    self.cpu.sub((-d) as u8, false);
                    self[R16::SP] = self[R16::SP].wrapping_sub(-d as u16);
                } else {
                    self.cpu.add(d as u8, false);
                    self[R16::SP] = self[R16::SP].wrapping_add(d as u16);
                }
                self.set_z(false);
                self.set_n(false);
                self[R8::A] = a_sav;
            }
            I::LD_HL_SP_D(d) => {
                let a_sav = self[R8::A];
                self[R8::A] = (self[R16::SP] & 0xff) as u8;
                if d < 0 {
                    self.cpu.sub(-d as u8, false);
                    self[R16::HL] = self[R16::SP].wrapping_sub(-d as u16);
                } else {
                    self.cpu.add(d as u8, false);
                    self[R16::HL] = self[R16::SP].wrapping_add(d as u16);
                }
                self.set_z(false);
                self.set_n(false);
                self[R8::A] = a_sav;
            }

            //rotate and shift
            I::RLCA => {
                let c = self[R8::A] >> 7;
                self.set_z(false);
                self.set_n(false);
                self.set_h(false);
                self.set_c(c != 0);
                self[R8::A] = (self[R8::A] << 1) | c;
            }
            I::RLA => {
                let c = self[R8::A] >> 7;
                self[R8::A] = (self[R8::A] << 1) | (self.c() as u8);
                self.set_z(false);
                self.set_n(false);
                self.set_h(false);
                self.set_c(c != 0);
            }
            I::RRCA => {
                let c = self[R8::A] & 1;
                self[R8::A] = self[R8::A] >> 1 | c << 7;
                self.set_z(false);
                self.set_n(false);
                self.set_h(false);
                self.set_c(c != 0);
            }
            I::RRA => {
                let c = self[R8::A] & 1;
                self[R8::A] = (self[R8::A] >> 1) | ((self.c() as u8) << 7);
                self.set_z(false);
                self.set_n(false);
                self.set_h(false);
                self.set_c(c != 0);
            }

            I::RLC_R(r) => {
                let c = self[r] >> 7;
                self.set_z(self[r] == 0);
                self.set_n(false);
                self.set_h(false);
                self.set_c(c != 0);
                self[r] = (self[r] << 1) | c;
            }
            I::RL_R(r) => {
                let c = self[r] >> 7;
                self[r] = (self[r] << 1) | (self.c() as u8);
                self.set_z(self[r] == 0);
                self.set_n(false);
                self.set_h(false);
                self.set_c(c != 0);
            }
            I::RRC_R(r) => {
                let c = self[r] & 1;
                self[r] = self[r] >> 1 | c << 7;
                self.set_z(self[r] == 0);
                self.set_n(false);
                self.set_h(false);
                self.set_c(c != 0);
            }
            I::RR_R(r) => {
                let c = self[r] & 1;
                self[r] = (self[r] >> 1) | ((self.c() as u8) << 7);
                self.set_z(self[r] == 0);
                self.set_n(false);
                self.set_h(false);
                self.set_c(c != 0);
            }

            I::RLC_iHL => {
                let c = self.indirect(R16::HL) >> 7;
                self.set_z(self.indirect(R16::HL) == 0);
                self.set_n(false);
                self.set_h(false);
                self.set_c(c != 0);
                self.indirect_set(R16::HL, (self.indirect(R16::HL) << 1) | c);
            }
            I::RL_iHL => {
                let c = self.indirect(R16::HL) >> 7;
                self.indirect_set(R16::HL, (self.indirect(R16::HL) << 1) | (self.c() as u8));
                self.set_z(self.indirect(R16::HL) == 0);
                self.set_n(false);
                self.set_h(false);
                self.set_c(c != 0);
            }
            I::RRC_iHL => {
                let c = self.indirect(R16::HL) & 1;
                self.indirect_set(R16::HL, self.indirect(R16::HL) >> 1 | c << 7);
                self.set_z(self.indirect(R16::HL) == 0);
                self.set_n(false);
                self.set_h(false);
                self.set_c(c != 0);
            }
            I::RR_iHL => {
                let c = self.indirect(R16::HL) & 1;
                self.indirect_set(R16::HL, (self.indirect(R16::HL) >> 1) | ((self.c() as u8) << 7));
                self.set_z(self.indirect(R16::HL) == 0);
                self.set_n(false);
                self.set_h(false);
                self.set_c(c != 0);
            }

            I::SLA_R(r) => {
                self.set_c(self[r] & 0x80 != 0);
                self[r] = self[r] << 1;
                self.set_z(self[r] == 0);
                self.set_n(false);
                self.set_h(false);
            }
            I::SRA_R(r) => {
                self.set_c(self[r] & 0x01 != 0);
                self[r] = (self[r] as i8 >> 1) as u8;
                self.set_z(self[r] == 0);
                self.set_n(false);
                self.set_h(false);
            }
            I::SRL_R(r) => {
                self.set_c(self[r] & 0x01 != 0);
                self[r] = self[r] >> 1;
                self.set_z(self[r] == 0);
                self.set_n(false);
                self.set_h(false);
            }
            I::SWAP_R(r) => {
                self[r] = self[r] >> 4 | self[r] << 4;
                self.set_z(self[r] == 0);
                self.set_n(false);
                self.set_h(false);
                self.set_c(false);
            }

            I::SLA_iHL => {
                self.set_c(self.indirect(R16::HL) & 0x80 != 0);
                self.indirect_set(R16::HL, self.indirect(R16::HL) << 1);
                self.set_z(self.indirect(R16::HL) == 0);
                self.set_n(false);
                self.set_h(false);
            }
            I::SRA_iHL => {
                self.set_c(self.indirect(R16::HL) & 0x01 != 0);
                self.indirect_set(R16::HL, (self.indirect(R16::HL) as i8 >> 1) as u8);
                self.set_z(self.indirect(R16::HL) == 0);
                self.set_n(false);
                self.set_h(false);
            }
            I::SRL_iHL => {
                self.set_c(self.indirect(R16::HL) & 0x01 != 0);
                self.indirect_set(R16::HL, self.indirect(R16::HL) >> 1);
                self.set_z(self.indirect(R16::HL) == 0);
                self.set_n(false);
                self.set_h(false);
            }
            I::SWAP_iHL => {
                self.indirect_set(R16::HL, 
                    self.indirect(R16::HL) >> 4 | self.indirect(R16::HL) << 4);
                self.set_z(self.indirect(R16::HL) == 0);
                self.set_n(false);
                self.set_h(false);
                self.set_c(false);
            }

            //1-bit operations
            I::BIT_N_R(n, r) => {
                self.set_z(self[r] & (1 << n) == 0);
                self.set_n(false);
                self.set_h(true);
            }
            I::BIT_N_iHL(n) => {
                self.set_z(self.indirect(R16::HL) & (1 << n) == 0);
                self.set_n(false);
                self.set_h(true);
            }
            I::SET_N_R(n, r) => self[r] |= 1 << n,
            I::SET_N_iHL(n) => self.indirect_set(R16::HL, self.indirect(R16::HL) | 1 << n),
            I::RES_N_R(n, r) => self[r] &= !(1 << n),
            I::RES_N_iHL(n) => self.indirect_set(R16::HL, self.indirect(R16::HL) & !(1 << n)),

            //control
            I::CCF => {
                self.set_n(false);
                self.set_h(false);
                self.set_c(!self.c());
            }
            I::SCF => {
                self.set_n(false);
                self.set_h(false);
                self.set_c(true);
            }
            I::NOP | I::HALT | I::STOP => {}
            I::DI => self.cpu.i = false,
            I::EI => self.cpu.i = true,

            //jumps
            I::JP_N(n) => self[R16::PC] = n,
            I::JP_HL => self[R16::PC] = self[R16::HL],
            I::JP_C_N(c, n) => {
                if self.check(c) {
                    self[R16::PC] = n
                }
            }

            I::JR_D(d) => {
                let mut n = self[R16::PC] as i32;
                n += d as i32;
                self[R16::PC] = (n & 0xffff) as u16;
            }
            I::JR_C_D(c, d) => {
                let mut n = self[R16::PC] as i32;
                n += d as i32;
                let n = (n & 0xffff) as u16;
                if self.check(c) {
                    self[R16::PC] = n
                }
            }
            I::CALL_N(n) => {
                self[R16::SP] -= 2;
                self.write_u16(self[R16::SP], self[R16::PC]);
                self[R16::PC] = n;
            }
            I::CALL_C_N(c, n) => {
                if self.check(c) {
                    self[R16::SP] -= 2;
                    self.write_u16(self[R16::SP], self[R16::PC]);
                    self[R16::PC] = n;
                }
            }
            I::RET => {
                self[R16::PC] = self.read_u16(self[R16::SP]);
                self[R16::SP] += 2;
            }
            I::RET_C(c) => {
                if self.check(c) {
                    self[R16::PC] = self.read_u16(self[R16::SP]);
                    self[R16::SP] += 2;
                }
            }
            I::RETI => {
                self[R16::PC] = self.read_u16(self[R16::SP]);
                self[R16::SP] += 2;
                self.cpu.i = true;
            }
            I::RST(n) => {
                self[R16::SP] -= 2;
                self.write_u16(self[R16::SP], self[R16::PC]);
                self[R16::PC] = n as u16;
            }
        }
    }
    fn check(&self, c: Condition) -> bool {
        match c {
            Condition::NZ => !self.z(),
            Condition::Z => self.z(),
            Condition::NC => !self.c(),
            Condition::C => self.c(),
        }
    }

    ///read 16-bit value from memory
    fn read_u16(&mut self, addr: u16) -> u16 {
        self.mem.get(addr) as u16 | (self.mem.get(addr + 1) as u16) << 8
    }

    ///write 16-bit value to memory
    fn write_u16(&mut self, addr: u16, val: u16) {
        self.mem.set(addr, (val & 0xff) as u8);
        self.mem.set(addr + 1, (val >> 8) as u8);
    }

    /// read immediate 16bit value from current position in memory while adjusting PC
    pub fn imm16(&mut self) -> u16 {
        let imm = self.read_u16(self[R16::PC]);
        self[R16::PC] += 2;
        imm
    }

    /// read immediate 8bit value from current position in memory while adjusting PC
    pub fn imm_u8(&mut self) -> u8 {
        let imm = self.indirect(R16::PC);
        self[R16::PC] += 1;
        imm
    }

    /// read immediate 8bit value from current position in memory while adjusting PC
    pub fn imm_i8(&mut self) -> i8 {
        let imm = self.indirect(R16::PC);
        self[R16::PC] += 1;
        imm as i8
    }

    /// advances pc and returns the next instruction
    pub fn next_instruction(&mut self) -> I {
        let opcode = self.imm_u8();
        match opcode {
            0x00 => I::NOP,
            0x01 => I::LD_R16_N(R16::BC, self.imm16()),
            0x02 => I::LD_iR_R(R16::BC, R8::A),
            0x03 => I::INC_R16(R16::BC),
            0x04 => I::INC_R(R8::B),
            0x05 => I::DEC_R(R8::B),
            0x06 => I::LD_R_N(R8::B, self.imm_u8()),
            0x07 => I::RLCA,
            0x08 => I::LD_iN_R16(self.imm16(), R16::SP),
            0x09 => I::ADD_R16(R16::BC),
            0x0a => I::LD_R_iR(R8::A, R16::BC),
            0x0b => I::DEC_R16(R16::BC),
            0x0c => I::INC_R(R8::C),
            0x0d => I::DEC_R(R8::C),
            0x0e => I::LD_R_N(R8::C, self.imm_u8()),
            0x0f => I::RRCA,

            0x10 => I::STOP,
            0x11 => I::LD_R16_N(R16::DE, self.imm16()),
            0x12 => I::LD_iR_R(R16::DE, R8::A),
            0x13 => I::INC_R16(R16::DE),
            0x14 => I::INC_R(R8::D),
            0x15 => I::DEC_R(R8::D),
            0x16 => I::LD_R_N(R8::D, self.imm_u8()),
            0x17 => I::RLA,
            0x18 => I::JR_D(self.imm_i8()),
            0x19 => I::ADD_R16(R16::DE),
            0x1a => I::LD_R_iR(R8::A, R16::DE),
            0x1b => I::DEC_R16(R16::DE),
            0x1c => I::INC_R(R8::E),
            0x1d => I::DEC_R(R8::E),
            0x1e => I::LD_R_N(R8::E, self.imm_u8()),
            0x1f => I::RRA,

            0x20 => I::JR_C_D(Condition::NZ, self.imm_i8()),
            0x21 => I::LD_R16_N(R16::HL, self.imm16()),
            0x22 => I::LDI_iR_R(R16::HL, R8::A),
            0x23 => I::INC_R16(R16::HL),
            0x24 => I::INC_R(R8::H),
            0x25 => I::DEC_R(R8::H),
            0x26 => I::LD_R_N(R8::H, self.imm_u8()),
            0x27 => I::DAA,
            0x28 => I::JR_C_D(Condition::Z, self.imm_i8()),
            0x29 => I::ADD_R16(R16::HL),
            0x2a => I::LDI_R_iR(R8::A, R16::HL),
            0x2b => I::DEC_R16(R16::HL),
            0x2c => I::INC_R(R8::L),
            0x2d => I::DEC_R(R8::L),
            0x2e => I::LD_R_N(R8::L, self.imm_u8()),
            0x2f => I::CPL,

            0x30 => I::JR_C_D(Condition::NC, self.imm_i8()),
            0x31 => I::LD_R16_N(R16::SP, self.imm16()),
            0x32 => I::LDD_iR_R(R16::HL, R8::A),
            0x33 => I::INC_R16(R16::SP),
            0x34 => I::INC_iR(R16::HL),
            0x35 => I::DEC_iR(R16::HL),
            0x36 => I::LD_iR_N(R16::HL, self.imm_u8()),
            0x37 => I::SCF,
            0x38 => I::JR_C_D(Condition::C, self.imm_i8()),
            0x39 => I::ADD_R16(R16::SP),
            0x3a => I::LDD_R_iR(R8::A, R16::HL),
            0x3b => I::DEC_R16(R16::SP),
            0x3c => I::INC_R(R8::A),
            0x3d => I::DEC_R(R8::A),
            0x3e => I::LD_R_N(R8::A, self.imm_u8()),
            0x3f => I::CCF,

            0x40 => I::LD_R_R(R8::B, R8::B),
            0x41 => I::LD_R_R(R8::B, R8::C),
            0x42 => I::LD_R_R(R8::B, R8::D),
            0x43 => I::LD_R_R(R8::B, R8::E),
            0x44 => I::LD_R_R(R8::B, R8::H),
            0x45 => I::LD_R_R(R8::B, R8::L),
            0x46 => I::LD_R_iR(R8::B, R16::HL),
            0x47 => I::LD_R_R(R8::B, R8::A),
            0x48 => I::LD_R_R(R8::C, R8::B),
            0x49 => I::LD_R_R(R8::C, R8::C),
            0x4a => I::LD_R_R(R8::C, R8::D),
            0x4b => I::LD_R_R(R8::C, R8::E),
            0x4c => I::LD_R_R(R8::C, R8::H),
            0x4d => I::LD_R_R(R8::C, R8::L),
            0x4e => I::LD_R_iR(R8::C, R16::HL),
            0x4f => I::LD_R_R(R8::C, R8::A),

            0x50 => I::LD_R_R(R8::D, R8::B),
            0x51 => I::LD_R_R(R8::D, R8::C),
            0x52 => I::LD_R_R(R8::D, R8::D),
            0x53 => I::LD_R_R(R8::D, R8::E),
            0x54 => I::LD_R_R(R8::D, R8::H),
            0x55 => I::LD_R_R(R8::D, R8::L),
            0x56 => I::LD_R_iR(R8::D, R16::HL),
            0x57 => I::LD_R_R(R8::D, R8::A),
            0x58 => I::LD_R_R(R8::E, R8::B),
            0x59 => I::LD_R_R(R8::E, R8::C),
            0x5a => I::LD_R_R(R8::E, R8::D),
            0x5b => I::LD_R_R(R8::E, R8::E),
            0x5c => I::LD_R_R(R8::E, R8::H),
            0x5d => I::LD_R_R(R8::E, R8::L),
            0x5e => I::LD_R_iR(R8::E, R16::HL),
            0x5f => I::LD_R_R(R8::E, R8::A),

            0x60 => I::LD_R_R(R8::H, R8::B),
            0x61 => I::LD_R_R(R8::H, R8::C),
            0x62 => I::LD_R_R(R8::H, R8::D),
            0x63 => I::LD_R_R(R8::H, R8::E),
            0x64 => I::LD_R_R(R8::H, R8::H),
            0x65 => I::LD_R_R(R8::H, R8::L),
            0x66 => I::LD_R_iR(R8::H, R16::HL),
            0x67 => I::LD_R_R(R8::H, R8::A),
            0x68 => I::LD_R_R(R8::L, R8::B),
            0x69 => I::LD_R_R(R8::L, R8::C),
            0x6a => I::LD_R_R(R8::L, R8::D),
            0x6b => I::LD_R_R(R8::L, R8::E),
            0x6c => I::LD_R_R(R8::L, R8::H),
            0x6d => I::LD_R_R(R8::L, R8::L),
            0x6e => I::LD_R_iR(R8::L, R16::HL),
            0x6f => I::LD_R_R(R8::L, R8::A),

            0x70 => I::LD_iR_R(R16::HL, R8::B),
            0x71 => I::LD_iR_R(R16::HL, R8::C),
            0x72 => I::LD_iR_R(R16::HL, R8::D),
            0x73 => I::LD_iR_R(R16::HL, R8::E),
            0x74 => I::LD_iR_R(R16::HL, R8::H),
            0x75 => I::LD_iR_R(R16::HL, R8::L),
            0x76 => I::HALT,
            0x77 => I::LD_iR_R(R16::HL, R8::A),
            0x78 => I::LD_R_R(R8::A, R8::B),
            0x79 => I::LD_R_R(R8::A, R8::C),
            0x7a => I::LD_R_R(R8::A, R8::D),
            0x7b => I::LD_R_R(R8::A, R8::E),
            0x7c => I::LD_R_R(R8::A, R8::H),
            0x7d => I::LD_R_R(R8::A, R8::L),
            0x7e => I::LD_R_iR(R8::A, R16::HL),
            0x7f => I::LD_R_R(R8::A, R8::A),

            0x80 => I::ADD_R(R8::B),
            0x81 => I::ADD_R(R8::C),
            0x82 => I::ADD_R(R8::D),
            0x83 => I::ADD_R(R8::E),
            0x84 => I::ADD_R(R8::H),
            0x85 => I::ADD_R(R8::L),
            0x86 => I::ADD_iR(R16::HL),
            0x87 => I::ADD_R(R8::A),
            0x88 => I::ADC_R(R8::B),
            0x89 => I::ADC_R(R8::C),
            0x8a => I::ADC_R(R8::D),
            0x8b => I::ADC_R(R8::E),
            0x8c => I::ADC_R(R8::H),
            0x8d => I::ADC_R(R8::L),
            0x8e => I::ADC_iR(R16::HL),
            0x8f => I::ADC_R(R8::A),

            0x90 => I::SUB_R(R8::B),
            0x91 => I::SUB_R(R8::C),
            0x92 => I::SUB_R(R8::D),
            0x93 => I::SUB_R(R8::E),
            0x94 => I::SUB_R(R8::H),
            0x95 => I::SUB_R(R8::L),
            0x96 => I::SUB_iR(R16::HL),
            0x97 => I::SUB_R(R8::A),
            0x98 => I::SBC_R(R8::B),
            0x99 => I::SBC_R(R8::C),
            0x9a => I::SBC_R(R8::D),
            0x9b => I::SBC_R(R8::E),
            0x9c => I::SBC_R(R8::H),
            0x9d => I::SBC_R(R8::L),
            0x9e => I::SBC_iR(R16::HL),
            0x9f => I::SBC_R(R8::A),

            0xa0 => I::AND_R(R8::B),
            0xa1 => I::AND_R(R8::C),
            0xa2 => I::AND_R(R8::D),
            0xa3 => I::AND_R(R8::E),
            0xa4 => I::AND_R(R8::H),
            0xa5 => I::AND_R(R8::L),
            0xa6 => I::AND_iR(R16::HL),
            0xa7 => I::AND_R(R8::A),
            0xa8 => I::XOR_R(R8::B),
            0xa9 => I::XOR_R(R8::C),
            0xaa => I::XOR_R(R8::D),
            0xab => I::XOR_R(R8::E),
            0xac => I::XOR_R(R8::H),
            0xad => I::XOR_R(R8::L),
            0xae => I::XOR_iR(R16::HL),
            0xaf => I::XOR_R(R8::A),

            0xb0 => I::OR_R(R8::B),
            0xb1 => I::OR_R(R8::C),
            0xb2 => I::OR_R(R8::D),
            0xb3 => I::OR_R(R8::E),
            0xb4 => I::OR_R(R8::H),
            0xb5 => I::OR_R(R8::L),
            0xb6 => I::OR_iR(R16::HL),
            0xb7 => I::OR_R(R8::A),
            0xb8 => I::CMP_R(R8::B),
            0xb9 => I::CMP_R(R8::C),
            0xba => I::CMP_R(R8::D),
            0xbb => I::CMP_R(R8::E),
            0xbc => I::CMP_R(R8::H),
            0xbd => I::CMP_R(R8::L),
            0xbe => I::CMP_iR(R16::HL),
            0xbf => I::CMP_R(R8::A),

            0xc0 => I::RET_C(Condition::NZ),
            0xc1 => I::POP(R16::BC),
            0xc2 => I::JP_C_N(Condition::NZ, self.imm16()),
            0xc3 => I::JP_N(self.imm16()),
            0xc4 => I::CALL_C_N(Condition::NZ, self.imm16()),
            0xc5 => I::PUSH(R16::BC),
            0xc6 => I::ADD_N(self.imm_u8()),
            0xc7 => I::RST(0x00),
            0xc8 => I::RET_C(Condition::Z),
            0xc9 => I::RET,
            0xca => I::JP_C_N(Condition::Z, self.imm16()),
            0xcb => self.cb_instruction(),
            0xcc => I::CALL_C_N(Condition::Z, self.imm16()),
            0xcd => I::CALL_N(self.imm16()),
            0xce => I::ADC_N(self.imm_u8()),
            0xcf => I::RST(0x08),
            0xd0 => I::RET_C(Condition::NC),
            0xd1 => I::POP(R16::DE),
            0xd2 => I::JP_C_N(Condition::NC, self.imm16()),
            0xd3 => I::NOP,
            0xd4 => I::CALL_C_N(Condition::NC, self.imm16()),
            0xd5 => I::PUSH(R16::DE),
            0xd6 => I::SUB_N(self.imm_u8()),
            0xd7 => I::RST(0x10),
            0xd8 => I::RET_C(Condition::C),
            0xd9 => I::RETI,
            0xda => I::JP_C_N(Condition::C, self.imm16()),
            0xdb => I::NOP,
            0xdc => I::CALL_C_N(Condition::C, self.imm16()),
            0xdd => I::NOP,
            0xde => I::SBC_N(self.imm_u8()),
            0xdf => I::RST(0x18),

            0xe0 => I::LD_iN_R(0xff00 + self.imm_u8() as u16, R8::A),
            0xe1 => I::POP(R16::HL),
            0xe2 => I::LD_iN_R(0xff00 + self[R8::C] as u16, R8::A),
            0xe3 => I::NOP,
            0xe4 => I::NOP,
            0xe5 => I::PUSH(R16::HL),
            0xe6 => I::AND_N(self.imm_u8()),
            0xe7 => I::RST(0x20),
            0xe8 => I::ADD_SP_D(self.imm_i8()),
            0xe9 => I::JP_HL,
            0xea => I::LD_iN_R(self.imm16(), R8::A),
            0xeb => I::NOP,
            0xec => I::NOP,
            0xed => I::NOP,
            0xee => I::XOR_N(self.imm_u8()),
            0xef => I::RST(0x28),

            0xf0 => I::LD_R_iN(R8::A, 0xff00 + self.imm_u8() as u16),
            0xf1 => I::POP(R16::AF),
            0xf2 => I::LD_R_iN(R8::A, 0xff00 + self[R8::C] as u16),
            0xf3 => I::DI,
            0xf4 => I::NOP,
            0xf5 => I::PUSH(R16::AF),
            0xf6 => I::OR_N(self.imm_u8()),
            0xf7 => I::RST(0x30),
            0xf8 => I::LD_HL_SP_D(self.imm_i8()),
            0xf9 => I::LD_R16_R16(R16::SP, R16::HL),
            0xfa => I::LD_R_iN(R8::A, self.imm16()),
            0xfb => I::EI,
            0xfc => I::NOP,
            0xfd => I::NOP,
            0xfe => I::CMP_N(self.imm_u8()),
            0xff => I::RST(0x38),
        }
    }

    fn cb_instruction(&mut self) -> I {
        let opcode = self.imm_u8();
        let ins = (opcode & 0b1111_1000) >> 3;
        let reg = opcode & 0b111;
        if reg == 6 {
            match ins {
                0x00 => I::RLC_iHL,
                0x01 => I::RRC_iHL,
                0x02 => I::RL_iHL,
                0x03 => I::RR_iHL,
                0x04 => I::SLA_iHL,
                0x05 => I::SRA_iHL,
                0x06 => I::SWAP_iHL,
                0x07 => I::SRL_iHL,
                0x08..=0x0f => I::BIT_N_iHL(ins & 0b111),
                0x10..=0x17 => I::RES_N_iHL(ins & 0b111),
                0x18..=0x1f => I::SET_N_iHL(ins & 0b111),
                _ => unreachable!(),
            }
        } else {
            let reg = match reg {
                0 => R8::B,
                1 => R8::C,
                2 => R8::D,
                3 => R8::E,
                4 => R8::H,
                5 => R8::L,
                7 => R8::A,
                _ => unreachable!(),
            };
            match ins {
                0x00 => I::RLC_R(reg),
                0x01 => I::RRC_R(reg),
                0x02 => I::RL_R(reg),
                0x03 => I::RR_R(reg),
                0x04 => I::SLA_R(reg),
                0x05 => I::SRA_R(reg),
                0x06 => I::SWAP_R(reg),
                0x07 => I::SRL_R(reg),
                0x08..=0x0f => I::BIT_N_R(ins & 0b111, reg),
                0x10..=0x17 => I::RES_N_R(ins & 0b111, reg),
                0x18..=0x1f => I::SET_N_R(ins & 0b111, reg),
                _ => unreachable!(),
            }
        }
    }

    pub fn z(&self) -> bool {
        self.cpu.z()
    }
    pub fn n(&self) -> bool {
        self.cpu.n()
    }
    pub fn h(&self) -> bool {
        self.cpu.h()
    }
    pub fn c(&self) -> bool {
        self.cpu.c()
    }
    pub fn i(&self) -> bool {
        self.cpu.i()
    }

    pub fn set_z(&mut self, v: bool) {
        self.cpu.set_z(v);
    }
    pub fn set_n(&mut self, v: bool) {
        self.cpu.set_n(v);
    }
    pub fn set_h(&mut self, v: bool) {
        self.cpu.set_h(v);
    }
    pub fn set_c(&mut self, v: bool) {
        self.cpu.set_c(v);
    }
    pub fn set_i(&mut self, v: bool) {
        self.cpu.set_i(v);
    }

    pub fn indirect(&self, idx: R16) -> u8 {
        self.mem.get(self.cpu[idx])
    }
    pub fn indirect_set(&mut self, idx: R16, val: u8) {
        self.mem.set(self.cpu[idx], val)
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn ld_r_r() {
        //0x78 => I::LD_R_R(R8::A, R8::B),
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x78);
        gr[R8::B] = 0xab;
        gr.update();
        assert_eq!(gr[R8::A], 0xab);
    }

    #[test]
    fn ld_r_n() {
        //0x0e LD C,u8
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x0e);
        gr.mem.set(1, 0xab);
        gr.update();
        assert_eq!(gr[R8::C], 0xab);
    }

    #[test]
    fn ld_ir_r() {
        //0x12 LD (DE), A
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x12);
        gr[R16::DE] = 1;
        gr[R8::A] = 0xca;
        gr.update();
        assert_eq!(gr.mem.get(1), 0xca);
    }

    #[test]
    fn ld_ir_n() {
        //0x36 LD (HL), u8
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x36);
        gr.mem.set(1, 0xca);
        gr[R16::HL] = 2;
        gr.update();
        assert_eq!(gr.mem.get(2), 0xca);
    }

    #[test]
    fn ld_r_ir() {
        //0x7e => I::LD_R_iR16(R8::A, R16::HL),
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x7e);
        gr.mem.set(1, 0xab);
        gr[R16::HL] = 0x1;
        gr.update();
        assert_eq!(gr[R8::A], 0xab);
    }

    #[test]
    fn ld_r_in() {
        //0xfa LD A, (u16)
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xfa);
        gr.mem.set(1, 3);
        gr.mem.set(2, 0);
        gr.mem.set(3, 0xca);
        gr.update();
    }

    #[test]
    fn ld_in_r() {
        //0xea LD (u16), A
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xea);
        gr.mem.set(1, 3);
        gr.mem.set(2, 0);
        gr[R8::A] = 0xca;
        gr.update();
        assert_eq!(gr.mem.get(3), 0xca);
    }

    #[test]
    fn ldi_ir_r() {
        //0x22 LD (HL+), A
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x22);
        gr[R16::HL] = 1;
        gr[R8::A] = 0xca;
        gr.update();
        assert_eq!(gr.mem.get(1), 0xca);
        assert_eq!(gr[R16::HL], 0x2);
    }

    #[test]
    fn ldi_r_ir() {
        //0x2A LD A, (HL+)
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x2a);
        gr.mem.set(1, 0xca);
        gr[R16::HL] = 1;
        gr.update();
        assert_eq!(gr[R8::A], 0xca);
        assert_eq!(gr[R16::HL], 2);
    }

    #[test]
    fn ldd_ir_r() {
        //0x32 LD (HL+), A
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x32);
        gr[R16::HL] = 1;
        gr[R8::A] = 0xca;
        gr.update();
        assert_eq!(gr.mem.get(1), 0xca);
        assert_eq!(gr[R16::HL], 0);
    }

    #[test]
    fn ldd_r_ir() {
        //0x3a LD A, (HL-)
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x3a);
        gr.mem.set(1, 0xca);
        gr[R16::HL] = 1;
        assert_eq!(gr[R8::A], 0);
        assert_eq!(gr[R16::HL], 1);
        gr.update();
        assert_eq!(gr[R8::A], 0xca);
        assert_eq!(gr[R16::HL], 0);
    }

    #[test]
    fn ld_r16_n() {
        //0x21 LD HL,u16
        //0x31 LD SP,u16
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x21);
        gr.mem.set(1, 0xaf);
        gr.mem.set(2, 0xec);
        gr.mem.set(3, 0x31);
        gr.mem.set(4, 0xfc);
        gr.mem.set(5, 0xca);
        gr.update();
        assert_eq!(gr[R16::HL], 0xecaf);
        gr.update();
        assert_eq!(gr[R16::SP], 0xcafc);
    }

    #[test]
    fn ld_in_r16() {
        //0x08 LD (u16),SP
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x08);
        gr.mem.set(1, 3);
        gr.mem.set(2, 0);
        gr[R16::SP] = 0xcafc;
        gr.update();
        assert_eq!(gr.mem.get(3), 0xfc);
        assert_eq!(gr.mem.get(4), 0xca);
    }

    #[test]
    fn ld_r16_r16() {
        //0xf9 ld sp,hl
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xf9);
        gr[R16::HL] = 0xcafc;
        gr.update();
        assert_eq!(gr[R16::SP], 0xcafc);
    }

    #[test]
    fn push() {
        //0xd5 PUSH DE
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xd5);
        gr[R16::SP] = 2;
        gr[R16::DE] = 0xcafc;
        gr.update();
        assert_eq!(gr.mem.get(0), 0xfc);
        assert_eq!(gr.mem.get(1), 0xca);
        assert_eq!(gr[R16::SP], 0);
    }

    #[test]
    fn pop_af() {
        //0xc1 POP AF
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xf1);
        gr.mem.set(1, 0xfc);
        gr.mem.set(2, 0xca);
        gr[R16::SP] = 1;
        gr.update();
        assert_eq!(gr[R16::AF], 0xcaf0);
    }
    #[test]
    fn pop() {
        //0xc1 POP BC
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xc1);
        gr.mem.set(1, 0xfc);
        gr.mem.set(2, 0xca);
        gr[R16::SP] = 1;
        gr.update();
        assert_eq!(gr[R16::BC], 0xcafc);
    }

    #[test]
    fn add_r_fitting() {
        //0x83 ADD A,E
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x83);

        gr[R8::A] = 0xca;
        gr[R8::E] = 0x11;
        gr.update();
        assert_eq!(gr[R8::A], 0xdb);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), false);
        assert_eq!(gr.c(), false);
    }

    #[test]
    fn add_r_overflow_low() {
        //0x83 ADD A,E
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x83);

        gr[R8::A] = 0xca;
        gr[R8::E] = 0x1f;
        gr.update();
        assert_eq!(gr[R8::A], 0xe9);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), true);
        assert_eq!(gr.c(), false);
    }

    #[test]
    fn add_r_overflow_high() {
        //0x83 ADD A,E
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x83);

        gr[R8::A] = 0xca;
        gr[R8::E] = 0x41;
        gr.update();
        assert_eq!(gr[R8::A], 0x0b);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), false);
        assert_eq!(gr.c(), true);
    }

    #[test]
    fn add_r_overflow_both() {
        //0x83 ADD A,E
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x83);

        gr[R8::A] = 0xca;
        gr[R8::E] = 0x46;
        gr.update();
        assert_eq!(gr[R8::A], 0x10);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), true);
        assert_eq!(gr.c(), true);
    }

    #[test]
    fn add_r_0sum() {
        //0x83 ADD A,E
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x83);

        gr[R8::A] = 0xff;
        gr[R8::E] = 0x01;
        gr.update();
        assert_eq!(gr[R8::A], 0x00);
        assert_eq!(gr.z(), true);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), true);
        assert_eq!(gr.c(), true);
    }

    #[test]
    fn add_r_0args() {
        //0x83 ADD A,E
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x83);

        gr[R8::A] = 0x0;
        gr[R8::E] = 0x0;
        gr.update();
        assert_eq!(gr[R8::A], 0x00);
        assert_eq!(gr.z(), true);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), false);
        assert_eq!(gr.c(), false);
    }

    #[test]
    fn add_n() {
        //0xc6 ADD A,u8
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xc6);
        gr.mem.set(1, 0xca);
        gr[R8::A] = 0x11;
        gr.update();
        assert_eq!(gr[R8::A], 0xdb);
    }

    #[test]
    fn add_ir() {
        //0x86 ADD A,(HL)
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x86);
        gr.mem.set(1, 0xca);
        gr[R8::A] = 0x11;
        gr[R16::HL] = 1;
        gr.update();
        assert_eq!(gr[R8::A], 0xdb);
    }

    #[test]
    fn adc_r_fitting() {
        //0x8b ADC A,E
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x8b);
        gr[R8::A] = 0xca;
        gr[R8::E] = 0x11;
        gr.set_c(true);
        gr.update();
        assert_eq!(gr[R8::A], 0xdc);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), false);
        assert_eq!(gr.c(), false);
    }

    #[test]
    fn adc_r_overflow_low() {
        //0x8b ADC A,E
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x8b);
        gr[R8::A] = 0xca;
        gr[R8::E] = 0x1f;
        gr.set_c(true);
        gr.update();
        assert_eq!(gr[R8::A], 0xea);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), true);
        assert_eq!(gr.c(), false);
    }

    #[test]
    fn adc_r_overflow_high() {
        //0x8b ADC A,E
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x8b);
        gr[R8::A] = 0xca;
        gr[R8::E] = 0x41;
        gr.set_c(true);
        gr.update();
        assert_eq!(gr[R8::A], 0x0c);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), false);
        assert_eq!(gr.c(), true);
    }

    #[test]
    fn adc_r_overflow_both() {
        //0x8b ADC A,E
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x8b);
        gr[R8::A] = 0xca;
        gr[R8::E] = 0x46;
        gr.set_c(true);
        gr.update();
        assert_eq!(gr[R8::A], 0x11);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), true);
        assert_eq!(gr.c(), true);
    }

    #[test]
    fn adc_r_0sum() {
        //0x8b ADC A,E
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x8b);
        gr[R8::A] = 0xfe;
        gr[R8::E] = 0x01;
        gr.set_c(true);
        gr.update();
        assert_eq!(gr[R8::A], 0x00);
        assert_eq!(gr.z(), true);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), true);
        assert_eq!(gr.c(), true);
    }

    #[test]
    fn adc_n() {
        //0xce ADC A,u8
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xce);
        gr.mem.set(1, 0xca);
        gr[R8::A] = 0x11;
        gr.set_c(true);
        gr.update();
        assert_eq!(gr[R8::A], 0xdc);
    }

    #[test]
    fn adc_ir() {
        //0x8e ADC A,(HL)
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x8e);
        gr.mem.set(1, 0xca);
        gr[R8::A] = 0x11;
        gr[R16::HL] = 1;
        gr.set_c(true);
        gr.update();
        assert_eq!(gr[R8::A], 0xdc);
    }

    #[test]
    fn sub_r_fitting() {
        //0x94 SUB A,H
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x94);
        gr[R8::A] = 0xca;
        gr[R8::H] = 0x11;
        gr.update();
        assert_eq!(gr[R8::A], 0xb9);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), true);
        assert_eq!(gr.h(), false);
        assert_eq!(gr.c(), false);
    }

    #[test]
    fn sub_r_overflow_low() {
        //0x94 SUB A,H
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x94);
        gr[R8::A] = 0xca;
        gr[R8::H] = 0x1f;
        gr.update();
        assert_eq!(gr[R8::A], 0xab);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), true);
        assert_eq!(gr.h(), true);
        assert_eq!(gr.c(), false);
    }

    #[test]
    fn sub_r_overflow_high() {
        //0x94 SUB A,H
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x94);
        gr[R8::A] = 0x4f;
        gr[R8::H] = 0x5a;
        gr.update();
        assert_eq!(gr[R8::A], 0xf5);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), true);
        assert_eq!(gr.h(), false);
        assert_eq!(gr.c(), true);
    }

    #[test]
    fn sub_r_overflow_both() {
        //0x94 SUB A,H
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x94);
        gr[R8::A] = 0x46;
        gr[R8::H] = 0xca;
        gr.update();
        assert_eq!(gr[R8::A], 0x7c);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), true);
        assert_eq!(gr.h(), true);
        assert_eq!(gr.c(), true);
    }

    #[test]
    fn sub_r_0sum() {
        //0x94 SUB A,H
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x94);
        gr[R8::A] = 0xfe;
        gr[R8::H] = 0xfe;
        gr.update();
        assert_eq!(gr[R8::A], 0x00);
        assert_eq!(gr.z(), true);
        assert_eq!(gr.n(), true);
        assert_eq!(gr.h(), false);
        assert_eq!(gr.c(), false);
    }

    #[test]
    fn sub_n() {
        //0xd6 SUB A,u8
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xd6);
        gr.mem.set(1, 0x11);
        gr[R8::A] = 0xca;
        gr.update();
        assert_eq!(gr[R8::A], 0xb9);
    }

    #[test]
    fn sub_ir() {
        //0x96 SUB A,(HL)
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x96);
        gr.mem.set(1, 0x11);
        gr[R8::A] = 0xca;
        gr[R16::HL] = 1;
        gr.update();
        assert_eq!(gr[R8::A], 0xb9);
    }

    #[test]
    fn sbc_r_fitting() {
        //0x9d SBC A,L
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x9d);
        gr[R8::A] = 0xca;
        gr[R8::L] = 0x11;
        gr.set_c(true);
        gr.update();
        assert_eq!(gr[R8::A], 0xb8);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), true);
        assert_eq!(gr.h(), false);
        assert_eq!(gr.c(), false);
    }

    #[test]
    fn sbc_r_overflow_low() {
        //0x9d SBC A,L
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x9d);
        gr[R8::A] = 0xca;
        gr[R8::L] = 0x1a;
        gr.set_c(true);
        gr.update();
        assert_eq!(gr[R8::A], 0xaf);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), true);
        assert_eq!(gr.h(), true);
        assert_eq!(gr.c(), false);
    }

    #[test]
    fn sbc_r_overflow_high() {
        //0x9d SBC A,L
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x9d);
        gr[R8::A] = 0x5a;
        gr[R8::L] = 0x61;
        gr.set_c(true);
        gr.update();
        assert_eq!(gr[R8::A], 0xf8);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), true);
        assert_eq!(gr.h(), false);
        assert_eq!(gr.c(), true);
    }

    #[test]
    fn sbc_r_overflow_both() {
        //0x9d SBC A,L
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x9d);
        gr[R8::A] = 0x11;
        gr[R8::L] = 0x21;
        gr.set_c(true);
        gr.update();
        assert_eq!(gr[R8::A], 0xef);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), true);
        assert_eq!(gr.h(), true);
        assert_eq!(gr.c(), true);
    }

    #[test]
    fn sbc_r_0sum() {
        //0x9d SBC A,L
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x9d);
        gr[R8::A] = 0xfe;
        gr[R8::L] = 0xfd;
        gr.set_c(true);
        gr.update();
        assert_eq!(gr[R8::A], 0x00);
        assert_eq!(gr.z(), true);
        assert_eq!(gr.n(), true);
        assert_eq!(gr.h(), false);
        assert_eq!(gr.c(), false);
    }

    #[test]
    fn sbc_n() {
        //0xde SBC A,u8
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xde);
        gr.mem.set(1, 0x11);
        gr[R8::A] = 0xca;
        gr.set_c(true);
        gr.update();
        assert_eq!(gr[R8::A], 0xb8);
    }

    #[test]
    fn sbc_ir() {
        //0x9e SBC A,(HL)
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x9e);
        gr.mem.set(1, 0x11);
        gr[R8::A] = 0xca;
        gr[R16::HL] = 1;
        gr.set_c(true);
        gr.update();
        assert_eq!(gr[R8::A], 0xb8);
    }

    #[test]
    fn and_r() {
        //0xa0 AND A, B
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xa0);
        gr[R8::A] = 0b1010_0101;
        gr[R8::B] = 0b1001_1001;
        gr.update();
        assert_eq!(gr[R8::A], 0b1000_0001);
    }

    #[test]
    fn and_n() {
        //0xe6 AND A, [u8]
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xe6);
        gr.mem.set(1, 0b1001_1001);
        gr[R8::A] = 0b1010_0101;
        gr.update();
        assert_eq!(gr[R8::A], 0b1000_0001);
    }
    #[test]
    fn and_ir() {
        //0xa6 AND A, [HL]
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xa6);
        gr[R8::A] = 0b1010_0101;
        gr.mem.set(1, 0b1001_1001);
        gr[R16::HL] = 1;
        gr.update();
        assert_eq!(gr[R8::A], 0b1000_0001);
    }

    #[test]
    fn xor_r() {
        //0xa8 Xxor A, B
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xa8);
        gr[R8::A] = 0b1010_0101;
        gr[R8::B] = 0b1001_1001;
        gr.update();
        assert_eq!(gr[R8::A], 0b0011_1100);
    }
    #[test]
    fn xor_n() {
        //0xee Xxor A, [u8]
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xee);
        gr.mem.set(1, 0b1001_1001);
        gr[R8::A] = 0b1010_0101;
        gr.update();
        assert_eq!(gr[R8::A], 0b0011_1100);
    }
    #[test]
    fn xor_ir() {
        //0xae Xxor A, [HL]
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xae);
        gr[R8::A] = 0b1010_0101;
        gr.mem.set(1, 0b1001_1001);
        gr[R16::HL] = 1;
        gr.update();
        assert_eq!(gr[R8::A], 0b0011_1100);
    }

    #[test]
    fn or_r() {
        //0xb0 OR A, B
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xb0);
        gr[R8::A] = 0b1010_0101;
        gr[R8::B] = 0b1001_1001;
        gr.update();
        assert_eq!(gr[R8::A], 0b1011_1101);
    }
    #[test]
    fn or_n() {
        //0xf6 OR A, [u8]
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xf6);
        gr.mem.set(1, 0b1001_1001);
        gr[R8::A] = 0b1010_0101;
        gr.update();
        assert_eq!(gr[R8::A], 0b1011_1101);
    }
    #[test]
    fn or_ir() {
        //0xb6 OR A, [HL]
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xb6);
        gr[R8::A] = 0b1010_0101;
        gr.mem.set(1, 0b1001_1001);
        gr[R16::HL] = 1;
        gr.update();
        assert_eq!(gr[R8::A], 0b1011_1101);
    }

    #[test]
    fn cmp_r() {
        //0xbc CMP A,H
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xbc);
        gr[R8::A] = 0x12;
        gr[R8::H] = 0x12;
        gr.update();
        assert_eq!(gr[R8::A], 0x12);
        assert_eq!(gr.z(), true);
        assert_eq!(gr.n(), true);
        assert_eq!(gr.h(), false);
        assert_eq!(gr.c(), false);
        gr[R16::PC] = 0;
        gr[R8::H] = 0x23;
        gr.update();
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), true);
        assert_eq!(gr.h(), true);
        assert_eq!(gr.c(), true);
    }
    #[test]
    fn cmp_n() {
        //0xbc CMP A,H
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xfe);
        gr.mem.set(1, 0x12);
        gr[R8::A] = 0x12;
        gr.update();
        assert_eq!(gr[R8::A], 0x12);
        assert_eq!(gr.z(), true);
        assert_eq!(gr.n(), true);
        assert_eq!(gr.h(), false);
        assert_eq!(gr.c(), false);
        gr[R16::PC] = 0;
        gr.mem.set(1, 0x23);
        gr.update();
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), true);
        assert_eq!(gr.h(), true);
        assert_eq!(gr.c(), true);
    }
    #[test]
    fn cmp_ir() {
        //0xbc CMP A,(HL)
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xbe);
        gr.mem.set(1, 0x12);
        gr[R8::A] = 0x12;
        gr[R16::HL] = 1;
        gr.update();
        assert_eq!(gr[R8::A], 0x12);
        assert_eq!(gr.z(), true);
        assert_eq!(gr.n(), true);
        assert_eq!(gr.h(), false);
        assert_eq!(gr.c(), false);
        gr[R16::PC] = 0;
        gr.mem.set(1, 0x23);
        gr.update();
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), true);
        assert_eq!(gr.h(), true);
        assert_eq!(gr.c(), true);
    }

    #[test]
    fn inc_r() {
        //0x04 INC D
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x04);
        gr[R8::B] = 0xca;
        gr.update();
        assert_eq!(gr[R8::B], 0xcb);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), false);
    }
    #[test]
    fn inc_r_with_overflow() {
        //0x14 INC D
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x14);
        gr[R8::D] = 0xcf;
        gr.update();
        assert_eq!(gr[R8::D], 0xd0);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), true);
    }
    #[test]
    fn inc_ir() {
        //0x34 INC (HL)
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x34);
        gr[R16::HL] = 1;
        gr.mem.set(1, 0xfc);
        gr.update();
        assert_eq!(gr.mem.get(1), 0xfd);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), false);
    }
    #[test]
    fn dec_r() {
        //0x25 DEC H
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x25);
        gr[R8::H] = 0xe0;
        gr.update();
        assert_eq!(gr[R8::H], 0xdf);
    }
    #[test]
    fn dec_ir() {
        //0x35 dec (HL)
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x35);
        gr.update();
        assert_eq!(gr.mem.get(0), 0x34);
    }
    #[test]
    fn daa_valid_bcd() {
        //0x27 DAA
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x27);
        gr[R8::A] = 0x93;
        gr.set_c(false);
        gr.set_h(false);
        gr.update();
        assert_eq!(gr[R8::A], 0x93);
    }
    #[test]
    fn daa_low_nibble_overflow() {
        //0x27 DAA
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x27);
        gr[R8::A] = 0x7a;
        gr.set_c(false);
        gr.set_h(false);
        gr.update();
        assert_eq!(gr[R8::A], 0x80);
    }
    #[test]
    fn daa_high_nibble_overflow() {
        //0x27 DAA
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x27);
        gr[R8::A] = 0xb3;
        gr.set_c(false);
        gr.set_h(false);
        gr.update();
        assert_eq!(gr[R8::A], 0x13);
        assert_eq!(gr.c(), true)
    }
    #[test]
    fn cpl() {
        //0x2f CPL
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x2f);
        gr[R8::A] = 0b1010_1010;
        gr.update();
        assert_eq!(gr[R8::A], 0b0101_0101);
    }

    #[test]
    fn add_r16() {
        //0x29 ADD HL, HL
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x29);
        gr[R16::HL] = 0x3afc;
        gr.update();
        assert_eq!(gr[R16::HL], 0x75f8);
    }
    #[test]
    fn add_r16_overflow_half() {
        //0x09 ADD HL, BC
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x09);
        gr[R16::HL] = 0x0f00;
        gr[R16::BC] = 0x0100;
        gr.update();
        assert_eq!(gr[R16::HL], 0x1000);
        assert_eq!(gr.c(), false);
        assert_eq!(gr.h(), true);
        assert_eq!(gr.n(), false);
    }
    #[test]
    fn add_r16_overflow() {
        //0x09 ADD HL, BC
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x09);
        gr[R16::HL] = 0xf000;
        gr[R16::BC] = 0x1000;
        gr.update();
        assert_eq!(gr[R16::HL], 0);
        assert_eq!(gr.c(), true);
        assert_eq!(gr.h(), false);
        assert_eq!(gr.n(), false);
    }
    #[test]
    fn inc_r16() {
        //0x13 INC DE
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x13);
        gr[R16::DE] = 0xcafc;
        gr.update();
        assert_eq!(gr[R16::DE], 0xcafd);
    }
    #[test]
    fn dec_r16() {
        //0x0b DEC BC
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x0b);
        gr[R16::BC] = 0xcafc;
        gr.update();
        assert_eq!(gr[R16::BC], 0xcafb);
    }
    #[test]
    fn add_sp_d_positive() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xe8);
        gr.mem.set(1, 0x7c);
        gr[R16::SP] = 0xca00;
        gr.update();
        assert_eq!(gr[R16::SP], 0xca7c);
    }
    #[test]
    fn add_sp_d_negative() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xe8);
        gr.mem.set(1, 0x0fc);
        gr[R16::SP] = 0xca00;
        gr.update();
        assert_eq!(gr[R16::SP], 0xc9fc);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), true);
        assert_eq!(gr.c(), true);
    }
    #[test]
    fn ld_hl_sp_d_positive() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xf8);
        gr.mem.set(1, 0b0101_1010);
        gr[R16::SP] = 0xcaa2;
        gr.update();
        assert_eq!(gr[R16::HL], 0xcafc);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), false);
        assert_eq!(gr.c(), false);
    }
    #[test]
    fn ld_hl_sp_d_negative() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xf8);
        gr.mem.set(1, 0b1101_1010);
        gr[R16::SP] = 0xca00;
        gr.update();
        assert_eq!(gr[R16::HL], 0xc9da);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), true);
        assert_eq!(gr.c(), true);
    }

    //rotate and shift
    #[test]
    fn rlca() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x07);
        gr.mem.set(1, 0x07);
        gr[R8::A] = 0b1010_1010;
        gr.update();
        assert_eq!(gr[R8::A], 0b0101_0101);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), false);
        assert_eq!(gr.c(), true);
        gr.update();
        assert_eq!(gr[R8::A], 0b1010_1010);
        assert_eq!(gr.c(), false);
    }
    #[test]
    fn rla() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x17);
        gr.mem.set(1, 0x17);
        gr.mem.set(2, 0x17);
        gr[R8::A] = 0b0101_0101;
        gr.set_c(true);
        gr.update();
        assert_eq!(gr[R8::A], 0b1010_1011);
        assert_eq!(gr.c(), false);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), false);
        gr.update();
        assert_eq!(gr[R8::A], 0b0101_0110);
        assert_eq!(gr.c(), true);
        gr.update();
        assert_eq!(gr[R8::A], 0b1010_1101);
        assert_eq!(gr.c(), false);
    }
    #[test]
    fn rrca() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x0f);
        gr.mem.set(1, 0x0f);
        gr[R8::A] = 0b1010_1010;
        gr.update();
        assert_eq!(gr[R8::A], 0b0101_0101);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), false);
        assert_eq!(gr.c(), false);
        gr.update();
        assert_eq!(gr[R8::A], 0b1010_1010);
        assert_eq!(gr.c(), true);
    }
    #[test]
    fn rra() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x1f);
        gr.mem.set(1, 0x1f);
        gr.mem.set(2, 0x1f);
        gr[R8::A] = 0b0101_0101;
        gr.set_c(true);
        gr.update();
        assert_eq!(gr[R8::A], 0b1010_1010);
        assert_eq!(gr.c(), true);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), false);
        gr.update();
        assert_eq!(gr[R8::A], 0b1101_0101);
        assert_eq!(gr.c(), false);
        gr.update();
        assert_eq!(gr[R8::A], 0b0110_1010);
        assert_eq!(gr.c(), true);
    }

    #[test]
    fn rlc_r() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xcb);
        gr.mem.set(1, 0x03);
        gr[R8::E] = 0b1000_0000;
        gr.update();
        assert_eq!(gr[R8::E], 0b0000_0001);
        assert_eq!(gr.c(), true);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), false);
    }
    #[test]
    fn rlc_ihl() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xcb);
        gr.mem.set(1, 0x06);
        gr.mem.set(2, 0b1000_0000);
        gr[R16::HL] = 2;
        gr.update();
        assert_eq!(gr.mem.get(2), 0b0000_0001);
        assert_eq!(gr.c(), true);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), false);
    }
    #[test]
    fn rl_r() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xcb);
        gr.mem.set(1, 0x13);
        gr[R8::E] = 0b1000_0000;
        gr.set_c(false);
        gr.update();
        assert_eq!(gr[R8::E], 0b0000_0000);
        assert_eq!(gr.c(), true);
        assert_eq!(gr.z(), true);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), false);
    }
    #[test]
    fn rl_ihl() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xcb);
        gr.mem.set(1, 0x16);
        gr.mem.set(2, 0b1000_0000);
        gr[R16::HL] = 2;
        gr.set_c(false);
        gr.update();
        assert_eq!(gr.mem.get(2), 0b0000_0000);
        assert_eq!(gr.c(), true);
        assert_eq!(gr.z(), true);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), false);
    }

    #[test]
    fn rrc_r() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xcb);
        gr.mem.set(1, 0x0b);
        gr[R8::E] = 0b0001_0001;
        gr.update();
        assert_eq!(gr[R8::E], 0b1000_1000);
        assert_eq!(gr.c(), true);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), false);
    }
    #[test]
    fn rrc_ihl() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xcb);
        gr.mem.set(1, 0x0e);
        gr.mem.set(2, 0b1000_0001);
        gr[R16::HL] = 2;
        gr.update();
        assert_eq!(gr.mem.get(2), 0b1100_0000);
        assert_eq!(gr.c(), true);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), false);
    }
    #[test]
    fn rr_r() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xcb);
        gr.mem.set(1, 0x1b);
        gr[R8::E] = 0b1000_0001;
        gr.set_c(false);
        gr.update();
        assert_eq!(gr[R8::E], 0b0100_0000);
        assert_eq!(gr.c(), true);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), false);
    }
    #[test]
    fn rr_ihl() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xcb);
        gr.mem.set(1, 0x1e);
        gr.mem.set(2, 0b1000_0001);
        gr[R16::HL] = 2;
        gr.set_c(false);
        gr.update();
        assert_eq!(gr.mem.get(2), 0b0100_0000);
        assert_eq!(gr.c(), true);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), false);
    }

    #[test]
    fn sla_r() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xcb);
        gr.mem.set(1, 0x23);
        gr[R8::E] = 0b1000_0001;
        gr.update();
        assert_eq!(gr[R8::E], 0b0000_0010);
        assert_eq!(gr.c(), true);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), false);
    }
    #[test]
    fn sla_ihl() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xcb);
        gr.mem.set(1, 0x26);
        gr.mem.set(2, 0b1000_0001);
        gr[R16::HL] = 2;
        gr.update();
        assert_eq!(gr.mem.get(2), 0b0000_0010);
        assert_eq!(gr.c(), true);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), false);
    }

    #[test]
    fn sra_r() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xcb);
        gr.mem.set(1, 0x2b);
        gr[R8::E] = 0b1000_0001;
        gr.update();
        assert_eq!(gr[R8::E], 0b1100_0000);
        assert_eq!(gr.c(), true);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), false);
    }
    #[test]
    fn sra_ihl() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xcb);
        gr.mem.set(1, 0x2e);
        gr.mem.set(2, 0b1000_0001);
        gr[R16::HL] = 2;
        gr.update();
        assert_eq!(gr.mem.get(2), 0b1100_0000);
        assert_eq!(gr.c(), true);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), false);
    }

    #[test]
    fn srl_r() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xcb);
        gr.mem.set(1, 0x3b);
        gr[R8::E] = 0b1000_0001;
        gr.update();
        assert_eq!(gr[R8::E], 0b0100_0000);
        assert_eq!(gr.c(), true);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), false);
    }
    #[test]
    fn srl_ihl() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xcb);
        gr.mem.set(1, 0x3e);
        gr.mem.set(2, 0b1000_0001);
        gr[R16::HL] = 2;
        gr.update();
        assert_eq!(gr.mem.get(2), 0b0100_0000);
        assert_eq!(gr.c(), true);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), false);
    }

    #[test]
    fn swap_r() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xcb);
        gr.mem.set(1, 0x33);
        gr[R8::E] = 0b1000_0001;
        gr.update();
        assert_eq!(gr[R8::E], 0b0001_1000);
        assert_eq!(gr.c(), false);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), false);
    }
    #[test]
    fn swap_ihl() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xcb);
        gr.mem.set(1, 0x36);
        gr.mem.set(2, 0b1000_0001);
        gr[R16::HL] = 2;
        gr.update();
        assert_eq!(gr.mem.get(2), 0b0001_1000);
        assert_eq!(gr.c(), false);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), false);
    }

    #[test]
    fn bit_n_r() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xcb);
        gr.mem.set(1, 0x63); //BIT 4,E
        gr[R8::E] = 0b0001_0000;
        gr.update();
        assert_eq!(gr[R8::E], 0b0001_0000);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), true);
        gr[R16::PC] = 0;
        gr.mem.set(1, 0x6b); //BIT 5,E
        gr.update();
        assert_eq!(gr[R8::E], 0b0001_0000);
        assert_eq!(gr.z(), true);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), true);
    }
    #[test]
    fn bit_n_ihl() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xcb);
        gr.mem.set(1, 0x66); //BIT 4,(HL)
        gr.mem.set(2, 0b0001_0000);
        gr[R16::HL] = 2;
        gr.update();
        assert_eq!(gr.mem.get(2), 0b0001_0000);
        assert_eq!(gr.z(), false);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), true);
        gr[R16::PC] = 0;
        gr.mem.set(1, 0x6e); //BIT 5,(HL)
        gr.update();
        assert_eq!(gr.mem.get(2), 0b0001_0000);
        assert_eq!(gr.z(), true);
        assert_eq!(gr.n(), false);
        assert_eq!(gr.h(), true);
    }

    #[test]
    fn res_n_r() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xcb);
        gr.mem.set(1, 0xa3); //RES 4,E
        gr[R8::E] = 0b0001_0000;
        gr.update();
        assert_eq!(gr[R8::E], 0b0000_0000);
        gr[R16::PC] = 0;
        gr.mem.set(1, 0xab); //RES 5,E
        gr.update();
        assert_eq!(gr[R8::E], 0b0000_0000);
    }
    #[test]
    fn res_n_ihl() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xcb);
        gr.mem.set(1, 0xa6); //RES 4,E
        gr.mem.set(2, 0b0001_0000);
        gr[R16::HL] = 2;
        gr.update();
        assert_eq!(gr.mem.get(2), 0b0000_0000);
        gr[R16::PC] = 0;
        gr.mem.set(1, 0xae); //RES 5,E
        gr.update();
        assert_eq!(gr.mem.get(2), 0b0000_0000);
    }

    #[test]
    fn set_n_r() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xcb);
        gr.mem.set(1, 0xe3); //set 4,E
        gr[R8::E] = 0b0001_0000;
        gr.update();
        assert_eq!(gr[R8::E], 0b0001_0000);
        gr[R16::PC] = 0;
        gr.mem.set(1, 0xeb); //set 5,E
        gr.update();
        assert_eq!(gr[R8::E], 0b0011_0000);
    }
    #[test]
    fn set_n_ihl() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xcb);
        gr.mem.set(1, 0xe6); //set 4,E
        gr.mem.set(2, 0b0001_0000);
        gr[R16::HL] = 2;
        gr.update();
        assert_eq!(gr.mem.get(2), 0b0001_0000);
        gr[R16::PC] = 0;
        gr.mem.set(1, 0xee); //set 5,E
        gr.update();
        assert_eq!(gr.mem.get(2), 0b0011_0000);
    }

    //CPU control
    #[test]
    fn ccf() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x3f);
        gr.set_c(false);
        gr.update();
        assert_eq!(gr.c(), true);
        gr[R16::PC] = 0;
        gr.update();
        assert_eq!(gr.c(), false);
    }

    #[test]
    fn scf() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x37);
        gr.set_c(false);
        gr.update();
        assert_eq!(gr.c(), true);
        gr[R16::PC] = 0;
        gr.update();
        assert_eq!(gr.c(), true);
    }

    #[test]
    fn nop() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0);
        gr[R16::PC] = 0;
        gr.update();
        assert_eq!(gr[R16::PC], 1);
    }
    #[test]
    fn di() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xf3);
        gr.cpu.i = true;
        gr.update();
        assert_eq!(gr.cpu.i, false);
    }
    #[test]
    fn ei() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xfb);
        gr.cpu.i = false;
        gr.update();
        assert_eq!(gr.cpu.i, true);
    }

    //jump
    #[test]
    fn jp_n() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xc3);
        gr.mem.set(1, 0xfc);
        gr.mem.set(2, 0xca);
        gr.update();
        assert_eq!(gr[R16::PC], 0xcafc);
    }

    #[test]
    fn jp_hl() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xe9);
        gr[R16::HL] = 0xcafc;
        gr.update();
        assert_eq!(gr[R16::PC], 0xcafc);
    }

    #[test]
    fn jp_c_n() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xca);
        gr.mem.set(1, 0xfc);
        gr.mem.set(2, 0xca);
        gr.set_z(false);
        gr.update();
        assert_eq!(gr[R16::PC], 3);
        gr.set_z(true);
        gr[R16::PC] = 0;
        gr.update();
        assert_eq!(gr[R16::PC], 0xcafc);
    }

    #[test]
    fn jr_d() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x18);
        gr.mem.set(1, 0x4a);
        gr.update();
        assert_eq!(gr[R16::PC], 0x004c);
        gr[R16::PC] = 0;
        gr.mem.set(1, 0xff);
        gr.update();
        assert_eq!(gr[R16::PC], 1);
    }

    #[test]
    fn jr_c_d() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0x20);
        gr.mem.set(1, 0x4a);
        gr.update();
        assert_eq!(gr[R16::PC], 0x004c);
        gr[R16::PC] = 0;
        gr.mem.set(1, 0xff);
        gr.set_z(true);
        gr.update();
        assert_eq!(gr[R16::PC], 2);
    }
    #[test]
    fn call_n() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xcd);
        gr.mem.set(1, 0xfc);
        gr.mem.set(2, 0xca);
        gr[R16::SP] = 6;
        gr.update();
        assert_eq!(gr[R16::PC], 0xcafc);
        assert_eq!(gr.read_u16(4), 0x0003);
        assert_eq!(gr[R16::SP], 4);
    }
    #[test]
    fn call_c_n() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xd4); //CALL NC, u16
        gr.mem.set(1, 0xfc);
        gr.mem.set(2, 0xca);
        gr[R16::SP] = 6;
        gr.set_c(false);
        gr.update();
        assert_eq!(gr[R16::PC], 0xcafc);
        assert_eq!(gr.read_u16(4), 3);
        assert_eq!(gr[R16::SP], 4);
        gr[R16::PC] = 0;
        gr.set_c(true);
        gr.update();
        assert_eq!(gr[R16::PC], 3);
        assert_eq!(gr[R16::SP], 4);
    }

    #[test]
    fn ret() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xc9);
        gr.mem.set(1, 0xfc);
        gr.mem.set(2, 0xca);
        gr[R16::SP] = 1;
        gr.update();
        assert_eq!(gr[R16::PC], 0xcafc);
        assert_eq!(gr[R16::SP], 3);
    }
    #[test]
    fn ret_c() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xd8);
        gr.mem.set(1, 0xfc);
        gr.mem.set(2, 0xca);
        gr[R16::SP] = 1;
        gr.set_c(true);
        gr.update();
        assert_eq!(gr[R16::PC], 0xcafc);
        assert_eq!(gr[R16::SP], 3);
        gr[R16::PC] = 0;
        gr.set_c(false);
        gr.update();
        assert_eq!(gr[R16::PC], 1);
        assert_eq!(gr[R16::SP], 3);
    }
    #[test]
    fn reti() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xD9);
        gr.mem.set(1, 0xfc);
        gr.mem.set(2, 0xca);
        gr[R16::SP] = 1;
        gr.set_i(false);
        gr.update();
        assert_eq!(gr[R16::PC], 0xcafc);
        assert_eq!(gr[R16::SP], 3);
        assert_eq!(gr.i(), true);
    }

    #[test]
    fn rst() {
        let mut gr = GameRusty::new();
        gr.mem.set(0, 0xc7);
        gr[R16::SP] = 3;
        gr.update();
        assert_eq!(gr[R16::PC], 0);
        assert_eq!(gr.read_u16(1), 0x0001);
        assert_eq!(gr[R16::SP], 1);
    }
}
