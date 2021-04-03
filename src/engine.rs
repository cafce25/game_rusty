use super::instruction::{Condition, Instruction as I};
use super::peripheries::{Cpu, Memory, R16, R8};

pub struct Engine<const MEM: usize> {
    cpu: Cpu,
    mem: Memory<MEM>,
}

impl<const MEM: usize> Engine<MEM> {
    pub fn new() -> Self {
        Self {
            cpu: Cpu::new(),
            mem: Memory::<MEM>::new(),
        }
    }

    /// advance the machine one step
    pub fn update(&mut self) {
        let instruction = self.next_instruction();
        match instruction {
            I::LD_R_R(t, s) => self.cpu[t] = self.cpu[s],
            I::LD_R_N(t, s) => self.cpu[t] = s,
            I::LD_iR16_R(t, s) => self.mem[self.cpu[t]] = self.cpu[s],
            I::LD_iR16_N(t, s) => self.mem[self.cpu[t]] = s,
            I::LD_R_iR16(t, s) => self.cpu[t] = self.mem[self.cpu[s]],
            I::LD_R_iN16(t, s) => self.cpu[t] = self.mem[s],
            I::LD_iN_R(t, s) => self.mem[t] = self.cpu[s],
            I::LD_R_iN(t, s) => self.cpu[t] = self.mem[s],
            I::LDI_iR16_R(t, s) => {
                self.mem[self.cpu[t]] = self.cpu[s];
                self.cpu[t] += 1;
            }
            I::LDI_R_iR16(t, s) => {
                self.cpu[t] = self.mem[self.cpu[s]];
                self.cpu[s] += 1;
            }
            I::LDD_iR16_R(t, s) => {
                self.mem[self.cpu[t]] = self.cpu[s];
                self.cpu[t] -= 1;
            }
            I::LDD_R_iR16(t, s) => {
                self.cpu[t] = self.mem[self.cpu[s]];
                self.cpu[s] -= 1;
            }

            I::LD_R16_N(t, s) => self.cpu[t] = s,
            I::LD_iN_R16(t, s) => self.write_u16(t, self.cpu[s]),
            I::LD_R16_R16(t, s) => self.cpu[t] = self.cpu[s],
            I::PUSH(r) => {
                self.cpu[R16::SP] -= 2;
                self.write_u16(self.cpu[R16::SP],  self.cpu[r]);
            }
            I::POP(r) => {
                self.cpu[r] = self.read_u16(self.cpu[R16::SP]);
                self.cpu[R16::SP] += 2;
            }
            _ => {unimplemented!()}
        }
    }

    //TODO make sure read and write u16 do the correct thing (endianness)
    fn read_u16(&mut self, addr: u16) -> u16 {
        (self.mem[addr] as u16) << 8 | self.mem[addr + 1] as u16
    }

    fn write_u16(&mut self, addr: u16, val: u16) {
        self.mem[addr] = (val & 0xff) as u8;
        self.mem[addr + 1] = (val >> 8) as u8;
    }

    /// read immediate 16bit value from current position in memory while adjusting PC
    pub fn imm16(&mut self) -> u16 {
        let imm = self.read_u16(self.cpu[R16::PC]);
        self.cpu[R16::PC] += 2;
        imm
    }

    /// read immediate 8bit value from current position in memory while adjusting PC
    pub fn imm_u8(&mut self) -> u8 {
        let imm = self.mem[self.cpu[R16::PC]];
        self.cpu[R16::PC] += 1;
        imm
    }

    /// read immediate 8bit value from current position in memory while adjusting PC
    pub fn imm_i8(&mut self) -> i8 {
        let imm = self.mem[self.cpu[R16::PC]];
        self.cpu[R16::PC] += 1;
        imm as i8
    }

    /// advances pc and returns the next instruction
    pub fn next_instruction(&mut self) -> I {
        let opcode = self.imm_u8();
        match opcode {
            0x00 => I::NOP,
            0x01 => I::LD_R16_N(R16::BC, self.imm16()),
            0x02 => I::LD_iR16_R(R16::BC, R8::A),
            0x03 => I::INC_R16(R16::BC),
            0x04 => I::INC_R(R8::B),
            0x05 => I::DEC_R(R8::B),
            0x06 => I::LD_R_N(R8::B, self.imm_u8()),
            0x07 => I::RLCA,
            0x08 => I::LD_iN_R16(self.imm16(), R16::SP),
            0x09 => I::ADD_R16(R16::BC),
            0x0a => I::LD_R_iR16(R8::A, R16::BC),
            0x0b => I::DEC_R16(R16::BC),
            0x0c => I::INC_R(R8::C),
            0x0d => I::DEC_R(R8::C),
            0x0e => I::LD_R_N(R8::C, self.imm_u8()),
            0x0f => I::RRCA,

            0x10 => I::STOP,
            0x11 => I::LD_R16_N(R16::DE, self.imm16()),
            0x12 => I::LD_iR16_R(R16::DE, R8::A),
            0x13 => I::INC_R16(R16::DE),
            0x14 => I::INC_R(R8::D),
            0x15 => I::DEC_R(R8::D),
            0x16 => I::LD_R_N(R8::D, self.imm_u8()),
            0x17 => I::RLA,
            0x18 => I::JR_D(self.imm_i8()),
            0x19 => I::ADD_R16(R16::DE),
            0x1a => I::LD_R_iR16(R8::A, R16::DE),
            0x1b => I::DEC_R16(R16::DE),
            0x1c => I::INC_R(R8::E),
            0x1d => I::DEC_R(R8::E),
            0x1e => I::LD_R_N(R8::E, self.imm_u8()),
            0x1f => I::RRA,

            0x20 => I::JR_C_D(Condition::NZ, self.imm_i8()),
            0x21 => I::LD_R16_N(R16::HL, self.imm16()),
            0x22 => I::LDI_iR16_R(R16::HL, R8::A),
            0x23 => I::INC_R16(R16::HL),
            0x24 => I::INC_R(R8::H),
            0x25 => I::DEC_R(R8::H),
            0x26 => I::LD_R_N(R8::H, self.imm_u8()),
            0x27 => I::DAA,
            0x28 => I::JR_C_D(Condition::Z, self.imm_i8()),
            0x29 => I::ADD_R16(R16::HL),
            0x2a => I::LDI_R_iR16(R8::A, R16::HL),
            0x2b => I::DEC_R16(R16::HL),
            0x2c => I::INC_R(R8::L),
            0x2d => I::DEC_R(R8::L),
            0x2e => I::LD_R_N(R8::L, self.imm_u8()),
            0x2f => I::CPL,

            0x30 => I::JR_C_D(Condition::NC, self.imm_i8()),
            0x31 => I::LD_R16_N(R16::SP, self.imm16()),
            0x32 => I::LDD_iR16_R(R16::HL, R8::A),
            0x33 => I::INC_R16(R16::SP),
            0x34 => I::INC_iR16(R16::HL),
            0x35 => I::DEC_iR16(R16::HL),
            0x36 => I::LD_iR16_N(R16::HL, self.imm_u8()),
            0x37 => I::SCF,
            0x38 => I::JR_C_D(Condition::C, self.imm_i8()),
            0x39 => I::ADD_R16(R16::SP),
            0x3a => I::LDD_R_iR16(R8::A, R16::HL),
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
            0x46 => I::LD_R_iR16(R8::B, R16::HL),
            0x47 => I::LD_R_R(R8::B, R8::A),
            0x48 => I::LD_R_R(R8::C, R8::B),
            0x49 => I::LD_R_R(R8::C, R8::C),
            0x4a => I::LD_R_R(R8::C, R8::D),
            0x4b => I::LD_R_R(R8::C, R8::E),
            0x4c => I::LD_R_R(R8::C, R8::H),
            0x4d => I::LD_R_R(R8::C, R8::L),
            0x4e => I::LD_R_iR16(R8::C, R16::HL),
            0x4f => I::LD_R_R(R8::C, R8::A),

            0x50 => I::LD_R_R(R8::D, R8::B),
            0x51 => I::LD_R_R(R8::D, R8::C),
            0x52 => I::LD_R_R(R8::D, R8::D),
            0x53 => I::LD_R_R(R8::D, R8::E),
            0x54 => I::LD_R_R(R8::D, R8::H),
            0x55 => I::LD_R_R(R8::D, R8::L),
            0x56 => I::LD_R_iR16(R8::D, R16::HL),
            0x57 => I::LD_R_R(R8::D, R8::A),
            0x58 => I::LD_R_R(R8::E, R8::B),
            0x59 => I::LD_R_R(R8::E, R8::C),
            0x5a => I::LD_R_R(R8::E, R8::D),
            0x5b => I::LD_R_R(R8::E, R8::E),
            0x5c => I::LD_R_R(R8::E, R8::H),
            0x5d => I::LD_R_R(R8::E, R8::L),
            0x5e => I::LD_R_iR16(R8::E, R16::HL),
            0x5f => I::LD_R_R(R8::E, R8::A),

            0x60 => I::LD_R_R(R8::H, R8::B),
            0x61 => I::LD_R_R(R8::H, R8::C),
            0x62 => I::LD_R_R(R8::H, R8::D),
            0x63 => I::LD_R_R(R8::H, R8::E),
            0x64 => I::LD_R_R(R8::H, R8::H),
            0x65 => I::LD_R_R(R8::H, R8::L),
            0x66 => I::LD_R_iR16(R8::H, R16::HL),
            0x67 => I::LD_R_R(R8::H, R8::A),
            0x68 => I::LD_R_R(R8::L, R8::B),
            0x69 => I::LD_R_R(R8::L, R8::C),
            0x6a => I::LD_R_R(R8::L, R8::D),
            0x6b => I::LD_R_R(R8::L, R8::E),
            0x6c => I::LD_R_R(R8::L, R8::H),
            0x6d => I::LD_R_R(R8::L, R8::L),
            0x6e => I::LD_R_iR16(R8::L, R16::HL),
            0x6f => I::LD_R_R(R8::L, R8::A),

            0x70 => I::LD_iR16_R(R16::HL, R8::B),
            0x71 => I::LD_iR16_R(R16::HL, R8::C),
            0x72 => I::LD_iR16_R(R16::HL, R8::D),
            0x73 => I::LD_iR16_R(R16::HL, R8::E),
            0x74 => I::LD_iR16_R(R16::HL, R8::H),
            0x75 => I::LD_iR16_R(R16::HL, R8::L),
            0x76 => I::HALT,
            0x77 => I::LD_iR16_R(R16::HL, R8::A),
            0x78 => I::LD_R_R(R8::A, R8::B),
            0x79 => I::LD_R_R(R8::A, R8::C),
            0x7a => I::LD_R_R(R8::A, R8::D),
            0x7b => I::LD_R_R(R8::A, R8::E),
            0x7c => I::LD_R_R(R8::A, R8::H),
            0x7d => I::LD_R_R(R8::A, R8::L),
            0x7e => I::LD_R_iR16(R8::A, R16::HL),
            0x7f => I::LD_R_R(R8::A, R8::A),

            0x80 => I::ADD_R(R8::B),
            0x81 => I::ADD_R(R8::C),
            0x82 => I::ADD_R(R8::D),
            0x83 => I::ADD_R(R8::E),
            0x84 => I::ADD_R(R8::H),
            0x85 => I::ADD_R(R8::L),
            0x86 => I::ADD_iR16(R16::HL),
            0x87 => I::ADD_R(R8::A),
            0x88 => I::ADC_R(R8::B),
            0x89 => I::ADC_R(R8::C),
            0x8a => I::ADC_R(R8::D),
            0x8b => I::ADC_R(R8::E),
            0x8c => I::ADC_R(R8::H),
            0x8d => I::ADC_R(R8::L),
            0x8e => I::ADC_iR16(R16::HL),
            0x8f => I::ADC_R(R8::A),

            0x90 => I::SUB_R(R8::B),
            0x91 => I::SUB_R(R8::C),
            0x92 => I::SUB_R(R8::D),
            0x93 => I::SUB_R(R8::E),
            0x94 => I::SUB_R(R8::H),
            0x95 => I::SUB_R(R8::L),
            0x96 => I::SUB_iR16(R16::HL),
            0x97 => I::SUB_R(R8::A),
            0x98 => I::SBC_R(R8::B),
            0x99 => I::SBC_R(R8::C),
            0x9a => I::SBC_R(R8::D),
            0x9b => I::SBC_R(R8::E),
            0x9c => I::SBC_R(R8::H),
            0x9d => I::SBC_R(R8::L),
            0x9e => I::SBC_iR16(R16::HL),
            0x9f => I::SBC_R(R8::A),

            0xa0 => I::AND_R(R8::B),
            0xa1 => I::AND_R(R8::C),
            0xa2 => I::AND_R(R8::D),
            0xa3 => I::AND_R(R8::E),
            0xa4 => I::AND_R(R8::H),
            0xa5 => I::AND_R(R8::L),
            0xa6 => I::AND_iR16(R16::HL),
            0xa7 => I::AND_R(R8::A),
            0xa8 => I::XOR_R(R8::B),
            0xa9 => I::XOR_R(R8::C),
            0xaa => I::XOR_R(R8::D),
            0xab => I::XOR_R(R8::E),
            0xac => I::XOR_R(R8::H),
            0xad => I::XOR_R(R8::L),
            0xae => I::XOR_iR16(R16::HL),
            0xaf => I::XOR_R(R8::A),

            0xb0 => I::OR_R(R8::B),
            0xb1 => I::OR_R(R8::C),
            0xb2 => I::OR_R(R8::D),
            0xb3 => I::OR_R(R8::E),
            0xb4 => I::OR_R(R8::H),
            0xb5 => I::OR_R(R8::L),
            0xb6 => I::OR_iR16(R16::HL),
            0xb7 => I::OR_R(R8::A),
            0xb8 => I::CMP_R(R8::B),
            0xb9 => I::CMP_R(R8::C),
            0xba => I::CMP_R(R8::D),
            0xbb => I::CMP_R(R8::E),
            0xbc => I::CMP_R(R8::H),
            0xbd => I::CMP_R(R8::L),
            0xbe => I::CMP_iR16(R16::HL),
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
            0xe2 => I::LD_iN_R(0xff00 + self.cpu[R8::C] as u16, R8::A),
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
            0xf2 => I::LD_R_iN(R8::A, 0xff00 + self.cpu[R8::C] as u16),
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
}
