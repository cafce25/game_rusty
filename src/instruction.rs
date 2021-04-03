use super::peripheries::{R16, R8};

#[allow(non_camel_case_types)]
#[derive(Debug, Copy, Clone)]
pub enum Instruction {
    //8-bit load
    LD_R_R(R8, R8),
    LD_R_N(R8, u8),
    LD_iR16_R(R16, R8),
    LD_iR16_N(R16, u8),
    LD_R_iR16(R8, R16),
    LD_R_iN16(R8, u16),
    LD_iN_R(u16, R8),
    LD_R_iN(R8, u16),
    LDI_iR16_R(R16, R8),
    LDI_R_iR16(R8, R16),
    LDD_iR16_R(R16, R8),
    LDD_R_iR16(R8, R16),
    //LD_A_BC,
    //LD_A_DE,
    //LD_A_NN,

    //16-bit load
    LD_R16_N(R16, u16),
    LD_iN_R16(u16, R16),
    LD_R16_R16(R16, R16),
    PUSH(R16),
    POP(R16),

    //8-bit arithmetic/logic
    ADD_R(R8),
    ADD_N(u8),
    ADD_iR16(R16),
    ADC_R(R8),
    ADC_N(u8),
    ADC_iR16(R16),
    SUB_R(R8),
    SUB_N(u8),
    SUB_iR16(R16),
    SBC_R(R8),
    SBC_N(u8),
    SBC_iR16(R16),
    AND_R(R8),
    AND_N(u8),
    AND_iR16(R16),
    XOR_R(R8),
    XOR_N(u8),
    XOR_iR16(R16),
    OR_R(R8),
    OR_N(u8),
    OR_iR16(R16),
    CMP_R(R8),
    CMP_N(u8),
    CMP_iR16(R16),
    INC_R(R8),
    INC_iR16(R16),
    DEC_R(R8),
    DEC_iR16(R16),
    DAA,
    CPL,

    //16-bit arithmetic/logic
    ADD_R16(R16),
    INC_R16(R16),
    DEC_R16(R16),
    ADD_SP_D(i8),
    LD_HL_SP_D(i8),

    //rotate and shift
    RLCA,
    RLA,
    RRCA,
    RRA,
    RLC_R(R8),
    RLC_iHL,
    RL_R(R8),
    RL_iHL,
    RRC_R(R8),
    RRC_iHL,
    RR_R(R8),
    RR_iHL,
    SLA_R(R8),
    SLA_iHL,
    SWAP_R(R8),
    SWAP_iHL,
    SRA_R(R8),
    SRA_iHL,
    SRL_R(R8),
    SRL_iHL,

    //1-bit operation
    BIT_N_R(u8, R8),
    BIT_N_iHL(u8),
    SET_N_R(u8, R8),
    SET_N_iHL(u8),
    RES_N_R(u8, R8),
    RES_N_iHL(u8),

    //CPU control
    CCF,
    SCF,
    NOP,
    HALT,
    STOP,
    DI,
    EI,

    //jump
    JP_N(u16),
    JP_HL,
    JP_C_N(Condition, u16),
    JR_D(i8),
    JR_C_D(Condition, i8),
    CALL_N(u16),
    CALL_C_N(Condition, u16),
    RET,
    RET_C(Condition),
    RETI,
    RST(u8),
}

#[derive(Debug, Copy, Clone)]
pub enum Condition {
    NZ,
    Z,
    NC,
    C,
}
