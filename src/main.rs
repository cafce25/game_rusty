mod peripheries;

fn main() {
    let m = peripheries::Cpu {
        r: peripheries::Registers {
            r16: peripheries::Registers16 {
                af: 0x00f0,
                bc: 0x1234,
                de: 0x5678,
                hl: 0x9abc,
                sp: 0xdef0,
                pc: 0x1234,
            },
        },
    };
}
