#!/usr/bin/env python3
# Decode function at 0x8000180c and return instructions

words = {
    1535: 0x40a00533,
    1536: 0xf61ff0ef,
    1537: 0x40b00533,
    1538: 0x00028067,
    1539: 0x00050613,  # 0x8000180c
    1540: 0x00000513,
    1541: 0x0015f693,
    1542: 0x00068463,
    1543: 0x00c50533,
    1544: 0x0015d593,
    1545: 0x00161613,
    1546: 0xfe0596e3,
    1547: 0x00008067,
}

def decode_reg(r):
    names = ['x0','ra','sp','gp','tp','t0','t1','t2','s0','s1','a0','a1','a2','a3','a4','a5','a6','a7','s2','s3','s4','s5','s6','s7','s8','s9','s10','s11','t3','t4','t5','t6']
    return names[r] if r < 32 else f'x{r}'

def sign_ext(val, bits):
    if val >= (1 << (bits-1)):
        val -= (1 << bits)
    return val

for word_idx, instr in sorted(words.items()):
    pc = 0x80000000 + word_idx * 4
    opcode = instr & 0x7f
    rd  = (instr >> 7) & 0x1f
    funct3 = (instr >> 12) & 0x7
    rs1 = (instr >> 15) & 0x1f
    rs2 = (instr >> 20) & 0x1f
    funct7 = (instr >> 25) & 0x7f

    i_imm = sign_ext((instr >> 20) & 0xfff, 12)
    s_imm = sign_ext(((instr >> 25) << 5) | ((instr >> 7) & 0x1f), 12)
    b_imm = sign_ext(((instr >> 31) << 12) | (((instr >> 7) & 1) << 11) | (((instr >> 25) & 0x3f) << 5) | (((instr >> 8) & 0xf) << 1), 13)
    j_imm = sign_ext(((instr >> 31) << 20) | (((instr >> 12) & 0xff) << 12) | (((instr >> 20) & 1) << 11) | (((instr >> 21) & 0x3ff) << 1), 21)

    if opcode == 0x33:
        op = {(0,0):'ADD',(0,0x20):'SUB',(1,0):'SLL',(2,0):'SLT',(3,0):'SLTU',(4,0):'XOR',(5,0):'SRL',(5,0x20):'SRA',(6,0):'OR',(7,0):'AND'}.get((funct3,funct7), f'R?{funct3},{funct7}')
        print(f"w{word_idx:04d} PC={pc:#010x}: {op} {decode_reg(rd)},{decode_reg(rs1)},{decode_reg(rs2)}")
    elif opcode == 0x13:
        op = {0:'ADDI',1:'SLLI',2:'SLTI',3:'SLTIU',4:'XORI',5:'SRLI',6:'ORI',7:'ANDI'}.get(funct3, f'I?{funct3}')
        if funct3==5 and funct7==0x20: op='SRAI'
        print(f"w{word_idx:04d} PC={pc:#010x}: {op} {decode_reg(rd)},{decode_reg(rs1)},{i_imm}")
    elif opcode == 0x03:
        op = {0:'LB',1:'LH',2:'LW',4:'LBU',5:'LHU'}.get(funct3, f'L?{funct3}')
        print(f"w{word_idx:04d} PC={pc:#010x}: {op} {decode_reg(rd)},{i_imm}({decode_reg(rs1)})")
    elif opcode == 0x23:
        op = {0:'SB',1:'SH',2:'SW'}.get(funct3, f'S?{funct3}')
        print(f"w{word_idx:04d} PC={pc:#010x}: {op} {decode_reg(rs2)},{s_imm}({decode_reg(rs1)})")
    elif opcode == 0x63:
        op = {0:'BEQ',1:'BNE',4:'BLT',5:'BGE',6:'BLTU',7:'BGEU'}.get(funct3, f'B?{funct3}')
        tgt = pc + b_imm
        print(f"w{word_idx:04d} PC={pc:#010x}: {op} {decode_reg(rs1)},{decode_reg(rs2)},{tgt:#010x}  [offset={b_imm}]")
    elif opcode == 0x6f:
        tgt = pc + j_imm
        print(f"w{word_idx:04d} PC={pc:#010x}: JAL {decode_reg(rd)},{tgt:#010x}  [offset={j_imm}]")
    elif opcode == 0x67:
        print(f"w{word_idx:04d} PC={pc:#010x}: JALR {decode_reg(rd)},{i_imm}({decode_reg(rs1)})")
    elif opcode == 0x37:
        u_imm = instr & 0xfffff000
        print(f"w{word_idx:04d} PC={pc:#010x}: LUI {decode_reg(rd)},{u_imm:#x}")
    elif opcode == 0x17:
        u_imm = instr & 0xfffff000
        print(f"w{word_idx:04d} PC={pc:#010x}: AUIPC {decode_reg(rd)},{u_imm:#x}")
    else:
        print(f"w{word_idx:04d} PC={pc:#010x}: ??? opcode={opcode:#04x}  ({instr:#010x})")
