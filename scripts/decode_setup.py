#!/usr/bin/env python3
# Decode setup code for the board test

import sys

def decode_reg(r):
    names = ['x0','ra','sp','gp','tp','t0','t1','t2','s0','s1','a0','a1','a2','a3','a4','a5','a6','a7','s2','s3','s4','s5','s6','s7','s8','s9','s10','s11','t3','t4','t5','t6']
    return names[r] if r < 32 else f'x{r}'

def sign_ext(val, bits):
    if val >= (1 << (bits-1)):
        val -= (1 << bits)
    return val

def decode_instr(word_idx, instr):
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
    u_imm = instr & 0xfffff000

    if opcode == 0x33:
        op = {(0,0):'ADD',(0,0x20):'SUB',(1,0):'SLL',(2,0):'SLT',(3,0):'SLTU',(4,0):'XOR',(5,0):'SRL',(5,0x20):'SRA',(6,0):'OR',(7,0):'AND',
              (0,1):'MUL',(1,1):'MULH',(2,1):'MULHSU',(3,1):'MULHU',(4,1):'DIV',(5,1):'DIVU',(6,1):'REM',(7,1):'REMU'}.get((funct3,funct7), f'R?{funct3},{funct7}')
        return f"  {op} {decode_reg(rd)},{decode_reg(rs1)},{decode_reg(rs2)}"
    elif opcode == 0x13:
        op = {0:'ADDI',1:'SLLI',2:'SLTI',3:'SLTIU',4:'XORI',5:'SRLI',6:'ORI',7:'ANDI'}.get(funct3, f'I?{funct3}')
        if funct3==5 and funct7==0x20: op='SRAI'
        return f"  {op} {decode_reg(rd)},{decode_reg(rs1)},{i_imm}"
    elif opcode == 0x03:
        op = {0:'LB',1:'LH',2:'LW',4:'LBU',5:'LHU'}.get(funct3, f'L?{funct3}')
        return f"  {op} {decode_reg(rd)},{i_imm}({decode_reg(rs1)})"
    elif opcode == 0x23:
        op = {0:'SB',1:'SH',2:'SW'}.get(funct3, f'S?{funct3}')
        return f"  {op} {decode_reg(rs2)},{s_imm}({decode_reg(rs1)})"
    elif opcode == 0x63:
        op = {0:'BEQ',1:'BNE',4:'BLT',5:'BGE',6:'BLTU',7:'BGEU'}.get(funct3, f'B?{funct3}')
        tgt = pc + b_imm
        return f"  {op} {decode_reg(rs1)},{decode_reg(rs2)},{tgt:#010x}"
    elif opcode == 0x6f:
        tgt = pc + j_imm
        return f"  JAL {decode_reg(rd)},{tgt:#010x} [{j_imm:+d}]"
    elif opcode == 0x67:
        return f"  JALR {decode_reg(rd)},{i_imm}({decode_reg(rs1)})"
    elif opcode == 0x37:
        return f"  LUI {decode_reg(rd)},{u_imm:#010x}"
    elif opcode == 0x17:
        return f"  AUIPC {decode_reg(rd)},{u_imm:#x} -> PC+{u_imm:#x}={pc+u_imm:#010x}"
    else:
        return f"  ??? opcode={opcode:#04x}  ({instr:#010x})"

# Read the hex file
with open(r'C:\Users\38560\Desktop\work\riscv-cpu-competition\tb\irom_board.hex') as f:
    hexwords = [int(line.strip(), 16) for line in f if line.strip()]

start = int(sys.argv[1]) if len(sys.argv) > 1 else 0
end = int(sys.argv[2]) if len(sys.argv) > 2 else min(start+60, len(hexwords))

for i in range(start, end):
    if i >= len(hexwords):
        break
    pc = 0x80000000 + i * 4
    print(f"w{i:04d} {pc:#010x}: {decode_instr(i, hexwords[i])}")
