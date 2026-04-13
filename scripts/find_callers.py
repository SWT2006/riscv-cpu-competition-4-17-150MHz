#!/usr/bin/env python3
# Find all JAL/JALR instructions and branch targets in irom_board.hex

import sys

def sign_ext(val, bits):
    if val >= (1 << (bits-1)):
        val -= (1 << bits)
    return val

with open(r'C:\Users\38560\Desktop\work\riscv-cpu-competition\tb\irom_board.hex') as f:
    hexwords = [int(line.strip(), 16) for line in f if line.strip()]

target_range_start = 0x80000014  # look for callers of code around word 5+
target_range_end   = 0x800000f4

print(f"Looking for branches/calls to range {target_range_start:#010x} - {target_range_end:#010x}")
print()

for i, instr in enumerate(hexwords):
    pc = 0x80000000 + i * 4
    opcode = instr & 0x7f

    if opcode == 0x6f:  # JAL
        j_imm = sign_ext(((instr >> 31) << 20) | (((instr >> 12) & 0xff) << 12) | (((instr >> 20) & 1) << 11) | (((instr >> 21) & 0x3ff) << 1), 21)
        rd = (instr >> 7) & 0x1f
        tgt = pc + j_imm
        if target_range_start <= tgt <= target_range_end:
            print(f"w{i:04d} PC={pc:#010x}: JAL x{rd},{tgt:#010x} [offset={j_imm}]")

print()
print("All calls that reach within the first 100 words (PC 0x80000000-0x80000190):")
for i, instr in enumerate(hexwords):
    pc = 0x80000000 + i * 4
    opcode = instr & 0x7f
    if opcode == 0x6f:  # JAL
        j_imm = sign_ext(((instr >> 31) << 20) | (((instr >> 12) & 0xff) << 12) | (((instr >> 20) & 1) << 11) | (((instr >> 21) & 0x3ff) << 1), 21)
        rd = (instr >> 7) & 0x1f
        tgt = pc + j_imm
        if 0x80000000 <= tgt <= 0x80000190:
            print(f"w{i:04d} PC={pc:#010x}: JAL x{rd},{tgt:#010x} [offset={j_imm}]")
