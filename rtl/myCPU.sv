`timescale 1ns/1ps
// myCPU: thin integration wrapper around cpu_core.
//
// Purpose: provide the exact module name and port interface required by the
// fixed top-level contract in reference/student_top(5).sv.
//
// This module contains no logic of its own.  All CPU behaviour lives in
// cpu_core and its pipeline sub-modules.
//
// Port notes:
//   irom_addr [31:0] — full 32-bit program counter.
//     student_top(5).sv receives this as "pc" and derives the 12-bit IROM
//     word address internally:  inst_addr = pc[13:2]
//   All other ports are passed through 1:1 to cpu_core.
module myCPU (
    input  wire        cpu_rst,
    input  wire        cpu_clk,

    // Instruction ROM interface
    // student_top slices irom_addr[13:2] to form the 12-bit IROM word address.
    output wire [31:0] irom_addr,
    input  wire [31:0] irom_data,

    // Peripheral bus (DRAM + MMIO, routed through perip_bridge in student_top)
    output wire [31:0] perip_addr,
    output wire        perip_wen,
    output wire [1:0]  perip_mask,
    output wire [31:0] perip_wdata,
    input  wire [31:0] perip_rdata
);

    cpu_core u_core (
        .cpu_clk     (cpu_clk),
        .cpu_rst     (cpu_rst),
        .irom_addr   (irom_addr),
        .irom_data   (irom_data),
        .perip_addr  (perip_addr),
        .perip_wen   (perip_wen),
        .perip_mask  (perip_mask),
        .perip_wdata (perip_wdata),
        .perip_rdata (perip_rdata)
    );

endmodule
