`timescale 1ns/1ps
// MEM stage: drives the peripheral bus, passes raw read data to pipe_memwb.
//
// Sign extension is deferred to the WB stage (stage_wb) to shorten the
// critical path:  exmem_alu_result → DRAM read → pipe_memwb register.
// Previously the sign-extension + WB mux sat between DRAM output and the
// MEM/WB register, adding ~2 ns to the dominant timing-critical path.
module stage_mem (
    input  wire [31:0] exmem_alu_result,  // byte address
    input  wire [31:0] exmem_rs2_data,    // store data (unshifted)
    input  wire        exmem_mem_read,
    input  wire        exmem_mem_write,
    input  wire [2:0]  exmem_mem_funct3,
    // Peripheral bus (to/from perip_bridge)
    output wire [31:0] perip_addr,
    output wire        perip_wen,
    output wire [1:0]  perip_mask,        // 00=byte, 01=halfword, 10=word
    output wire [31:0] perip_wdata,
    input  wire [31:0] perip_rdata,
    // Raw load data — sign extension deferred to WB stage
    output wire [31:0] mem_read_data
);
    assign perip_addr  = exmem_alu_result;
    assign perip_wen   = exmem_mem_write;
    assign perip_mask  = exmem_mem_funct3[1:0];  // funct3[1:0]: 00=B,01=H,10=W
    assign perip_wdata = exmem_rs2_data;

    assign mem_read_data = perip_rdata;

endmodule
