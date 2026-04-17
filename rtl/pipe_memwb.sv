`timescale 1ns/1ps
// MEM/WB pipeline register.
//
// Timing optimisation: sign extension and WB mux are NOT pre-computed here.
// Raw memory data and funct3 are registered; the combinational WB mux
// (including sign extension) lives in stage_wb, AFTER these registers.
// This removes ~2 ns of logic from the DRAM-read critical path.
module pipe_memwb (
    input  wire        clk,
    input  wire        cpu_rst,
    input  wire        hold,          // freeze register (bram_stall)
    // Inputs from MEM
    input  wire [31:0] mem_pc_plus4,
    input  wire [31:0] mem_alu_result,
    input  wire [31:0] mem_read_data,   // raw peripheral read data (no sign ext)
    input  wire [4:0]  mem_rd_addr,
    input  wire        mem_reg_write,
    input  wire [1:0]  mem_wb_sel,
    input  wire [2:0]  mem_mem_funct3,  // needed for WB-stage sign extension
    // Outputs to WB
    output reg  [31:0] memwb_pc_plus4,
    output reg  [31:0] memwb_alu_result,
    output reg  [31:0] memwb_mem_data,  // raw — sign extension in stage_wb
    output reg  [4:0]  memwb_rd_addr,
    output reg         memwb_reg_write,
    output reg  [1:0]  memwb_wb_sel,
    output reg  [2:0]  memwb_mem_funct3,
    output reg         memwb_fwd_valid   // pre-computed: reg_write & (rd != x0)
);
    always @(posedge clk) begin
        if (cpu_rst) begin
            memwb_pc_plus4   <= 32'b0;
            memwb_alu_result <= 32'b0;
            memwb_mem_data   <= 32'b0;
            memwb_rd_addr    <= 5'b0;
            memwb_reg_write  <= 1'b0;
            memwb_wb_sel     <= 2'b0;
            memwb_mem_funct3 <= 3'b0;
            memwb_fwd_valid  <= 1'b0;
        end else if (!hold) begin
            memwb_pc_plus4   <= mem_pc_plus4;
            memwb_alu_result <= mem_alu_result;
            memwb_mem_data   <= mem_read_data;
            memwb_rd_addr    <= mem_rd_addr;
            memwb_reg_write  <= mem_reg_write;
            memwb_wb_sel     <= mem_wb_sel;
            memwb_mem_funct3 <= mem_mem_funct3;
            memwb_fwd_valid  <= mem_reg_write & (mem_rd_addr != 5'b0);
        end
    end
endmodule
