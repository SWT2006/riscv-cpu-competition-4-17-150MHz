`timescale 1ns/1ps
// EX/MEM pipeline register.
// flush: insert NOP bubble (zeroed control signals) during multi-cycle EX ops.
// exmem_fwd_data: pre-computed forwarding value — removes one mux level from
//                 the EX-stage critical path (was: wb_sel mux in stage_ex).
module pipe_exmem (
    input  wire        clk,
    input  wire        cpu_rst,
    input  wire        flush,         // insert NOP bubble (e.g. div_stall)
    input  wire        hold,          // freeze register (e.g. bram_stall)
    // Inputs from EX
    input  wire [31:0] ex_pc_plus4,
    input  wire [31:0] ex_alu_result,
    input  wire [31:0] ex_rs2_data,
    input  wire [4:0]  ex_rd_addr,
    input  wire        ex_mem_read,
    input  wire        ex_mem_write,
    input  wire [2:0]  ex_mem_funct3,
    input  wire        ex_reg_write,
    input  wire [1:0]  ex_wb_sel,
    // Outputs to MEM / forwarding
    output reg  [31:0] exmem_pc_plus4,
    output reg  [31:0] exmem_alu_result,
    output reg  [31:0] exmem_rs2_data,
    output reg  [4:0]  exmem_rd_addr,
    output reg         exmem_mem_read,
    output reg         exmem_mem_write,
    output reg  [2:0]  exmem_mem_funct3,
    output reg         exmem_reg_write,
    output reg  [1:0]  exmem_wb_sel,
    // Pre-computed forwarding data: selects pc_plus4 when wb_sel==10 (JAL/JALR),
    // otherwise alu_result.  Saves one mux level in the EX forwarding path.
    output reg  [31:0] exmem_fwd_data
);
    always @(posedge clk) begin
        if (cpu_rst || flush) begin
            exmem_pc_plus4    <= 32'b0;
            exmem_alu_result  <= 32'b0;
            exmem_rs2_data    <= 32'b0;
            exmem_rd_addr     <= 5'b0;
            exmem_mem_read    <= 1'b0;
            exmem_mem_write   <= 1'b0;
            exmem_mem_funct3  <= 3'b0;
            exmem_reg_write   <= 1'b0;
            exmem_wb_sel      <= 2'b0;
            exmem_fwd_data    <= 32'b0;
        end else if (!hold) begin
            exmem_pc_plus4    <= ex_pc_plus4;
            exmem_alu_result  <= ex_alu_result;
            exmem_rs2_data    <= ex_rs2_data;
            exmem_rd_addr     <= ex_rd_addr;
            exmem_mem_read    <= ex_mem_read;
            exmem_mem_write   <= ex_mem_write;
            exmem_mem_funct3  <= ex_mem_funct3;
            exmem_reg_write   <= ex_reg_write;
            exmem_wb_sel      <= ex_wb_sel;
            // Pre-compute the forwarding mux that was in stage_ex:
            //   JAL/JALR write pc+4 back, so forward pc+4 instead of alu_result.
            exmem_fwd_data    <= (ex_wb_sel == 2'b10) ? ex_pc_plus4
                                                      : ex_alu_result;
        end
    end
endmodule
