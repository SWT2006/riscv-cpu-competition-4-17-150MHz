`timescale 1ns/1ps
// MEM/WB pipeline register.
// memwb_write_data: pre-computed writeback mux — same optimisation pattern as
//                   exmem_fwd_data.  Removes one 3:1 mux from the
//                   WB→ID bypass and WB→EX forwarding critical paths.
module pipe_memwb (
    input  wire        clk,
    input  wire        cpu_rst,
    input  wire        hold,          // freeze register (bram_stall)
    // Inputs from MEM
    input  wire [31:0] mem_pc_plus4,
    input  wire [31:0] mem_alu_result,
    input  wire [31:0] mem_read_data,
    input  wire [4:0]  mem_rd_addr,
    input  wire        mem_reg_write,
    input  wire [1:0]  mem_wb_sel,
    // Outputs to WB
    output reg  [31:0] memwb_pc_plus4,
    output reg  [31:0] memwb_alu_result,
    output reg  [31:0] memwb_mem_data,
    output reg  [4:0]  memwb_rd_addr,
    output reg         memwb_reg_write,
    output reg  [1:0]  memwb_wb_sel,
    // Pre-computed writeback data: selects the correct source (ALU/MEM/PC+4)
    // at registration time, so downstream consumers get a direct register output.
    output reg  [31:0] memwb_write_data
);
    always @(posedge clk) begin
        if (cpu_rst) begin
            memwb_pc_plus4   <= 32'b0;
            memwb_alu_result <= 32'b0;
            memwb_mem_data   <= 32'b0;
            memwb_rd_addr    <= 5'b0;
            memwb_reg_write  <= 1'b0;
            memwb_wb_sel     <= 2'b0;
            memwb_write_data <= 32'b0;
        end else if (!hold) begin
            memwb_pc_plus4   <= mem_pc_plus4;
            memwb_alu_result <= mem_alu_result;
            memwb_mem_data   <= mem_read_data;
            memwb_rd_addr    <= mem_rd_addr;
            memwb_reg_write  <= mem_reg_write;
            memwb_wb_sel     <= mem_wb_sel;
            // Pre-compute the WB mux: same logic as stage_wb, but registered.
            memwb_write_data <= (mem_wb_sel == 2'b01) ? mem_read_data :
                                (mem_wb_sel == 2'b10) ? mem_pc_plus4  :
                                                        mem_alu_result;
        end
    end
endmodule
