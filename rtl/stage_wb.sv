`timescale 1ns/1ps
// WB stage: sign-extend raw memory data and select write-back source.
//
// Sign extension was moved here from stage_mem to shorten the MEM-stage
// critical path (DRAM read → pipe_memwb register).  All inputs are
// registered outputs of pipe_memwb, so the combinational depth here
// only affects the WB→ID register-file write and WB→EX forwarding paths,
// both of which have ample timing margin.
module stage_wb (
    input  wire [31:0] memwb_alu_result,
    input  wire [31:0] memwb_mem_data,    // raw data from peripheral bus
    input  wire [31:0] memwb_pc_plus4,
    input  wire [1:0]  memwb_wb_sel,      // 00=ALU, 01=MEM, 10=PC+4
    input  wire [2:0]  memwb_mem_funct3,  // load width/sign for sign extension
    output reg  [31:0] wb_write_data
);
    reg [31:0] mem_extended;
    always @(*) begin
        case (memwb_mem_funct3)
            3'b000:  mem_extended = {{24{memwb_mem_data[7]}},  memwb_mem_data[7:0]};
            3'b001:  mem_extended = {{16{memwb_mem_data[15]}}, memwb_mem_data[15:0]};
            3'b010:  mem_extended = memwb_mem_data;
            3'b100:  mem_extended = {24'b0, memwb_mem_data[7:0]};
            3'b101:  mem_extended = {16'b0, memwb_mem_data[15:0]};
            default: mem_extended = memwb_mem_data;
        endcase
    end

    always @(*) begin
        case (memwb_wb_sel)
            2'b01:   wb_write_data = mem_extended;
            2'b10:   wb_write_data = memwb_pc_plus4;
            default: wb_write_data = memwb_alu_result;
        endcase
    end
endmodule
