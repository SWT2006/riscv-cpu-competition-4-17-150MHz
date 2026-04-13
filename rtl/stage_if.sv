`timescale 1ns/1ps
// IF stage: program counter and instruction fetch.
// PC resets to 0x8000_0000 to match the board address map.
module stage_if (
    input  wire        clk,
    input  wire        cpu_rst,
    input  wire        stall,          // hold PC and IF/ID (load-use hazard)
    input  wire        branch_taken,   // redirect PC
    input  wire [31:0] branch_target,
    // IROM interface — full 32-bit PC; the word address is derived externally
    output wire [31:0] irom_word_addr,
    input  wire [31:0] irom_data,
    // Outputs to IF/ID pipeline register
    output reg  [31:0] pc_out,
    output wire [31:0] pc_plus4,
    output wire [31:0] instruction
);
    localparam RESET_PC = 32'h8000_0000;

    always @(posedge clk) begin
        if (cpu_rst)
            pc_out <= RESET_PC;
        else if (branch_taken)
            pc_out <= branch_target;
        else if (!stall)
            pc_out <= pc_out + 32'd4;
    end

    // Expose full PC as the IROM address output.
    // student_top(5).sv derives the 12-bit IROM word address as pc[13:2].
    // riscv_soc.v (sim) derives its 13-bit sim_imem word address as pc[14:2].
    assign irom_word_addr = pc_out;
    assign pc_plus4       = pc_out + 32'd4;
    assign instruction    = irom_data;

endmodule
