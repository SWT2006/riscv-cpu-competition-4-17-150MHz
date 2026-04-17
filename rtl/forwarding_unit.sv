`timescale 1ns/1ps
// Data forwarding unit.
// forward_a/b encoding: 2'b00=no forward, 2'b10=from EX/MEM, 2'b01=from MEM/WB.
// EX/MEM forwarding takes priority over MEM/WB when both match.
module forwarding_unit (
    input  wire [4:0] idex_rs1_addr,
    input  wire [4:0] idex_rs2_addr,
    input  wire [4:0] exmem_rd_addr,
    input  wire       exmem_reg_write,
    input  wire [4:0] memwb_rd_addr,
    input  wire       memwb_reg_write,
    input  wire       memwb_fwd_valid,  // pre-computed: reg_write & (rd != x0)
    output reg  [1:0] forward_a,
    output reg  [1:0] forward_b
);
    always @(*) begin
        // rs1 forwarding
        if (exmem_reg_write && exmem_rd_addr != 5'b0 && exmem_rd_addr == idex_rs1_addr)
            forward_a = 2'b10;
        else if (memwb_fwd_valid && memwb_rd_addr == idex_rs1_addr)
            forward_a = 2'b01;
        else
            forward_a = 2'b00;

        // rs2 forwarding
        if (exmem_reg_write && exmem_rd_addr != 5'b0 && exmem_rd_addr == idex_rs2_addr)
            forward_b = 2'b10;
        else if (memwb_fwd_valid && memwb_rd_addr == idex_rs2_addr)
            forward_b = 2'b01;
        else
            forward_b = 2'b00;
    end
endmodule
