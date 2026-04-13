`timescale 1ns/1ps
// Quick testbench with 60ms timeout (3M cycles at 50MHz) to distinguish
// performance timeout from correctness bugs
module tb_quick;
    localparam HALT_PC = 32'h8000_0010;
    reg clk = 0;
    reg rst = 1;
    always #10 clk = ~clk;
    initial begin
        rst = 1;
        repeat(10) @(posedge clk);
        @(negedge clk);
        rst = 0;
    end
    reg  [63:0] virtual_sw  = 64'hDEAD_BEEF_1234_5678;
    reg  [ 7:0] virtual_key = 8'b0;
    wire [31:0] virtual_led;
    wire [39:0] virtual_seg;
    student_top u_dut (
        .w_clk_50Mhz  (clk),
        .w_clk_rst    (rst),
        .virtual_key  (virtual_key),
        .virtual_sw   (virtual_sw),
        .virtual_led  (virtual_led),
        .virtual_seg  (virtual_seg)
    );
    wire [31:0] pc = u_dut.u_cpu.u_if.pc_out;
    integer total_cycles = 0;
    integer halt_cnt = 0;
    always @(posedge clk) begin
        if (!rst) begin
            total_cycles <= total_cycles + 1;
            if (pc == HALT_PC) halt_cnt <= halt_cnt + 1;
            else halt_cnt <= 0;
            if (halt_cnt >= 5) begin
                $display("PASS at cycle %0d  LED=0x%08h  SEG=0x%010h",
                         total_cycles, virtual_led, virtual_seg);
                $finish;
            end
        end
    end
    initial begin
        #60_000_000;
        $display("TIMEOUT at 3M cycles. PC=0x%08h  LED=0x%08h  SEG=0x%010h",
                 pc, virtual_led, virtual_seg);
        $finish;
    end
endmodule
