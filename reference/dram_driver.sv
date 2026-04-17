`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: dram_driver — BRAM-based data memory
//
// Timing fix: converts the original distributed-RAM (DRAM IP / RAMS64E) to
// inferred Block RAM.  This eliminates:
//   - fo=1756 / fo=3716 routing on DRAM address bits  (paths 4, 6 in timing)
//   - SB/SH read-modify-write combinatorial path through distributed RAM
//   - WHS near-miss (0.058 ns) on RAMS64E hold requirement
//
// BRAM timing model (150 MHz, 6.667 ns period):
//   posedge N  : EX/MEM register drives exmem_alu_result → perip_addr stable
//                shortly after posedge N (Tco_reg ≈ 0.3 ns)
//   posedge N+1: BRAM latches address (setup met: addr stable for 6 ns)
//                bram_stall holds the pipeline frozen this cycle
//   posedge N+2: BRAM output register valid (Tco_BRAM ≈ 2 ns after posedge N+1)
//                bram_stall deasserts; MEM/WB captures rdata_reg
//
// Write timing: STORE in MEM → BRAM write commits at posedge N+1 (correct MEM stage).
// Byte writes: byte_we generated from perip_mask + perip_addr offset.
// Read-first mode: simultaneous read/write to same address returns OLD value.
//
// Interface is unchanged from reference_onecycle/dram_driver — no other file
// changes required beyond cpu_core.sv (bram_stall) and pipe_exmem/memwb (hold).
//////////////////////////////////////////////////////////////////////////////////

module dram_driver(
    input  logic         clk,

    input  logic [17:0]  perip_addr,
    input  logic [31:0]  perip_wdata,
    input  logic [1:0]   perip_mask,
    input  logic         dram_wen,
    output logic [31:0]  perip_rdata
);
    logic [15:0] addr;
    logic [1:0]  offset;
    logic [3:0]  byte_we;
    logic [31:0] wdata_aligned;

    assign addr   = perip_addr[17:2];
    assign offset = perip_addr[1:0];

    // =========================================================================
    // Byte-write enable and data alignment
    // Eliminates read-modify-write dependency on distributed RAM read output.
    // =========================================================================
    always_comb begin
        byte_we       = 4'b0000;
        wdata_aligned = 32'b0;
        if (dram_wen) begin
            case (perip_mask)
                2'b10: begin // SW
                    byte_we       = 4'b1111;
                    wdata_aligned = perip_wdata;
                end
                2'b01: begin // SH
                    case (offset[1])
                        1'b0: begin
                            byte_we       = 4'b0011;
                            wdata_aligned = {16'b0, perip_wdata[15:0]};
                        end
                        default: begin
                            byte_we       = 4'b1100;
                            wdata_aligned = {perip_wdata[15:0], 16'b0};
                        end
                    endcase
                end
                2'b00: begin // SB
                    case (offset)
                        2'b00: begin byte_we = 4'b0001; wdata_aligned = {24'b0, perip_wdata[7:0]};         end
                        2'b01: begin byte_we = 4'b0010; wdata_aligned = {16'b0, perip_wdata[7:0], 8'b0};   end
                        2'b10: begin byte_we = 4'b0100; wdata_aligned = {8'b0, perip_wdata[7:0], 16'b0};   end
                        default: begin byte_we = 4'b1000; wdata_aligned = {perip_wdata[7:0], 24'b0};       end
                    endcase
                end
                default: begin
                    byte_we       = 4'b1111;
                    wdata_aligned = perip_wdata;
                end
            endcase
        end
    end

    // =========================================================================
    // Block RAM — 64K × 32-bit (256 KB)
    // (* ram_style = "block" *) forces RAMB36E1 inference.
    // Synchronous posedge write, synchronous posedge read (1-cycle latency).
    // =========================================================================
    (* ram_style = "block" *) logic [31:0] mem [0:65535];

    // Byte-granular write (posedge, same MEM-stage timing as original)
    always_ff @(posedge clk) begin
        if (byte_we[0]) mem[addr][ 7: 0] <= wdata_aligned[ 7: 0];
        if (byte_we[1]) mem[addr][15: 8] <= wdata_aligned[15: 8];
        if (byte_we[2]) mem[addr][23:16] <= wdata_aligned[23:16];
        if (byte_we[3]) mem[addr][31:24] <= wdata_aligned[31:24];
    end

    // Synchronous read — 1-cycle latency (BRAM output register).
    // Address is presented from exmem_alu_result (EX/MEM register, stable
    // from posedge N + ~0.3 ns). BRAM latches at posedge N+1; data valid
    // at posedge N+1 + ~2 ns. bram_stall in cpu_core holds the pipeline
    // so MEM/WB captures at posedge N+2 with large timing margin.
    logic [31:0] rdata_reg;
    always_ff @(posedge clk) begin
        rdata_reg <= mem[addr];
    end

    // =========================================================================
    // Byte / halfword extraction (combinatorial, identical to original)
    // =========================================================================
    always_comb begin
        perip_rdata = 32'b0;
        case (perip_mask)
            2'b00: // LB / LBU
                case (offset)
                    2'b00: perip_rdata = {24'b0, rdata_reg[7:0]};
                    2'b01: perip_rdata = {24'b0, rdata_reg[15:8]};
                    2'b10: perip_rdata = {24'b0, rdata_reg[23:16]};
                    default: perip_rdata = {24'b0, rdata_reg[31:24]};
                endcase
            2'b01: // LH / LHU
                case (offset[1])
                    1'b0: perip_rdata = {16'b0, rdata_reg[15:0]};
                    default: perip_rdata = {16'b0, rdata_reg[31:16]};
                endcase
            2'b10: perip_rdata = rdata_reg; // LW
            default: perip_rdata = 32'b0;
        endcase
    end

endmodule
