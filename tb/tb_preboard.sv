`timescale 1ns/1ps
// =============================================================================
// tb_preboard.sv — Final pre-FPGA simulation testbench
//
// DUT : student_top (full board integration)
//       ├─ cpu_core (5-stage pipeline: IF/ID/EX/MEM/WB)
//       │   ├─ stage_if, stage_id, stage_ex, stage_mem, stage_wb
//       │   ├─ pipe_ifid, pipe_idex, pipe_exmem, pipe_memwb
//       │   ├─ hazard_unit, forwarding_unit, csr_unit
//       ├─ IROM  ← behavioural stub (ip_stubs.sv), loaded from irom_board.hex
//       └─ perip_bridge
//           ├─ dram_driver → DRAM ← behavioural stub (ip_stubs.sv)
//           ├─ counter
//           └─ display_seg → seg7 ← behavioural stub (ip_stubs.sv)
//
// Memory sources (authoritative — irom.coe and dram.coe remain source of truth):
//   IROM : reference_onecycle/irom.coe  →  coe2hex.py  →  tb/irom_board.hex
//   DRAM : reference_onecycle/dram.coe  →  coe2hex.py  →  tb/dram_board.hex
//
// Address map (from perip_bridge.sv):
//   0x8000_0000 – 0x8000_FFFF  IROM  (pc[15:2] = 14-bit word addr)
//   0x8010_0000 – 0x8013_FFFF  DRAM  (perip_addr[15:2] = 14-bit word addr)
//   0x8020_0000                SW0   switches[31:0]
//   0x8020_0004                SW1   switches[63:32]
//   0x8020_0010                KEY
//   0x8020_0020                SEG   (write/read)
//   0x8020_0040                LED   (write)
//   0x8020_0050                CNT   millisecond counter
//
// Program (from irom.coe):
//   Word 0  0x8000_0000 : 00137117  AUIPC x2, 0x137000 → sp = PC + 0x137000
//   Word 1  0x8000_0004 : 62810113  ADDI  x2, x2, 0x628 → sp = 0x8013_7628
//   Word 2  0x8000_0008 : 1cd000ef  JAL   x1, test_suite_1
//   Word 3  0x8000_000C : 6c8000ef  JAL   x1, test_suite_2
//   Word 4  0x8000_0010 : 0000006f  JAL   x0, 0  ← HALT LOOP (PASS anchor)
//
// Termination policy:
//   PASS    : PC holds at 0x8000_0010 for ≥ 5 consecutive cycles
//   FAIL    : PC is frozen at a non-halt address for ≥ 200 consecutive cycles
//             (distinguishes deadlock / HW bug from the intentional halt loop)
//   TIMEOUT : 20 ms simulation time elapsed (1 000 000 cycles at 50 MHz)
//
// Five-stage pipeline awareness:
//   The pipeline takes 5 cycles to prime after reset.  All assertion thresholds
//   are set well beyond pipeline fill latency.  One-cycle branch flushes (JAL,
//   taken branch) are expected and do NOT trigger the stuck detector.
//   Load-use stalls hold the PC for 1 cycle; the stuck threshold of 200 cycles
//   far exceeds any legitimate stall burst.
//
// Run from the tb/ directory so that $readmemh can find irom_board.hex and
// dram_board.hex in the working directory.
// =============================================================================

module tb_preboard;

    // =========================================================================
    // Parameters
    // =========================================================================
    localparam RESET_PC    = 32'h8000_0000;   // PC after reset (stage_if.sv)
    localparam HALT_PC     = 32'h8000_0010;   // JAL x0,0 self-loop (irom.coe word 4)

    localparam DRAM_BASE   = 32'h8010_0000;   // perip_bridge.sv DRAM_BASE
    localparam DRAM_END    = 32'h8014_0000;   // perip_bridge.sv DRAM_END (excl.)
    localparam MMIO_BASE   = 32'h8020_0000;
    localparam LED_ADDR    = 32'h8020_0040;
    localparam SEG_ADDR    = 32'h8020_0020;
    localparam CNT_ADDR    = 32'h8020_0050;

    // First expected instruction from irom.coe (word 0 = AUIPC x2, 0x137000)
    localparam EXPECTED_INSTR0 = 32'h0013_7117;

    // =========================================================================
    // Clock and reset
    // =========================================================================
    reg clk = 0;
    reg rst = 1;

    always #10 clk = ~clk;   // 50 MHz  (20 ns period)

    // Hold reset for 10 rising edges, then release on a falling edge.
    // Releasing on the falling edge ensures the next rising edge sees rst=0
    // cleanly without setup-time violations in a real scenario.
    initial begin
        rst = 1;
        repeat (10) @(posedge clk);
        @(negedge clk);
        rst = 0;
        $display("[%0t ns] RESET released — PC should drive toward 0x8000_0004 next posedge",
                 $time);
    end

    // =========================================================================
    // Virtual I/O (matches student_top port widths)
    // =========================================================================
    reg  [63:0] virtual_sw  = 64'hDEAD_BEEF_1234_5678;
    reg  [ 7:0] virtual_key = 8'b0;
    wire [31:0] virtual_led;
    wire [39:0] virtual_seg;

    // =========================================================================
    // DUT: student_top
    // The IROM and DRAM Vivado IPs are provided by ip_stubs.sv (same compilation).
    // perip_bridge, dram_driver, counter, display_seg are the real RTL modules.
    // =========================================================================
    student_top u_dut (
        .w_clk_50Mhz  (clk),
        .w_clk_rst    (rst),
        .virtual_key  (virtual_key),
        .virtual_sw   (virtual_sw),
        .virtual_led  (virtual_led),
        .virtual_seg  (virtual_seg)
    );

    // =========================================================================
    // Hierarchical signal aliases  (read-only wires into DUT internals)
    // All paths verified against the actual RTL signal declarations.
    // =========================================================================

    // ── Fetch path ────────────────────────────────────────────────────────────
    // pc_out is a 'reg' inside stage_if (u_if inside cpu_core inside u_dut).
    wire [31:0] pc          = u_dut.u_cpu.u_if.pc_out;
    wire [13:0] irom_addr_w = u_dut.irom_word_addr;   // wire in student_top
    wire [31:0] irom_data_w = u_dut.irom_data;        // wire in student_top

    // ── IF/ID pipeline register ───────────────────────────────────────────────
    wire [31:0] ifid_instr  = u_dut.u_cpu.ifid_instruction;   // wire in cpu_core
    wire [31:0] ifid_pc_w   = u_dut.u_cpu.ifid_pc;

    // ── Hazard / branch ───────────────────────────────────────────────────────
    wire        stall_w     = u_dut.u_cpu.stall;
    wire        branch_taken_w  = u_dut.u_cpu.ex_branch_taken;
    wire [31:0] branch_target_w = u_dut.u_cpu.ex_branch_target;

    // ── Writeback ─────────────────────────────────────────────────────────────
    wire        wb_wen      = u_dut.u_cpu.memwb_reg_write;
    wire [4:0]  wb_rd       = u_dut.u_cpu.memwb_rd_addr;
    wire [31:0] wb_data     = u_dut.u_cpu.wb_write_data;

    // ── Peripheral bus (MEM stage drives these) ───────────────────────────────
    wire [31:0] perip_addr_w  = u_dut.perip_addr;
    wire        perip_wen_w   = u_dut.perip_wen;
    wire [1:0]  perip_mask_w  = u_dut.perip_mask;
    wire [31:0] perip_wdata_w = u_dut.perip_wdata;
    wire [31:0] perip_rdata_w = u_dut.perip_rdata;

    // ── Trap ──────────────────────────────────────────────────────────────────
    wire        trap_valid_w = u_dut.u_cpu.ex_trap_valid;

    // =========================================================================
    // Counters and one-shot flags
    // =========================================================================
    integer total_cycles    = 0;
    integer wb_count        = 0;
    integer store_count     = 0;
    integer dram_store_count = 0;
    integer mmio_store_count = 0;
    integer branch_count    = 0;
    integer stall_count_total = 0;
    integer trap_count      = 0;
    integer reset_cycle_cnt = 0;   // counts posedges while rst=1 (for PC init check)

    integer halt_loop_cnt   = 0;   // consecutive cycles at HALT_PC
    integer stuck_cnt       = 0;   // consecutive cycles at same non-halt PC

    reg [31:0] prev_pc      = RESET_PC;

    // One-shot diagnostic flags
    reg first_nonzero_fetch = 0;
    reg first_wb_done       = 0;
    reg first_store_done    = 0;
    reg first_dram_store    = 0;
    reg first_mmio_store    = 0;
    reg first_branch_done   = 0;
    reg irom_word0_checked  = 0;
    reg pc_reset_checked    = 0;
    reg pipeline_prime_checked = 0;
    reg prev_led_valid      = 0;

    reg [31:0] prev_led_r   = 32'b0;

    // =========================================================================
    // CHECK 1: IROM initialisation — during reset irom_addr=0, check word 0
    // This fires at the first posedge (irom_addr is driven combinatorially
    // from pc_out which is 0x8000_0000 during reset → irom_word_addr = 0).
    // =========================================================================
    always @(posedge clk) begin
        if (!irom_word0_checked && irom_addr_w == 14'h0 && (^irom_data_w !== 1'bx)) begin
            irom_word0_checked <= 1;
            if (irom_data_w === EXPECTED_INSTR0) begin
                $display("[%0t ns] CHECK PASS: IROM[0]=0x%08h — matches irom.coe word 0 (AUIPC x2)",
                         $time, irom_data_w);
            end else begin
                $display("[%0t ns] CHECK FAIL: IROM[0]=0x%08h, expected 0x%08h",
                         $time, irom_data_w, EXPECTED_INSTR0);
                $display("         Ensure irom_board.hex was generated from");
                $display("         reference_onecycle/irom.coe via scripts/coe2hex.py");
            end
        end
    end

    // =========================================================================
    // CHECK 2: PC reset value — verify pc_out = RESET_PC while rst is active
    //
    // Must wait for reset_cycle_cnt >= 1 before reading pc.
    // At the very first posedge (t=10ns), pc_out is still X because the
    // stage_if register update and the testbench always-block race at that
    // edge.  By the second reset posedge (t=30ns) the register is guaranteed
    // to hold RESET_PC.
    // =========================================================================
    always @(posedge clk) begin
        if (rst) reset_cycle_cnt <= reset_cycle_cnt + 1;
    end

    always @(posedge clk) begin
        if (rst && reset_cycle_cnt >= 1 && !pc_reset_checked) begin
            pc_reset_checked <= 1;
            if (pc === RESET_PC)
                $display("[%0t ns] CHECK PASS: PC = 0x%08h during reset", $time, pc);
            else
                $display("[%0t ns] CHECK FAIL: PC = 0x%08h during reset, expected 0x%08h",
                         $time, pc, RESET_PC);
        end
    end

    // =========================================================================
    // Main monitor: runs after reset deasserts
    // =========================================================================
    always @(posedge clk) begin
        if (!rst) begin
            total_cycles <= total_cycles + 1;

            // ── Fetch activity ────────────────────────────────────────────────
            if (!first_nonzero_fetch && irom_data_w !== 32'bx && irom_data_w != 32'b0) begin
                first_nonzero_fetch <= 1;
                $display("[%0t ns] FETCH: first non-zero instruction 0x%08h at PC=0x%08h",
                         $time, irom_data_w, pc);
            end

            // ── Writeback activity ────────────────────────────────────────────
            if (wb_wen && wb_rd != 5'b0) begin
                wb_count <= wb_count + 1;
                if (!first_wb_done) begin
                    first_wb_done <= 1;
                    $display("[%0t ns] WB: first register write  x%02d = 0x%08h  (cycle %0d after reset)",
                             $time, wb_rd, wb_data, total_cycles + 1);
                end
            end

            // ── Branch activity ───────────────────────────────────────────────
            if (branch_taken_w) begin
                branch_count <= branch_count + 1;
                if (!first_branch_done) begin
                    first_branch_done <= 1;
                    $display("[%0t ns] BRANCH: first taken branch → target 0x%08h",
                             $time, branch_target_w);
                end
            end

            // ── Stall activity ────────────────────────────────────────────────
            if (stall_w)
                stall_count_total <= stall_count_total + 1;

            // ── Trap activity ─────────────────────────────────────────────────
            if (trap_valid_w) begin
                trap_count <= trap_count + 1;
                $display("[%0t ns] TRAP: trap_valid asserted (mcause handling active)", $time);
            end

            // ── Store activity ────────────────────────────────────────────────
            if (perip_wen_w) begin
                store_count <= store_count + 1;

                if (!first_store_done) begin
                    first_store_done <= 1;
                    $display("[%0t ns] STORE: first memory write  addr=0x%08h  data=0x%08h  mask=%02b",
                             $time, perip_addr_w, perip_wdata_w, perip_mask_w);
                end

                if (perip_addr_w >= DRAM_BASE && perip_addr_w < DRAM_END) begin
                    dram_store_count <= dram_store_count + 1;
                    if (!first_dram_store) begin
                        first_dram_store <= 1;
                        $display("[%0t ns] DRAM STORE: addr=0x%08h  data=0x%08h  mask=%02b",
                                 $time, perip_addr_w, perip_wdata_w, perip_mask_w);
                    end
                end

                if (perip_addr_w >= MMIO_BASE) begin
                    mmio_store_count <= mmio_store_count + 1;
                    if (!first_mmio_store) begin
                        first_mmio_store <= 1;
                        $display("[%0t ns] MMIO STORE: addr=0x%08h  data=0x%08h",
                                 $time, perip_addr_w, perip_wdata_w);
                    end
                    // Detail for known MMIO registers
                    if (perip_addr_w == LED_ADDR)
                        $display("[%0t ns] MMIO LED   ← 0x%08h", $time, perip_wdata_w);
                    if (perip_addr_w == SEG_ADDR)
                        $display("[%0t ns] MMIO SEG   ← 0x%08h", $time, perip_wdata_w);
                    if (perip_addr_w == CNT_ADDR)
                        $display("[%0t ns] MMIO CNT   ← 0x%08h", $time, perip_wdata_w);
                end
            end

            // ── LED change detection ─────────────────────────────────────────
            if (prev_led_valid && virtual_led !== prev_led_r) begin
                $display("[%0t ns] LED changed: 0x%08h → 0x%08h",
                         $time, prev_led_r, virtual_led);
            end
            prev_led_r     <= virtual_led;
            prev_led_valid <= 1;

            // ── Halt and stuck detection ──────────────────────────────────────
            if (pc == HALT_PC) begin
                halt_loop_cnt <= halt_loop_cnt + 1;
                stuck_cnt     <= 0;
            end else if (pc == prev_pc) begin
                stuck_cnt     <= stuck_cnt + 1;
                halt_loop_cnt <= 0;
            end else begin
                stuck_cnt     <= 0;
                halt_loop_cnt <= 0;
            end
            prev_pc <= pc;

        end else begin
            // During reset: keep counters / flags reset
            prev_pc        <= RESET_PC;
            halt_loop_cnt  <= 0;
            stuck_cnt      <= 0;
            prev_led_r     <= 32'b0;
            prev_led_valid <= 0;
        end
    end

    // =========================================================================
    // CHECK 3: Pipeline prime — assert at least 1 WB write within 30 cycles
    // A 5-stage pipeline primes in exactly 5 cycles. If nothing is written
    // back by cycle 30, the pipeline is clearly frozen.
    // =========================================================================
    always @(posedge clk) begin
        if (!rst && total_cycles == 30 && !pipeline_prime_checked) begin
            pipeline_prime_checked <= 1;
            if (wb_count > 0)
                $display("[%0t ns] CHECK PASS: pipeline prime confirmed (%0d WB writes in 30 cycles)",
                         $time, wb_count);
            else begin
                $display("[%0t ns] CHECK FAIL: no WB write in 30 cycles — pipeline frozen?");
                $display("         stall=%b  ifid_instr=0x%08h  pc=0x%08h",
                         stall_w, ifid_instr, pc);
            end
        end
    end

    // =========================================================================
    // CHECK 4: X/Z on PC (combinatorial gate — fires every cycle in active op)
    // Only reports each occurrence; does not stop the simulation.
    // =========================================================================
    always @(posedge clk) begin
        if (!rst && (^pc === 1'bx)) begin
            $display("[%0t ns] CHECK FAIL: PC contains X/Z — check reset logic or branch target",
                     $time);
        end
    end

    // =========================================================================
    // CHECK 5: DRAM store should appear within 500 cycles (stack frame setup)
    // Reported as a warning; does not stop the simulation.
    // =========================================================================
    always @(posedge clk) begin
        if (!rst && total_cycles == 500 && dram_store_count == 0) begin
            $display("[%0t ns] WARN: No DRAM store in 500 cycles.", $time);
            $display("         stack pointer (x2) expected near 0x8013_7628 (DRAM region).");
            $display("         pc=0x%08h  branch_count=%0d  stall_total=%0d",
                     pc, branch_count, stall_count_total);
        end
    end

    // =========================================================================
    // TERMINATION: PASS — PC held in halt loop for ≥ 5 consecutive cycles
    // =========================================================================
    always @(posedge clk) begin
        if (!rst && halt_loop_cnt >= 5) begin
            $display("");
            $display("==================================================================");
            $display("[%0t ns]  *** SIMULATION  P A S S ***", $time);
            $display("  Program completed: PC reached halt loop at 0x%08h", HALT_PC);
            $display("  (irom.coe word 4 = 0x0000006f = JAL x0, 0)");
            $display("------------------------------------------------------------------");
            $display("  Total cycles           : %0d", total_cycles);
            $display("  WB writes              : %0d", wb_count);
            $display("  Memory stores (total)  : %0d", store_count);
            $display("    DRAM stores          : %0d", dram_store_count);
            $display("    MMIO stores          : %0d", mmio_store_count);
            $display("  Branch/jump events     : %0d", branch_count);
            $display("  Load-use stall cycles  : %0d", stall_count_total);
            $display("  Trap events            : %0d", trap_count);
            $display("  LED output             : 0x%08h", virtual_led);
            $display("  SEG output             : 0x%010h", virtual_seg);
            $display("==================================================================");
            $display("");
            $finish;
        end
    end

    // =========================================================================
    // TERMINATION: FAIL — PC stuck at non-halt address for ≥ 200 cycles
    // =========================================================================
    always @(posedge clk) begin
        if (!rst && stuck_cnt >= 200) begin
            $display("");
            $display("==================================================================");
            $display("[%0t ns]  *** SIMULATION  F A I L  (CPU STUCK) ***", $time);
            $display("  PC frozen at 0x%08h for 200+ consecutive cycles", pc);
            $display("  This is not the halt loop (0x%08h)", HALT_PC);
            $display("------------------------------------------------------------------");
            $display("  Total cycles           : %0d", total_cycles);
            $display("  WB writes so far       : %0d", wb_count);
            $display("  DRAM stores so far     : %0d", dram_store_count);
            $display("  Branch events so far   : %0d", branch_count);
            $display("  stall=%b  branch_taken=%b", stall_w, branch_taken_w);
            $display("  ifid_instr=0x%08h  ifid_pc=0x%08h", ifid_instr, ifid_pc_w);
            $display("  perip_addr=0x%08h  perip_wen=%b  perip_wdata=0x%08h",
                     perip_addr_w, perip_wen_w, perip_wdata_w);
            $display("  LED = 0x%08h", virtual_led);
            $display("==================================================================");
            $display("");
            $display("Likely causes:");
            $display("  1. Instruction fetch returning 0x00000000 (NOP infinite loop).");
            $display("     → Check irom_board.hex was written to the tb/ working directory.");
            $display("  2. Branch target computation error (PC loops on itself).");
            $display("  3. Stall signal stuck high (check hazard_unit or forwarding_unit).");
            $display("  4. CSR mtvec = 0 trap loop if ECALL fires early.");
            $display("");
            $finish;
        end
    end

    // =========================================================================
    // TERMINATION: TIMEOUT — 20 ms simulation time (1 000 000 cycles)
    // =========================================================================
    initial begin
        #20_000_000;   // 20 ms at 1 ns resolution
        $display("");
        $display("==================================================================");
        $display("[%0t ns]  *** SIMULATION  T I M E O U T  (20 ms) ***", $time);
        $display("  Final PC               : 0x%08h", pc);
        $display("  Halt loop PC           : 0x%08h  (never reached = %0s)",
                 HALT_PC, halt_loop_cnt > 0 ? "partial" : "not reached");
        $display("------------------------------------------------------------------");
        $display("  Total cycles           : %0d", total_cycles);
        $display("  WB writes              : %0d", wb_count);
        $display("  DRAM stores            : %0d", dram_store_count);
        $display("  MMIO stores            : %0d", mmio_store_count);
        $display("  Branch events          : %0d", branch_count);
        $display("  Load-use stall cycles  : %0d", stall_count_total);
        $display("  Trap events            : %0d", trap_count);
        $display("  LED                    : 0x%08h", virtual_led);
        $display("  SEG                    : 0x%010h", virtual_seg);
        $display("==================================================================");
        if (halt_loop_cnt > 0)
            $display("NOTE: PC was at halt loop at some point (halt_loop_cnt=%0d)", halt_loop_cnt);
        else if (wb_count > 0 && dram_store_count == 0)
            $display("NOTE: Pipeline running but no DRAM stores — stack or address map issue?");
        $display("");
        $finish;
    end

    // =========================================================================
    // Waveform dump
    // Dumps the full hierarchy; for large runs, limit to key instances.
    // =========================================================================
    initial begin
        $dumpfile("tb_preboard.vcd");
        $dumpvars(0, tb_preboard);
    end

endmodule
