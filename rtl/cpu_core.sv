`timescale 1ns/1ps
// 5-stage pipelined RV32IM CPU core.
// Interface matches the fixed top-level contract (reference/student_top(5).sv).
//   cpu_rst  : active-high synchronous reset
//   irom_*   : instruction ROM — irom_addr is the full 32-bit PC;
//              student_top extracts pc[13:2] as the 12-bit IROM word address
//   perip_*  : peripheral bus — routed to perip_bridge in student_top
module cpu_core (
    input  wire        cpu_clk,
    input  wire        cpu_rst,
    // IROM
    output wire [31:0] irom_addr,   // full PC; student_top uses irom_addr[13:2]
    input  wire [31:0] irom_data,
    // Peripheral bus (DRAM + MMIO via perip_bridge)
    output wire [31:0] perip_addr,
    output wire        perip_wen,
    output wire [1:0]  perip_mask,
    output wire [31:0] perip_wdata,
    input  wire [31:0] perip_rdata
);

    // -------------------------------------------------------------------
    // Inter-stage wires
    // -------------------------------------------------------------------

    // IF outputs
    wire [31:0] if_pc, if_pc_plus4, if_instruction;
    wire        if_predicted;          // IF-stage prediction active
    wire [31:0] if_predicted_target;   // predicted next-PC (for EX verification)

    // IF/ID register outputs
    wire [31:0] ifid_pc, ifid_pc_plus4, ifid_instruction;
    wire        ifid_predicted;        // predicted flag from IF/ID register
    wire [31:0] ifid_predicted_target; // predicted target from IF/ID register

    // ID outputs
    wire [31:0] id_rs1_data, id_rs2_data, id_imm, id_csr_rdata;
    wire [4:0]  id_rs1_addr, id_rs2_addr, id_rd_addr, id_alu_op;
    wire [2:0]  id_mem_funct3;
    wire [11:0] id_csr_addr;
    wire        id_alu_src, id_mem_read, id_mem_write, id_reg_write;
    wire [1:0]  id_wb_sel;
    wire        id_branch, id_jal, id_jalr, id_auipc;
    wire        id_ecall, id_mret, id_csr_op;

    // ID/EX register outputs
    wire [31:0] idex_pc, idex_pc_plus4, idex_rs1_data, idex_rs2_data, idex_imm;
    wire [4:0]  idex_rs1_addr, idex_rs2_addr, idex_rd_addr, idex_alu_op;
    wire [2:0]  idex_mem_funct3, idex_funct3;
    wire [11:0] idex_csr_addr;
    wire [31:0] idex_csr_rdata;
    wire        idex_alu_src, idex_mem_read, idex_mem_write, idex_reg_write;
    wire [1:0]  idex_wb_sel;
    wire        idex_branch, idex_jal, idex_jalr, idex_auipc;
    wire        idex_ecall, idex_mret, idex_csr_op;
    wire        idex_predicted;        // predicted flag from ID/EX register
    wire [31:0] idex_predicted_target; // predicted target from ID/EX register

    // EX outputs
    wire [31:0] ex_alu_result, ex_rs2_data_fwd;
    wire        ex_branch_taken;
    wire [31:0] ex_branch_target;
    wire        ex_trap_valid;
    wire [31:0] ex_trap_epc, ex_trap_mcause;
    wire        ex_csr_wen;
    wire [11:0] ex_csr_waddr;
    wire [31:0] ex_csr_wdata;
    wire        div_stall;           // multi-cycle divide stall
    wire        mul_stall;           // pipelined multiply stall (1 cycle)

    // EX/MEM register outputs
    wire [31:0] exmem_pc_plus4, exmem_alu_result, exmem_rs2_data;
    wire [4:0]  exmem_rd_addr;
    wire [2:0]  exmem_mem_funct3;
    wire        exmem_mem_read, exmem_mem_write, exmem_reg_write;
    wire [1:0]  exmem_wb_sel;
    wire [31:0] exmem_fwd_data;      // pre-computed forwarding value

    // MEM output
    wire [31:0] mem_read_data;

    // MEM/WB register outputs
    wire [31:0] memwb_pc_plus4, memwb_alu_result, memwb_mem_data;
    wire [4:0]  memwb_rd_addr;
    wire        memwb_reg_write;
    wire [1:0]  memwb_wb_sel;
    wire [31:0] memwb_write_data;    // pre-computed WB mux (from pipe_memwb)

    // WB output — direct from pre-computed register (no combinational mux)
    wire [31:0] wb_write_data = memwb_write_data;

    // Hazard / forwarding
    wire        hz_stall;            // load-use hazard stall
    wire [1:0]  forward_a, forward_b;

    // CSR outputs to EX
    wire [31:0] csr_mtvec, csr_mepc;
    wire [31:0] id_csr_rdata_raw;

    // -------------------------------------------------------------------
    // Pipeline control
    // -------------------------------------------------------------------
    // Combined EX-stage stall (divide or pipelined multiply)
    wire ex_stall = div_stall | mul_stall;

    // BRAM read-latency stall: synchronous BRAM needs 1 extra cycle after
    // EX/MEM latches to produce valid read data.
    // bram_wait: cleared immediately; set for 1 cycle when LOAD enters MEM.
    // bram_stall = exmem_mem_read & ~bram_wait:
    //   - Active the cycle AFTER EX/MEM latches a LOAD (before bram_wait captures 1)
    //   - Deasserts once bram_wait=1 so pipeline resumes the next cycle
    reg bram_wait;
    always @(posedge clk) begin
        if (cpu_rst)    bram_wait <= 1'b0;
        else if (bram_wait) bram_wait <= 1'b0;   // self-clearing after 1 cycle
        else            bram_wait <= exmem_mem_read;
    end
    wire bram_stall = exmem_mem_read & ~bram_wait;

    // Combined stall: load-use hazard OR EX multi-cycle operation OR BRAM wait
    wire pipeline_stall = hz_stall | ex_stall | bram_stall;

    // -----------------------------------------------------------------
    // Mispredict detection (unified for all prediction sources)
    //
    // Direction mismatch: the actual taken/not-taken outcome differs from
    // what IF predicted.  Covers:
    //   - Forward branch taken (predicted not-taken) → redirect to target
    //   - Backward branch not-taken (predicted taken) → redirect to PC+4
    //   - Unpredicted JALR/ECALL/MRET (predicted=0, taken=1) → redirect
    //   - Correctly predicted JAL/backward-branch → no redirect
    //
    // Target mismatch: RAS-predicted JALR return was taken but the actual
    // target (computed from rs1+imm) differs from what the RAS predicted.
    // Only checked for JALR because branch/JAL targets are deterministic
    // (PC+imm matches in both IF and EX).
    // -----------------------------------------------------------------
    wire direction_mismatch = ex_branch_taken ^ idex_predicted;
    wire target_mismatch    = idex_predicted & ex_branch_taken & idex_jalr
                            & (ex_branch_target != idex_predicted_target);
    wire ex_redirect = direction_mismatch | target_mismatch;

    // Redirect target:
    //   - Predicted taken but actually not taken → fall through to PC+4
    //   - Otherwise (not predicted, or target mismatch) → actual branch target
    wire [31:0] ex_redirect_target = (idex_predicted & ~ex_branch_taken)
                                   ? idex_pc_plus4 : ex_branch_target;

    // Flush: only on misprediction
    wire flush = ex_redirect;

    // -------------------------------------------------------------------
    // Stage 1: IF
    // -------------------------------------------------------------------
    stage_if u_if (
        .clk              (cpu_clk),
        .cpu_rst          (cpu_rst),
        .stall            (pipeline_stall),
        .branch_taken     (ex_redirect),          // mispredict recovery
        .branch_target    (ex_redirect_target),    // correct redirect target
        .irom_word_addr   (irom_addr),
        .irom_data        (irom_data),
        // (BHT feedback removed — BTFNT is purely static, no EX feedback)
        .pc_out           (if_pc),
        .pc_plus4         (if_pc_plus4),
        .instruction      (if_instruction),
        .predicted        (if_predicted),
        .predicted_target (if_predicted_target)
    );

    // -------------------------------------------------------------------
    // IF/ID register
    // -------------------------------------------------------------------
    pipe_ifid u_pipe_ifid (
        .clk                  (cpu_clk),
        .cpu_rst              (cpu_rst),
        .stall                (pipeline_stall),
        .flush                (flush),
        .if_pc                (if_pc),
        .if_pc_plus4          (if_pc_plus4),
        .if_instruction       (if_instruction),
        .if_predicted         (if_predicted),
        .if_predicted_target  (if_predicted_target),
        .ifid_pc              (ifid_pc),
        .ifid_pc_plus4        (ifid_pc_plus4),
        .ifid_instruction     (ifid_instruction),
        .ifid_predicted       (ifid_predicted),
        .ifid_predicted_target(ifid_predicted_target)
    );

    // -------------------------------------------------------------------
    // CSR unit (ID-stage read path + EX-stage write path)
    // -------------------------------------------------------------------
    csr_unit u_csr (
        .clk         (cpu_clk),
        .cpu_rst     (cpu_rst),
        .csr_raddr   (ifid_instruction[31:20]),
        .csr_rdata   (id_csr_rdata_raw),
        .csr_wen     (ex_csr_wen),
        .csr_waddr   (ex_csr_waddr),
        .csr_wdata   (ex_csr_wdata),
        .trap_valid  (ex_trap_valid),
        .trap_epc    (ex_trap_epc),
        .trap_mcause (ex_trap_mcause),
        .mtvec       (csr_mtvec),
        .mepc        (csr_mepc)
    );

    // -------------------------------------------------------------------
    // CSR EX→ID forwarding:
    // When a CSR write is in EX and the next instruction in ID reads the
    // same CSR register, forward the new write data to avoid a RAW hazard.
    // -------------------------------------------------------------------
    wire [31:0] id_csr_rdata_fwd =
        (ex_csr_wen && (ex_csr_waddr == ifid_instruction[31:20]))
            ? ex_csr_wdata
            : id_csr_rdata_raw;

    // -------------------------------------------------------------------
    // Stage 2: ID
    // -------------------------------------------------------------------
    stage_id u_stage_id (
        .clk          (cpu_clk),
        .cpu_rst      (cpu_rst),
        .instruction  (ifid_instruction),
        .wb_reg_write (memwb_reg_write),
        .wb_rd_addr   (memwb_rd_addr),
        .wb_write_data(wb_write_data),
        .csr_rdata    (id_csr_rdata_fwd),
        .rs1_data     (id_rs1_data),
        .rs2_data     (id_rs2_data),
        .imm          (id_imm),
        .rs1_addr     (id_rs1_addr),
        .rs2_addr     (id_rs2_addr),
        .rd_addr      (id_rd_addr),
        .csr_addr     (id_csr_addr),
        .csr_rdata_out(id_csr_rdata),
        .alu_op       (id_alu_op),
        .alu_src      (id_alu_src),
        .mem_read     (id_mem_read),
        .mem_write    (id_mem_write),
        .mem_funct3   (id_mem_funct3),
        .reg_write    (id_reg_write),
        .wb_sel       (id_wb_sel),
        .branch       (id_branch),
        .jal          (id_jal),
        .jalr         (id_jalr),
        .lui          (),           // not needed downstream (alu_op=PASS_B handles it)
        .auipc        (id_auipc),
        .ecall        (id_ecall),
        .mret         (id_mret),
        .csr_op       (id_csr_op)
    );

    // -------------------------------------------------------------------
    // Hazard detection
    // -------------------------------------------------------------------
    hazard_unit u_hazard (
        .idex_mem_read  (idex_mem_read),
        .idex_rd_addr   (idex_rd_addr),
        .ifid_rs1_addr  (ifid_instruction[19:15]),
        .ifid_rs2_addr  (ifid_instruction[24:20]),
        .stall          (hz_stall)
    );

    // -------------------------------------------------------------------
    // ID/EX register
    // flush: load-use stall inserts NOP bubble, branch flush inserts NOP.
    //        Suppressed during bram_stall to avoid corrupting held state.
    // hold:  multi-cycle EX op or BRAM stall keeps the instruction in EX.
    // -------------------------------------------------------------------
    pipe_idex u_pipe_idex (
        .clk            (cpu_clk),
        .cpu_rst        (cpu_rst),
        .flush          ((hz_stall | flush) & ~bram_stall),
        .hold           (ex_stall | bram_stall),
        .id_pc          (ifid_pc),
        .id_pc_plus4    (ifid_pc_plus4),
        .id_rs1_data    (id_rs1_data),
        .id_rs2_data    (id_rs2_data),
        .id_imm         (id_imm),
        .id_rs1_addr    (id_rs1_addr),
        .id_rs2_addr    (id_rs2_addr),
        .id_rd_addr     (id_rd_addr),
        .id_alu_op      (id_alu_op),
        .id_alu_src     (id_alu_src),
        .id_mem_read    (id_mem_read),
        .id_mem_write   (id_mem_write),
        .id_mem_funct3  (id_mem_funct3),
        .id_reg_write   (id_reg_write),
        .id_wb_sel      (id_wb_sel),
        .id_branch      (id_branch),
        .id_jal         (id_jal),
        .id_jalr        (id_jalr),
        .id_auipc       (id_auipc),
        .id_funct3      (ifid_instruction[14:12]),
        .id_ecall       (id_ecall),
        .id_mret        (id_mret),
        .id_csr_op      (id_csr_op),
        .id_csr_rdata   (id_csr_rdata),
        .id_csr_addr    (id_csr_addr),
        .id_predicted         (ifid_predicted),
        .id_predicted_target  (ifid_predicted_target),
        .idex_pc        (idex_pc),
        .idex_pc_plus4  (idex_pc_plus4),
        .idex_rs1_data  (idex_rs1_data),
        .idex_rs2_data  (idex_rs2_data),
        .idex_imm       (idex_imm),
        .idex_rs1_addr  (idex_rs1_addr),
        .idex_rs2_addr  (idex_rs2_addr),
        .idex_rd_addr   (idex_rd_addr),
        .idex_alu_op    (idex_alu_op),
        .idex_alu_src   (idex_alu_src),
        .idex_mem_read  (idex_mem_read),
        .idex_mem_write (idex_mem_write),
        .idex_mem_funct3(idex_mem_funct3),
        .idex_reg_write (idex_reg_write),
        .idex_wb_sel    (idex_wb_sel),
        .idex_branch    (idex_branch),
        .idex_jal       (idex_jal),
        .idex_jalr      (idex_jalr),
        .idex_auipc     (idex_auipc),
        .idex_funct3    (idex_funct3),
        .idex_ecall     (idex_ecall),
        .idex_mret      (idex_mret),
        .idex_csr_op    (idex_csr_op),
        .idex_csr_rdata (idex_csr_rdata),
        .idex_csr_addr  (idex_csr_addr),
        .idex_predicted       (idex_predicted),
        .idex_predicted_target(idex_predicted_target)
    );

    // -------------------------------------------------------------------
    // Forwarding unit
    // -------------------------------------------------------------------
    forwarding_unit u_fwd (
        .idex_rs1_addr  (idex_rs1_addr),
        .idex_rs2_addr  (idex_rs2_addr),
        .exmem_rd_addr  (exmem_rd_addr),
        .exmem_reg_write(exmem_reg_write),
        .memwb_rd_addr  (memwb_rd_addr),
        .memwb_reg_write(memwb_reg_write),
        .forward_a      (forward_a),
        .forward_b      (forward_b)
    );

    // -------------------------------------------------------------------
    // Stage 3: EX
    // -------------------------------------------------------------------
    stage_ex u_ex (
        .clk              (cpu_clk),
        .cpu_rst          (cpu_rst),
        .idex_pc          (idex_pc),
        .idex_pc_plus4    (idex_pc_plus4),
        .idex_rs1_data    (idex_rs1_data),
        .idex_rs2_data    (idex_rs2_data),
        .idex_imm         (idex_imm),
        .idex_rs1_addr    (idex_rs1_addr),
        .idex_rs2_addr    (idex_rs2_addr),
        .idex_alu_op      (idex_alu_op),
        .idex_alu_src     (idex_alu_src),
        .idex_branch      (idex_branch),
        .idex_jal         (idex_jal),
        .idex_jalr        (idex_jalr),
        .idex_auipc       (idex_auipc),
        .idex_funct3      (idex_funct3),
        .idex_ecall       (idex_ecall),
        .idex_mret        (idex_mret),
        .idex_csr_op      (idex_csr_op),
        .idex_csr_rdata   (idex_csr_rdata),
        .idex_csr_addr    (idex_csr_addr),
        .csr_mtvec        (csr_mtvec),
        .csr_mepc         (csr_mepc),
        .forward_a        (forward_a),
        .forward_b        (forward_b),
        .exmem_fwd_data   (exmem_fwd_data),
        .wb_write_data    (wb_write_data),
        .alu_result       (ex_alu_result),
        .rs2_data_fwd     (ex_rs2_data_fwd),
        .branch_taken     (ex_branch_taken),
        .branch_target    (ex_branch_target),
        .trap_valid       (ex_trap_valid),
        .trap_epc         (ex_trap_epc),
        .trap_mcause      (ex_trap_mcause),
        .csr_wen          (ex_csr_wen),
        .csr_waddr        (ex_csr_waddr),
        .csr_wdata        (ex_csr_wdata),
        .div_stall        (div_stall),
        .mul_stall        (mul_stall)
    );

    // -------------------------------------------------------------------
    // EX/MEM register
    // flush: insert NOP bubble while EX multi-cycle op is active.
    // hold:  freeze during BRAM stall so the LOAD stays in MEM.
    // -------------------------------------------------------------------
    pipe_exmem u_pipe_exmem (
        .clk              (cpu_clk),
        .cpu_rst          (cpu_rst),
        .flush            (ex_stall & ~bram_stall),
        .hold             (bram_stall),
        .ex_pc_plus4      (idex_pc_plus4),
        .ex_alu_result    (ex_alu_result),
        .ex_rs2_data      (ex_rs2_data_fwd),
        .ex_rd_addr       (idex_rd_addr),
        .ex_mem_read      (idex_mem_read),
        .ex_mem_write     (idex_mem_write),
        .ex_mem_funct3    (idex_mem_funct3),
        .ex_reg_write     (idex_reg_write),
        .ex_wb_sel        (idex_wb_sel),
        .exmem_pc_plus4   (exmem_pc_plus4),
        .exmem_alu_result (exmem_alu_result),
        .exmem_rs2_data   (exmem_rs2_data),
        .exmem_rd_addr    (exmem_rd_addr),
        .exmem_mem_read   (exmem_mem_read),
        .exmem_mem_write  (exmem_mem_write),
        .exmem_mem_funct3 (exmem_mem_funct3),
        .exmem_reg_write  (exmem_reg_write),
        .exmem_wb_sel     (exmem_wb_sel),
        .exmem_fwd_data   (exmem_fwd_data)
    );

    // -------------------------------------------------------------------
    // Stage 4: MEM — drives peripheral bus
    // -------------------------------------------------------------------
    stage_mem u_mem (
        .exmem_alu_result (exmem_alu_result),
        .exmem_rs2_data   (exmem_rs2_data),
        .exmem_mem_read   (exmem_mem_read),
        .exmem_mem_write  (exmem_mem_write),
        .exmem_mem_funct3 (exmem_mem_funct3),
        .perip_addr       (perip_addr),
        .perip_wen        (perip_wen),
        .perip_mask       (perip_mask),
        .perip_wdata      (perip_wdata),
        .perip_rdata      (perip_rdata),
        .mem_read_data    (mem_read_data)
    );

    // -------------------------------------------------------------------
    // MEM/WB register
    // -------------------------------------------------------------------
    pipe_memwb u_pipe_memwb (
        .clk             (cpu_clk),
        .cpu_rst         (cpu_rst),
        .hold            (bram_stall),
        .mem_pc_plus4    (exmem_pc_plus4),
        .mem_alu_result  (exmem_alu_result),
        .mem_read_data   (mem_read_data),
        .mem_rd_addr     (exmem_rd_addr),
        .mem_reg_write   (exmem_reg_write),
        .mem_wb_sel      (exmem_wb_sel),
        .memwb_pc_plus4  (memwb_pc_plus4),
        .memwb_alu_result(memwb_alu_result),
        .memwb_mem_data  (memwb_mem_data),
        .memwb_rd_addr   (memwb_rd_addr),
        .memwb_reg_write (memwb_reg_write),
        .memwb_wb_sel    (memwb_wb_sel),
        .memwb_write_data(memwb_write_data)
    );

    // -------------------------------------------------------------------
    // Stage 5: WB — pre-computed in pipe_memwb as memwb_write_data.
    // The old stage_wb 3:1 mux is absorbed into the MEM/WB register,
    // so wb_write_data is now a direct register output (see wire decl above).
    // -------------------------------------------------------------------

endmodule
