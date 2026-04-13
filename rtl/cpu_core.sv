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

    // IF/ID register outputs
    wire [31:0] ifid_pc, ifid_pc_plus4, ifid_instruction;

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

    // EX outputs
    wire [31:0] ex_alu_result, ex_rs2_data_fwd;
    wire        ex_branch_taken;
    wire [31:0] ex_branch_target;
    wire        ex_trap_valid;
    wire [31:0] ex_trap_epc, ex_trap_mcause;
    wire        ex_csr_wen;
    wire [11:0] ex_csr_waddr;
    wire [31:0] ex_csr_wdata;

    // EX/MEM register outputs
    wire [31:0] exmem_pc_plus4, exmem_alu_result, exmem_rs2_data;
    wire [4:0]  exmem_rd_addr;
    wire [2:0]  exmem_mem_funct3;
    wire        exmem_mem_read, exmem_mem_write, exmem_reg_write;
    wire [1:0]  exmem_wb_sel;

    // MEM output
    wire [31:0] mem_read_data;

    // MEM/WB register outputs
    wire [31:0] memwb_pc_plus4, memwb_alu_result, memwb_mem_data;
    wire [4:0]  memwb_rd_addr;
    wire        memwb_reg_write;
    wire [1:0]  memwb_wb_sel;

    // WB output
    wire [31:0] wb_write_data;

    // Hazard / forwarding
    wire        stall;
    wire [1:0]  forward_a, forward_b;

    // CSR outputs to EX
    wire [31:0] csr_mtvec, csr_mepc;
    wire [31:0] id_csr_rdata_raw;

    // -------------------------------------------------------------------
    // Flush: any taken branch or trap redirects the PC
    // -------------------------------------------------------------------
    wire flush = ex_branch_taken;

    // -------------------------------------------------------------------
    // Stage 1: IF
    // -------------------------------------------------------------------
    stage_if u_if (
        .clk           (cpu_clk),
        .cpu_rst       (cpu_rst),
        .stall         (stall),
        .branch_taken  (ex_branch_taken),
        .branch_target (ex_branch_target),
        .irom_word_addr(irom_addr),
        .irom_data     (irom_data),
        .pc_out        (if_pc),
        .pc_plus4      (if_pc_plus4),
        .instruction   (if_instruction)
    );

    // -------------------------------------------------------------------
    // IF/ID register
    // -------------------------------------------------------------------
    pipe_ifid u_pipe_ifid (
        .clk             (cpu_clk),
        .cpu_rst         (cpu_rst),
        .stall           (stall),
        .flush           (flush),
        .if_pc           (if_pc),
        .if_pc_plus4     (if_pc_plus4),
        .if_instruction  (if_instruction),
        .ifid_pc         (ifid_pc),
        .ifid_pc_plus4   (ifid_pc_plus4),
        .ifid_instruction(ifid_instruction)
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
        .stall          (stall)
    );

    // -------------------------------------------------------------------
    // ID/EX register
    // -------------------------------------------------------------------
    pipe_idex u_pipe_idex (
        .clk            (cpu_clk),
        .cpu_rst        (cpu_rst),
        .flush          (stall | flush),   // stall inserts NOP bubble in EX
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
        .idex_csr_addr  (idex_csr_addr)
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
        .exmem_alu_result (exmem_alu_result),
        .exmem_wb_sel     (exmem_wb_sel),
        .exmem_pc_plus4   (exmem_pc_plus4),
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
        .csr_wdata        (ex_csr_wdata)
    );

    // -------------------------------------------------------------------
    // EX/MEM register
    // -------------------------------------------------------------------
    pipe_exmem u_pipe_exmem (
        .clk              (cpu_clk),
        .cpu_rst          (cpu_rst),
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
        .exmem_wb_sel     (exmem_wb_sel)
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
        .memwb_wb_sel    (memwb_wb_sel)
    );

    // -------------------------------------------------------------------
    // Stage 5: WB
    // -------------------------------------------------------------------
    stage_wb u_wb (
        .memwb_alu_result(memwb_alu_result),
        .memwb_mem_data  (memwb_mem_data),
        .memwb_pc_plus4  (memwb_pc_plus4),
        .memwb_wb_sel    (memwb_wb_sel),
        .wb_write_data   (wb_write_data)
    );

endmodule
