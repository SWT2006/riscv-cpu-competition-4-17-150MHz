`timescale 1ns/1ps
// EX stage: ALU, branch comparison, jump/trap target, CSR write data.
//
// Timing-closure optimisations vs. the original single-cycle design:
//   1. DIV/DIVU/REM/REMU use a multi-cycle sequential divider (div_unit).
//      A `div_stall` output freezes the upstream pipeline while it computes.
//   2. Three independent 32×32→64 multiplies are consolidated into a single
//      33×33 signed multiply, reducing DSP48 usage from ~12 to ~4 tiles.
//   3. The EX/MEM forwarding mux (wb_sel ? pc+4 : alu) is pre-computed
//      in pipe_exmem as `exmem_fwd_data`, removing one mux level from the
//      forwarding→branch critical path.
//   4. The 33×33 multiply result is registered (pipelined) so Vivado can map
//      it into DSP48 MREG, breaking the longest combinational cone.
//      MUL/MULH/MULHSU/MULHU now take 2 EX cycles (1 stall cycle).
//
// Misalignment exception detection is intentionally omitted to shorten the
// critical path; the competition software does not require it.
module stage_ex (
    input  wire        clk,            // needed by div_unit FSM
    input  wire        cpu_rst,        // active-high synchronous reset
    input  wire [31:0] idex_pc,
    input  wire [31:0] idex_pc_plus4,
    input  wire [31:0] idex_rs1_data,
    input  wire [31:0] idex_rs2_data,
    input  wire [31:0] idex_imm,
    input  wire [4:0]  idex_rs1_addr,
    input  wire [4:0]  idex_rs2_addr,
    input  wire [4:0]  idex_alu_op,
    input  wire        idex_alu_src,
    input  wire        idex_branch,
    input  wire        idex_jal,
    input  wire        idex_jalr,
    input  wire        idex_auipc,
    input  wire [2:0]  idex_funct3,
    input  wire        idex_ecall,
    input  wire        idex_mret,
    input  wire        idex_csr_op,
    input  wire [31:0] idex_csr_rdata,
    input  wire [11:0] idex_csr_addr,
    // CSR outputs for ECALL/MRET branch targets (registered in csr_unit)
    input  wire [31:0] csr_mtvec,
    input  wire [31:0] csr_mepc,
    // Forwarding inputs
    input  wire [1:0]  forward_a,
    input  wire [1:0]  forward_b,
    input  wire [31:0] exmem_fwd_data,   // pre-computed in pipe_exmem
    input  wire [31:0] wb_write_data,
    // Outputs
    output wire [31:0] alu_result,
    output wire [31:0] rs2_data_fwd,   // forwarded rs2 (for store data)
    output wire        branch_taken,
    output wire [31:0] branch_target,
    // Trap outputs → csr_unit
    output wire        trap_valid,
    output wire [31:0] trap_epc,
    output wire [31:0] trap_mcause,
    // CSR instruction write → csr_unit
    output wire        csr_wen,
    output wire [11:0] csr_waddr,
    output wire [31:0] csr_wdata,
    // Pipeline stall for multi-cycle divide
    output wire        div_stall,
    // Pipeline stall for pipelined multiply (1 extra cycle)
    output wire        mul_stall
);

    localparam ALU_ADD    = 5'd0;
    localparam ALU_SUB    = 5'd1;
    localparam ALU_SLL    = 5'd2;
    localparam ALU_SLT    = 5'd3;
    localparam ALU_SLTU   = 5'd4;
    localparam ALU_XOR    = 5'd5;
    localparam ALU_SRL    = 5'd6;
    localparam ALU_SRA    = 5'd7;
    localparam ALU_OR     = 5'd8;
    localparam ALU_AND    = 5'd9;
    localparam ALU_PASS_B = 5'd10;
    localparam ALU_MUL    = 5'd11;
    localparam ALU_MULH   = 5'd12;
    localparam ALU_MULHSU = 5'd13;
    localparam ALU_MULHU  = 5'd14;
    localparam ALU_DIV    = 5'd15;
    localparam ALU_DIVU   = 5'd16;
    localparam ALU_REM    = 5'd17;
    localparam ALU_REMU   = 5'd18;

    // ---------------------------------------------------------------
    // Forwarding muxes — split into two independent copies per operand
    // to reduce forward_a/forward_b fanout from 32 to 16 per copy.
    //   fwd_a_alu  / fwd_b_alu  : ALU path (arithmetic, logic, shift, MUL)
    //   fwd_a_misc / fwd_b_misc : JALR, branch compare, CSR write, store data
    // (* keep = "true" *) prevents Vivado merging identical always blocks.
    // ---------------------------------------------------------------
    (* keep = "true" *) reg [31:0] fwd_a_alu;
    (* keep = "true" *) reg [31:0] fwd_a_misc;
    always @(*) begin
        case (forward_a)
            2'b10:   fwd_a_alu = exmem_fwd_data;
            2'b01:   fwd_a_alu = wb_write_data;
            default: fwd_a_alu = idex_rs1_data;
        endcase
    end
    always @(*) begin
        case (forward_a)
            2'b10:   fwd_a_misc = exmem_fwd_data;
            2'b01:   fwd_a_misc = wb_write_data;
            default: fwd_a_misc = idex_rs1_data;
        endcase
    end

    (* keep = "true" *) reg [31:0] fwd_b_alu;
    (* keep = "true" *) reg [31:0] fwd_b_misc;
    always @(*) begin
        case (forward_b)
            2'b10:   fwd_b_alu = exmem_fwd_data;
            2'b01:   fwd_b_alu = wb_write_data;
            default: fwd_b_alu = idex_rs2_data;
        endcase
    end
    always @(*) begin
        case (forward_b)
            2'b10:   fwd_b_misc = exmem_fwd_data;
            2'b01:   fwd_b_misc = wb_write_data;
            default: fwd_b_misc = idex_rs2_data;
        endcase
    end

    assign rs2_data_fwd = fwd_b_misc;

    // ---------------------------------------------------------------
    // ALU operand selection (uses _alu copies, keeps placement local)
    // ---------------------------------------------------------------
    wire [31:0] alu_a = idex_auipc ? idex_pc  : fwd_a_alu;
    wire [31:0] alu_b = idex_alu_src ? idex_imm : fwd_b_alu;

    // ---------------------------------------------------------------
    // Consolidated multiply (single 33×33 signed → 66-bit product)
    //
    // Sign/zero-extend to 33 bits based on the multiply variant:
    //   MUL/MULH:   sign-extend both   (lower 32 bits identical either way)
    //   MULHSU:     sign-extend a, zero-extend b
    //   MULHU:      zero-extend both
    //
    // The multiply is placed directly inside a clocked always block so
    // Vivado can map it into DSP48E1 tiles with MREG (output register).
    // The (* use_dsp = "yes" *) attribute is on the registered output —
    // this is the canonical Vivado inference pattern for pipelined DSP48.
    // ---------------------------------------------------------------
    wire mul_a_sign = (idex_alu_op != ALU_MULHU);
    wire mul_b_sign = (idex_alu_op == ALU_MUL || idex_alu_op == ALU_MULH);

    wire signed [32:0] mul_op_a = {mul_a_sign & alu_a[31], alu_a};
    wire signed [32:0] mul_op_b = {mul_b_sign & alu_b[31], alu_b};

    // ---------------------------------------------------------------
    // Multi-cycle divider
    // ---------------------------------------------------------------
    wire is_div_op = (idex_alu_op == ALU_DIV)  || (idex_alu_op == ALU_DIVU)
                   || (idex_alu_op == ALU_REM)  || (idex_alu_op == ALU_REMU);

    wire        div_busy, div_done;
    wire [31:0] div_result;

    // Start a new division when a div op appears and the divider is idle
    // and hasn't just produced a result (done=1 means result is being consumed).
    wire div_start = is_div_op && !div_busy && !div_done;

    wire div_op_signed = (idex_alu_op == ALU_DIV) || (idex_alu_op == ALU_REM);
    wire div_op_rem    = (idex_alu_op == ALU_REM) || (idex_alu_op == ALU_REMU);

    div_unit u_div (
        .clk       (clk),
        .rst       (cpu_rst),
        .start     (div_start),
        .dividend  (alu_a),
        .divisor   (alu_b),
        .op_signed (div_op_signed),
        .op_rem    (div_op_rem),
        .busy      (div_busy),
        .done      (div_done),
        .result    (div_result)
    );

    // Stall upstream while divider is active.
    // On the start cycle (IDLE, is_div_op=1, done=0): stall=1.
    // While computing (busy=1, done=0): stall=1.
    // On the result cycle (done=1): stall=0 → pipeline advances.
    assign div_stall = is_div_op & ~div_done;

    // ---------------------------------------------------------------
    // Pipelined multiply — register the 33×33 product for FPGA timing.
    //
    // Without pipelining, the combinational path is:
    //   forwarding mux → operand select → 33×33 multiply → ALU mux
    // The Vivado timing report shows this as 285 CARRY4 cells / 325
    // logic levels / 54 ns — the single dominant critical path.
    //
    // With this register, multiply takes 2 EX cycles:
    //   Cycle 1: operands → DSP48 multiplier → mul_result_r (MREG)
    //   Cycle 2: mul_result_r → ALU mux → pipe_exmem  (trivial path)
    //
    // The multiply is directly inside the clocked always block with
    // (* use_dsp = "yes" *) on the register.  This is the canonical
    // Vivado coding style for DSP48E1 inference with output register
    // (MREG).  Placing the attribute on a combinational wire does not
    // reliably trigger DSP48 mapping — the SYNTH-10 "Wide multiplier"
    // warnings in the timing report confirm this failure mode.
    //
    // The stall mechanism mirrors div_stall: hold pipe_idex, flush
    // pipe_exmem.
    // ---------------------------------------------------------------
    wire is_mul_op = (idex_alu_op == ALU_MUL)    || (idex_alu_op == ALU_MULH)
                  || (idex_alu_op == ALU_MULHSU)  || (idex_alu_op == ALU_MULHU);

    reg mul_valid;   // 1 = registered multiply result is ready to use
    always @(posedge clk) begin
        if (cpu_rst)
            mul_valid <= 1'b0;
        else
            mul_valid <= is_mul_op & ~mul_valid;
    end

    assign mul_stall = is_mul_op & ~mul_valid;

    // Canonical DSP48 inference pattern: multiply directly in clocked
    // block, attribute on the registered output.  Vivado maps this into
    // DSP48E1 tiles (4 tiles for 33×33 signed) with MREG enabled,
    // replacing the 285-CARRY4-cell combinational chain.
    (* use_dsp = "yes" *)
    reg signed [65:0] mul_result_r;
    always @(posedge clk) begin
        mul_result_r <= mul_op_a * mul_op_b;
    end

    // ---------------------------------------------------------------
    // ALU (RV32IM)
    // ---------------------------------------------------------------
    reg [31:0] alu_out;
    always @(*) begin
        case (idex_alu_op)
            ALU_ADD:    alu_out = alu_a + alu_b;
            ALU_SUB:    alu_out = alu_a - alu_b;
            ALU_SLL:    alu_out = alu_a << alu_b[4:0];
            ALU_SLT:    alu_out = ($signed(alu_a) < $signed(alu_b)) ? 32'd1 : 32'd0;
            ALU_SLTU:   alu_out = (alu_a < alu_b) ? 32'd1 : 32'd0;
            ALU_XOR:    alu_out = alu_a ^ alu_b;
            ALU_SRL:    alu_out = alu_a >> alu_b[4:0];
            ALU_SRA:    alu_out = $signed(alu_a) >>> alu_b[4:0];
            ALU_OR:     alu_out = alu_a | alu_b;
            ALU_AND:    alu_out = alu_a & alu_b;
            ALU_PASS_B: alu_out = alu_b;
            // Multiply — pipelined: use registered result (available on 2nd EX cycle)
            ALU_MUL:    alu_out = mul_result_r[31:0];
            ALU_MULH:   alu_out = mul_result_r[63:32];
            ALU_MULHSU: alu_out = mul_result_r[63:32];
            ALU_MULHU:  alu_out = mul_result_r[63:32];
            // Divide / remainder — from sequential divider
            ALU_DIV:    alu_out = div_result;
            ALU_DIVU:   alu_out = div_result;
            ALU_REM:    alu_out = div_result;
            ALU_REMU:   alu_out = div_result;
            default:    alu_out = 32'b0;
        endcase
    end

    // CSR instructions: rd gets old CSR value (alu_result = csr_rdata)
    assign alu_result = idex_csr_op ? idex_csr_rdata : alu_out;

    // ---------------------------------------------------------------
    // Branch comparison
    // ---------------------------------------------------------------
    reg branch_cond;
    always @(*) begin
        case (idex_funct3)
            3'b000:  branch_cond = (fwd_a_misc == fwd_b_misc);                   // BEQ
            3'b001:  branch_cond = (fwd_a_misc != fwd_b_misc);                   // BNE
            3'b100:  branch_cond = ($signed(fwd_a_misc) < $signed(fwd_b_misc));  // BLT
            3'b101:  branch_cond = ($signed(fwd_a_misc) >= $signed(fwd_b_misc)); // BGE
            3'b110:  branch_cond = (fwd_a_misc < fwd_b_misc);                    // BLTU
            3'b111:  branch_cond = (fwd_a_misc >= fwd_b_misc);                   // BGEU
            default: branch_cond = 1'b0;
        endcase
    end

    // ---------------------------------------------------------------
    // Branch / jump targets
    // ---------------------------------------------------------------
    wire [31:0] pc_branch = idex_pc + idex_imm;
    wire [31:0] jalr_tgt  = (fwd_a_misc + idex_imm) & 32'hFFFF_FFFE;

    assign branch_taken  = idex_jal | idex_jalr | (idex_branch & branch_cond)
                         | idex_ecall | idex_mret;

    assign branch_target = idex_mret  ? csr_mepc  :
                           idex_ecall ? csr_mtvec :
                           idex_jalr  ? jalr_tgt  :
                                        pc_branch;

    // ---------------------------------------------------------------
    // Trap output (ECALL and EBREAK; distinguished by idex_csr_addr)
    // ---------------------------------------------------------------
    assign trap_valid  = idex_ecall;
    assign trap_epc    = idex_pc;
    // ECALL from M-mode: mcause=11; EBREAK: mcause=3
    assign trap_mcause = (idex_csr_addr == 12'h001) ? 32'd3 : 32'd11;

    // ---------------------------------------------------------------
    // CSR write data (for CSR instructions)
    // ---------------------------------------------------------------
    wire [31:0] zimm = {27'b0, idex_rs1_addr};  // zero-extended immediate for CSRRxI

    reg [31:0] csr_wd;
    always @(*) begin
        case (idex_funct3)
            3'b001: csr_wd = fwd_a_misc;                          // CSRRW
            3'b010: csr_wd = idex_csr_rdata | fwd_a_misc;        // CSRRS
            3'b011: csr_wd = idex_csr_rdata & ~fwd_a_misc;       // CSRRC
            3'b101: csr_wd = zimm;                                // CSRRWI
            3'b110: csr_wd = idex_csr_rdata | zimm;              // CSRRSI
            3'b111: csr_wd = idex_csr_rdata & ~zimm;             // CSRRCI
            default: csr_wd = fwd_a_misc;
        endcase
    end

    assign csr_wen   = idex_csr_op;
    assign csr_waddr = idex_csr_addr;
    assign csr_wdata = csr_wd;

endmodule
