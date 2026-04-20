`timescale 1ns/1ps
//////////////////////////////////////////////////////////////
// 5-Stage RISC-V Pipeline (RV32I)
// Stages: IF -> ID -> EX -> MEM -> WB
// Memories are integrated in fetch (imem) and memory (dmem) stages
//////////////////////////////////////////////////////////////
module pipe
#(
    parameter [31:0] RESET = 32'h0000_0000
)
(
    input               clk,
    input               reset,
    input               stall,          // External stall
    input               dmem_we_ext,
    input  [9:0]        dmem_addr_ext,
    input  [31:0]       dmem_wdata_ext,
    input  [31:0]       io_rdata,
    output [9:0]        io_addr,
    output              io_we,
    output [31:0]       io_wdata,
    output              exception,
    output [31:0]       pc_out,
    output [31:0]       wb_data_out
);

`include "./opcode.vh"

//////////////////////////////////////////////////////////////
// Pipeline Control Signals
//////////////////////////////////////////////////////////////
wire        stall_if;
wire        stall_id;
wire        stall_ex;
wire        stall_mem;
wire        flush_if;
wire        flush_id;
wire        flush_ex;
wire        branch_taken;
wire [31:0] branch_target;

// Hazard detection
wire        load_use_hazard;

//////////////////////////////////////////////////////////////
// IF Stage Signals
//////////////////////////////////////////////////////////////
wire [31:0] if_pc;
wire [31:0] if_pc_plus4;
wire [31:0] if_instruction;
wire        if_valid;

//////////////////////////////////////////////////////////////
// ID Stage Signals
//////////////////////////////////////////////////////////////
wire [31:0] id_pc;
wire [31:0] id_pc_plus4;
wire [31:0] id_rs1_data;
wire [31:0] id_rs2_data;
wire [31:0] id_immediate;
wire [4:0]  id_rs1_addr;
wire [4:0]  id_rs2_addr;
wire [4:0]  id_rd_addr;
wire [2:0]  id_funct3;
wire [6:0]  id_funct7;
wire [6:0]  id_opcode;
wire        id_alu_src;
wire        id_mem_write;
wire        id_mem_read;
wire        id_mem_to_reg;
wire        id_reg_write;
wire        id_branch;
wire        id_jal;
wire        id_jalr;
wire        id_lui;
wire        id_auipc;
wire        id_is_muldiv;       // RV32M instruction
wire        id_is_mul;          // 1 = multiply, 0 = divide
wire        id_is_macc;         // MACC accelerator instruction (8-bit)
wire        id_is_macc32;       // MACC accelerator instruction (32-bit)
wire        id_valid;

// Register file read addresses from decode stage
wire [4:0]  decode_rs1_addr;
wire [4:0]  decode_rs2_addr;

//////////////////////////////////////////////////////////////
// EX Stage Signals
//////////////////////////////////////////////////////////////
wire [31:0] ex_pc_plus4;
wire [31:0] ex_alu_result;
wire [31:0] ex_rs2_data;
wire [4:0]  ex_rd_addr;
wire [2:0]  ex_funct3;
wire        ex_mem_write;
wire        ex_mem_read;
wire        ex_mem_to_reg;
wire        ex_reg_write;
wire        ex_valid;
wire [4:0]  ex_rs1_addr;
wire [4:0]  ex_rs2_addr;
wire        muldiv_stall;       // RV32M multicycle stall

//////////////////////////////////////////////////////////////
// MEM Stage Signals
//////////////////////////////////////////////////////////////
wire [31:0] mem_pc_plus4;
wire [31:0] mem_alu_result;
wire [31:0] mem_read_data;
wire [4:0]  mem_rd_addr;
wire [2:0]  mem_funct3;
wire        mem_mem_to_reg;
wire        mem_reg_write;
wire        mem_valid;
wire [31:0] mem_forward_data;

//////////////////////////////////////////////////////////////
// WB Stage Signals
//////////////////////////////////////////////////////////////
wire        wb_reg_write_en;
wire [4:0]  wb_rd_addr;
wire [31:0] wb_rd_data;
wire [31:0] wb_forward_data;

//////////////////////////////////////////////////////////////
// Register File
//////////////////////////////////////////////////////////////
reg [31:0] regs [31:1];  // x1-x31 (x0 is hardwired to 0)

// Register file read (combinational) with WB bypass
// If WB is writing to the same register we're reading, use the write data
wire [31:0] reg_rdata1;
wire [31:0] reg_rdata2;

assign reg_rdata1 = (decode_rs1_addr == 5'd0) ? 32'd0 :
                    (wb_reg_write_en && (wb_rd_addr == decode_rs1_addr)) ? wb_rd_data :
                    regs[decode_rs1_addr];
assign reg_rdata2 = (decode_rs2_addr == 5'd0) ? 32'd0 :
                    (wb_reg_write_en && (wb_rd_addr == decode_rs2_addr)) ? wb_rd_data :
                    regs[decode_rs2_addr];

// Register file write (sequential)
integer i;
always @(posedge clk or negedge reset) begin
    if (!reset) begin
        for (i = 1; i < 32; i = i + 1)
            regs[i] <= 32'b0;
    end
    else if (wb_reg_write_en) begin
        regs[wb_rd_addr] <= wb_rd_data;
    end
end

//////////////////////////////////////////////////////////////
// Forwarding Unit
//////////////////////////////////////////////////////////////
reg [1:0] forward_a;
reg [1:0] forward_b;

always @(*) begin
    // Default: no forwarding
    forward_a = 2'b00;
    forward_b = 2'b00;
    
    // Forward from EX/MEM stage (priority 1)
    // BUT NOT if it's a load instruction (data not ready yet)
    // Check if EX/MEM stage writes to rs1
    if (ex_reg_write && ex_valid && (ex_rd_addr != 5'd0) && 
        (ex_rd_addr == id_rs1_addr) && !ex_mem_to_reg) begin
        forward_a = 2'b10;  // Forward from EX/MEM
    end
    // Check if MEM/WB stage writes to rs1
    else if (mem_reg_write && mem_valid && (mem_rd_addr != 5'd0) && 
             (mem_rd_addr == id_rs1_addr)) begin
        forward_a = 2'b01;  // Forward from MEM/WB
    end
    
    // Forward from EX/MEM stage (priority 1)
    // BUT NOT if it's a load instruction (data not ready yet)
    // Check if EX/MEM stage writes to rs2
    if (ex_reg_write && ex_valid && (ex_rd_addr != 5'd0) && 
        (ex_rd_addr == id_rs2_addr) && !ex_mem_to_reg) begin
        forward_b = 2'b10;  // Forward from EX/MEM
    end
    // Check if MEM/WB stage writes to rs2
    else if (mem_reg_write && mem_valid && (mem_rd_addr != 5'd0) && 
             (mem_rd_addr == id_rs2_addr)) begin
        forward_b = 2'b01;  // Forward from MEM/WB
    end
end

//////////////////////////////////////////////////////////////
// Hazard Detection Unit
//////////////////////////////////////////////////////////////
// Load-use hazard: stall when load in EX and dependent instruction in ID
assign load_use_hazard = id_mem_read && id_valid &&
                         ((id_rd_addr == decode_rs1_addr) || 
                          (id_rd_addr == decode_rs2_addr)) &&
                         (id_rd_addr != 5'd0);

//////////////////////////////////////////////////////////////
// Pipeline Control
//////////////////////////////////////////////////////////////
// Stall signals - muldiv_stall freezes the entire pipeline front-end
assign stall_if  = stall || load_use_hazard || muldiv_stall;
assign stall_id  = stall || muldiv_stall;  // Also stall ID when muldiv is busy
assign stall_ex  = stall;
assign stall_mem = stall;

// Flush signals
// - IF flushes on branch taken (wrong path instructions)
// - ID flushes on branch taken OR load-use hazard (insert bubble for stall)
// - Don't flush if muldiv is stalling (we want to keep the instruction)
assign flush_if  = branch_taken && !muldiv_stall;
assign flush_id  = (branch_taken || load_use_hazard) && !muldiv_stall;
assign flush_ex  = 1'b0;  // EX stage not flushed

// Exception signal (placeholder)
assign exception = 1'b0;
assign wb_data_out = wb_reg_write_en ? wb_rd_data : 32'b0;

//////////////////////////////////////////////////////////////
// IF Stage Instance (includes instruction memory)
//////////////////////////////////////////////////////////////
fetch #(.RESET(RESET)) u_fetch (
    .clk            (clk),
    .reset          (reset),
    .stall          (stall_if),
    .flush          (flush_if),
    .branch_taken   (branch_taken),
    .branch_target  (branch_target),
    .pc_out         (if_pc),
    .pc_plus4_out   (if_pc_plus4),
    .instruction_out(if_instruction),
    .valid_out      (if_valid),
    .current_pc     (pc_out)
);

//////////////////////////////////////////////////////////////
// ID Stage Instance
//////////////////////////////////////////////////////////////
decode #(.RESET(RESET)) u_decode (
    .clk            (clk),
    .reset          (reset),
    .stall          (stall_id),
    .flush          (flush_id),
    .pc_in          (if_pc),
    .pc_plus4_in    (if_pc_plus4),
    .instruction_in (if_instruction),
    .valid_in       (if_valid),
    .reg_rdata1     (reg_rdata1),
    .reg_rdata2     (reg_rdata2),
    .rs1_addr       (decode_rs1_addr),
    .rs2_addr       (decode_rs2_addr),
    .pc_out         (id_pc),
    .pc_plus4_out   (id_pc_plus4),
    .rs1_data_out   (id_rs1_data),
    .rs2_data_out   (id_rs2_data),
    .immediate_out  (id_immediate),
    .rs1_addr_out   (id_rs1_addr),
    .rs2_addr_out   (id_rs2_addr),
    .rd_addr_out    (id_rd_addr),
    .funct3_out     (id_funct3),
    .funct7_out     (id_funct7),
    .opcode_out     (id_opcode),
    .alu_src_out    (id_alu_src),
    .mem_write_out  (id_mem_write),
    .mem_read_out   (id_mem_read),
    .mem_to_reg_out (id_mem_to_reg),
    .reg_write_out  (id_reg_write),
    .branch_out     (id_branch),
    .jal_out        (id_jal),
    .jalr_out       (id_jalr),
    .lui_out        (id_lui),
    .auipc_out      (id_auipc),
    .is_muldiv_out  (id_is_muldiv),
    .is_mul_out     (id_is_mul),
    .is_macc_out    (id_is_macc),
    .is_macc32_out  (id_is_macc32),
    .valid_out      (id_valid)
);

//////////////////////////////////////////////////////////////
// EX Stage Instance
//////////////////////////////////////////////////////////////
execute #(.RESET(RESET)) u_execute (
    .clk                (clk),
    .reset              (reset),
    .stall              (stall_ex),
    .flush              (flush_ex),
    .pc_in              (id_pc),
    .pc_plus4_in        (id_pc_plus4),
    .rs1_data_in        (id_rs1_data),
    .rs2_data_in        (id_rs2_data),
    .immediate_in       (id_immediate),
    .rs1_addr_in        (id_rs1_addr),
    .rs2_addr_in        (id_rs2_addr),
    .rd_addr_in         (id_rd_addr),
    .funct3_in          (id_funct3),
    .funct7_in          (id_funct7),
    .opcode_in          (id_opcode),
    .alu_src_in         (id_alu_src),
    .mem_write_in       (id_mem_write),
    .mem_read_in        (id_mem_read),
    .mem_to_reg_in      (id_mem_to_reg),
    .reg_write_in       (id_reg_write),
    .branch_in          (id_branch),
    .jal_in             (id_jal),
    .jalr_in            (id_jalr),
    .lui_in             (id_lui),
    .auipc_in           (id_auipc),
    .is_muldiv_in       (id_is_muldiv),
    .is_mul_in          (id_is_mul),
    .is_macc_in         (id_is_macc),
    .is_macc32_in       (id_is_macc32),
    .valid_in           (id_valid),
    .forward_ex_mem_data(mem_forward_data),   // Forward from MEM stage (not EX output!)
    .forward_mem_wb_data(wb_forward_data),
    .forward_a          (forward_a),
    .forward_b          (forward_b),
    .branch_taken       (branch_taken),
    .branch_target      (branch_target),
    .muldiv_stall       (muldiv_stall),
    .pc_plus4_out       (ex_pc_plus4),
    .alu_result_out     (ex_alu_result),
    .rs2_data_out       (ex_rs2_data),
    .rd_addr_out        (ex_rd_addr),
    .funct3_out         (ex_funct3),
    .mem_write_out      (ex_mem_write),
    .mem_read_out       (ex_mem_read),
    .mem_to_reg_out     (ex_mem_to_reg),
    .reg_write_out      (ex_reg_write),
    .valid_out          (ex_valid),
    .rs1_addr_out       (ex_rs1_addr),
    .rs2_addr_out       (ex_rs2_addr)
);

//////////////////////////////////////////////////////////////
// MEM Stage Instance (includes data memory)
//////////////////////////////////////////////////////////////
memory #(.RESET(RESET)) u_memory (
    .clk                (clk),
    .reset              (reset),
    .stall              (stall_mem),
    .pc_plus4_in        (ex_pc_plus4),
    .alu_result_in      (ex_alu_result),
    .rs2_data_in        (ex_rs2_data),
    .rd_addr_in         (ex_rd_addr),
    .funct3_in          (ex_funct3),
    .mem_write_in       (ex_mem_write),
    .mem_read_in        (ex_mem_read),
    .mem_to_reg_in      (ex_mem_to_reg),
    .reg_write_in       (ex_reg_write),
    .valid_in           (ex_valid),
    .io_we              (io_we),
    .io_addr            (io_addr),
    .io_wdata           (io_wdata),
    .pc_plus4_out       (mem_pc_plus4),
    .alu_result_out     (mem_alu_result),
    .mem_read_data_out  (mem_read_data),
    .rd_addr_out        (mem_rd_addr),
    .funct3_out         (mem_funct3),
    .mem_to_reg_out     (mem_mem_to_reg),
    .reg_write_out      (mem_reg_write),
    .valid_out          (mem_valid),
    .forward_data       (mem_forward_data)
);

//////////////////////////////////////////////////////////////
// WB Stage Instance
//////////////////////////////////////////////////////////////
writeback #(.RESET(RESET)) u_writeback (
    .clk                (clk),
    .reset              (reset),
    .pc_plus4_in        (mem_pc_plus4),
    .alu_result_in      (mem_alu_result),
    .mem_read_data_in   (mem_read_data),
    .rd_addr_in         (mem_rd_addr),
    .funct3_in          (mem_funct3),
    .mem_to_reg_in      (mem_mem_to_reg),
    .reg_write_in       (mem_reg_write),
    .valid_in           (mem_valid),
    .reg_write_en       (wb_reg_write_en),
    .rd_addr            (wb_rd_addr),
    .rd_data            (wb_rd_data),
    .forward_data       (wb_forward_data)
);

endmodule
