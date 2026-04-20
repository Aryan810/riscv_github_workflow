`timescale 1ns/1ps
//////////////////////////////////////////////////////////////
// Stage 2: Instruction Decode (ID)
//////////////////////////////////////////////////////////////
module decode
#(
    parameter [31:0] RESET = 32'h0000_0000
)
(
    input               clk,
    input               reset,
    
    // Pipeline control
    input               stall,          // Stall the ID stage
    input               flush,          // Flush the ID stage
    
    // From IF/ID register
    input  [31:0]       pc_in,
    input  [31:0]       pc_plus4_in,
    input  [31:0]       instruction_in,
    input               valid_in,
    
    // Register file read data (from pipeline)
    input  [31:0]       reg_rdata1,
    input  [31:0]       reg_rdata2,
    
    // Register file read address (to pipeline)
    output [4:0]        rs1_addr,
    output [4:0]        rs2_addr,
    
    // Outputs to ID/EX register
    output reg [31:0]   pc_out,
    output reg [31:0]   pc_plus4_out,
    output reg [31:0]   rs1_data_out,
    output reg [31:0]   rs2_data_out,
    output reg [31:0]   immediate_out,
    output reg [4:0]    rs1_addr_out,
    output reg [4:0]    rs2_addr_out,
    output reg [4:0]    rd_addr_out,
    output reg [2:0]    funct3_out,
    output reg [6:0]    funct7_out,         // Full funct7
    output reg [6:0]    opcode_out,         // Full opcode
    
    // Control signals to ID/EX
    output reg          alu_src_out,        // 0: rs2, 1: immediate
    output reg          mem_write_out,
    output reg          mem_read_out,
    output reg          mem_to_reg_out,
    output reg          reg_write_out,
    output reg          branch_out,
    output reg          jal_out,
    output reg          jalr_out,
    output reg          lui_out,
    output reg          auipc_out,
    output reg          is_muldiv_out,      // RV32M multiply/divide instruction
    output reg          is_mul_out,         // 1 = multiply, 0 = divide
    output reg          is_macc_out,        // MACC accelerator instruction (8-bit)
    output reg          is_macc32_out,      // MACC accelerator instruction (32-bit)
    output reg          valid_out
);

`include "opcode.vh"

//////////////////////////////////////////////////////////////
// Instruction field extraction
//////////////////////////////////////////////////////////////
wire [6:0] opcode   = instruction_in[`OPCODE];
wire [4:0] rd       = instruction_in[`RD];
wire [2:0] funct3   = instruction_in[`FUNC3];
wire [4:0] rs1      = instruction_in[`RS1];
wire [4:0] rs2      = instruction_in[`RS2];
wire       funct7_5 = instruction_in[`SUBTYPE];  // bit 30
wire [6:0] funct7   = instruction_in[`FUNC7];

// RV32M detection
wire is_muldiv = (opcode == ARITHR) && (funct7 == FUNC7_M);
wire is_mul    = is_muldiv && (funct3[2] == 1'b0);  // MUL/MULH/MULHSU/MULHU

// MACC detection
wire is_macc = (opcode == CUSTOM_0) && (funct7 == 7'b0000000);
wire is_macc32 = (opcode == CUSTOM_0) && (funct7 == 7'b0000001);

// Register addresses for register file
assign rs1_addr = rs1;
assign rs2_addr = rs2;

//////////////////////////////////////////////////////////////
// Immediate generation
//////////////////////////////////////////////////////////////
reg [31:0] immediate;
reg        illegal_inst;

always @(*) begin
    immediate = 32'h0;
    illegal_inst = 1'b0;
    
    case (opcode)
        // I-type: JALR, LOAD, ARITHI
        JALR:   immediate = {{20{instruction_in[31]}}, instruction_in[31:20]};
        LOAD:   immediate = {{20{instruction_in[31]}}, instruction_in[31:20]};
        ARITHI: begin
            // Shift instructions use shamt (lower 5 bits)
            if (funct3 == SLL || funct3 == SR)
                immediate = {27'b0, instruction_in[24:20]};
            else
                immediate = {{20{instruction_in[31]}}, instruction_in[31:20]};
        end
        
        // S-type: STORE
        STORE:  immediate = {{20{instruction_in[31]}}, instruction_in[31:25], instruction_in[11:7]};
        
        // B-type: BRANCH
        BRANCH: immediate = {{19{instruction_in[31]}}, instruction_in[31], instruction_in[7],
                            instruction_in[30:25], instruction_in[11:8], 1'b0};
        
        // U-type: LUI, AUIPC
        LUI:    immediate = {instruction_in[31:12], 12'b0};
        AUIPC:  immediate = {instruction_in[31:12], 12'b0};
        
        // J-type: JAL
        JAL:    immediate = {{11{instruction_in[31]}}, instruction_in[31], instruction_in[19:12],
                            instruction_in[20], instruction_in[30:21], 1'b0};
        
        // R-type: ARITHR - no immediate
        ARITHR: immediate = 32'h0;

        CUSTOM_0: immediate = 32'h0;
        
        default: illegal_inst = 1'b1;
    endcase
end

//////////////////////////////////////////////////////////////
// Control signal generation
//////////////////////////////////////////////////////////////
reg        alu_src;
reg        mem_write;
reg        mem_read;
reg        mem_to_reg;
reg        reg_write;
reg        branch;
reg        jal;
reg        jalr;
reg        lui;
reg        auipc;

always @(*) begin
    // Default values
    alu_src    = 1'b0;
    mem_write  = 1'b0;
    mem_read   = 1'b0;
    mem_to_reg = 1'b0;
    reg_write  = 1'b0;
    branch     = 1'b0;
    jal        = 1'b0;
    jalr       = 1'b0;
    lui        = 1'b0;
    auipc      = 1'b0;
    
    case (opcode)
        LUI: begin
            lui       = 1'b1;
            reg_write = 1'b1;
        end
        
        AUIPC: begin
            auipc     = 1'b1;
            reg_write = 1'b1;
        end
        
        JAL: begin
            jal       = 1'b1;
            reg_write = 1'b1;
        end
        
        JALR: begin
            jalr      = 1'b1;
            alu_src   = 1'b1;
            reg_write = 1'b1;
        end
        
        BRANCH: begin
            branch    = 1'b1;
        end
        
        LOAD: begin
            alu_src    = 1'b1;
            mem_read   = 1'b1;
            mem_to_reg = 1'b1;
            reg_write  = 1'b1;
        end
        
        STORE: begin
            alu_src   = 1'b1;
            mem_write = 1'b1;
        end
        
        ARITHI: begin
            alu_src   = 1'b1;
            reg_write = 1'b1;
        end
        
        ARITHR: begin
            reg_write = 1'b1;
        end

        CUSTOM_0: begin
            if (funct7 == 7'b0000000 && funct3 == MACC_GET) begin
                reg_write = 1'b1; // macc.get writes to rd
            end
            else if (funct7 == 7'b0000001 && funct3 == MACC32_GET) begin
                reg_write = 1'b1; // macc32.get writes to rd
            end
        end
        
        default: begin
            // NOP or unknown - no operation
        end
    endcase
end

//////////////////////////////////////////////////////////////
// ID/EX Pipeline Register
//////////////////////////////////////////////////////////////
always @(posedge clk or negedge reset) begin
    if (!reset) begin
        pc_out          <= 32'h0;
        pc_plus4_out    <= 32'h0;
        rs1_data_out    <= 32'h0;
        rs2_data_out    <= 32'h0;
        immediate_out   <= 32'h0;
        rs1_addr_out    <= 5'h0;
        rs2_addr_out    <= 5'h0;
        rd_addr_out     <= 5'h0;
        funct3_out      <= 3'h0;
        funct7_out      <= 7'h0;
        opcode_out      <= 7'h0;
        alu_src_out     <= 1'b0;
        mem_write_out   <= 1'b0;
        mem_read_out    <= 1'b0;
        mem_to_reg_out  <= 1'b0;
        reg_write_out   <= 1'b0;
        branch_out      <= 1'b0;
        jal_out         <= 1'b0;
        jalr_out        <= 1'b0;
        lui_out         <= 1'b0;
        auipc_out       <= 1'b0;
        is_muldiv_out   <= 1'b0;
        is_mul_out      <= 1'b0;
        is_macc_out     <= 1'b0;
        is_macc32_out   <= 1'b0;
        valid_out       <= 1'b0;
    end
    else if (flush) begin
        // Insert bubble on flush
        pc_out          <= 32'h0;
        pc_plus4_out    <= 32'h0;
        rs1_data_out    <= 32'h0;
        rs2_data_out    <= 32'h0;
        immediate_out   <= 32'h0;
        rs1_addr_out    <= 5'h0;
        rs2_addr_out    <= 5'h0;
        rd_addr_out     <= 5'h0;
        funct3_out      <= 3'h0;
        funct7_out      <= 7'h0;
        opcode_out      <= 7'h0;
        alu_src_out     <= 1'b0;
        mem_write_out   <= 1'b0;
        mem_read_out    <= 1'b0;
        mem_to_reg_out  <= 1'b0;
        reg_write_out   <= 1'b0;
        branch_out      <= 1'b0;
        jal_out         <= 1'b0;
        jalr_out        <= 1'b0;
        lui_out         <= 1'b0;
        auipc_out       <= 1'b0;
        is_muldiv_out   <= 1'b0;
        is_mul_out      <= 1'b0;
        is_macc_out     <= 1'b0;
        is_macc32_out   <= 1'b0;
        valid_out       <= 1'b0;
    end
    else if (!stall) begin
        pc_out          <= pc_in;
        pc_plus4_out    <= pc_plus4_in;
        rs1_data_out    <= reg_rdata1;
        rs2_data_out    <= reg_rdata2;
        immediate_out   <= immediate;
        rs1_addr_out    <= rs1;
        rs2_addr_out    <= rs2;
        rd_addr_out     <= rd;
        funct3_out      <= funct3;
        funct7_out      <= funct7;
        opcode_out      <= opcode;
        alu_src_out     <= alu_src;
        mem_write_out   <= mem_write;
        mem_read_out    <= mem_read;
        mem_to_reg_out  <= mem_to_reg;
        reg_write_out   <= reg_write;
        branch_out      <= branch;
        jal_out         <= jal;
        jalr_out        <= jalr;
        lui_out         <= lui;
        auipc_out       <= auipc;
        is_muldiv_out   <= is_muldiv;
        is_mul_out      <= is_mul;
        is_macc_out     <= is_macc;
        is_macc32_out   <= is_macc32;
        valid_out       <= valid_in & ~illegal_inst;
    end
    // If stalled, hold current values
end

endmodule
