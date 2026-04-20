`timescale 1ns/1ps
//////////////////////////////////////////////////////////////
// Stage 1: Instruction Fetch (IF)
// Includes integrated instruction memory
//////////////////////////////////////////////////////////////
module fetch
#(
    parameter [31:0] RESET = 32'h0000_0000
)
(
    input               clk,
    input               reset,
    
    // Pipeline control
    input               stall,          // Stall the IF stage (from hazard unit)
    input               flush,          // Flush the IF stage (on branch taken)
    
    // Branch/Jump control from EX stage
    input               branch_taken,
    input  [31:0]       branch_target,
    
    // Outputs to IF/ID register
    output reg [31:0]   pc_out,
    output reg [31:0]   pc_plus4_out,
    output reg [31:0]   instruction_out,
    output reg          valid_out,
    
    // PC output for debugging
    output [31:0]       current_pc
);

`include "opcode.vh"

//////////////////////////////////////////////////////////////
// Program Counter (declared first for use in memory addressing)
//////////////////////////////////////////////////////////////
reg [31:0] pc;
wire [31:0] pc_next;

//////////////////////////////////////////////////////////////
// Instruction Memory (Integrated)
//////////////////////////////////////////////////////////////
(* ram_style = "block" *)
reg [31:0] imem [0:1023];  // 4KB instruction memory
integer i;

initial begin
    for (i = 0; i < 1024; i = i + 1)
        imem[i] = NOP;
    $readmemh("imem.hex", imem);
end

// Combinational instruction fetch
wire [31:0] fetched_instruction;
assign fetched_instruction = imem[pc[11:2]];

// Next PC selection
assign pc_next = branch_taken ? branch_target : pc + 4;

// Current PC output
assign current_pc = pc;

// PC update
always @(posedge clk or negedge reset) begin
    if (!reset) begin
        pc <= RESET;
    end
    else if (!stall) begin
        pc <= pc_next;
    end
end

//////////////////////////////////////////////////////////////
// IF/ID Pipeline Register
//////////////////////////////////////////////////////////////
always @(posedge clk or negedge reset) begin
    if (!reset) begin
        pc_out          <= RESET;
        pc_plus4_out    <= RESET + 4;
        instruction_out <= NOP;
        valid_out       <= 1'b0;
    end
    else if (flush) begin
        // Insert bubble on flush (branch taken)
        pc_out          <= pc;
        pc_plus4_out    <= pc + 4;
        instruction_out <= NOP;
        valid_out       <= 1'b0;
    end
    else if (!stall) begin
        pc_out          <= pc;
        pc_plus4_out    <= pc + 4;
        instruction_out <= fetched_instruction;
        valid_out       <= 1'b1;
    end
    // If stalled, hold current values
end

endmodule
