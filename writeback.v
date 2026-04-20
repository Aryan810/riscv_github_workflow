`timescale 1ns/1ps
//////////////////////////////////////////////////////////////
// Stage 5: Write Back (WB)
//////////////////////////////////////////////////////////////
module writeback
#(
    parameter [31:0] RESET = 32'h0000_0000
)
(
    input               clk,
    input               reset,
    
    // From MEM/WB register
    input  [31:0]       pc_plus4_in,
    input  [31:0]       alu_result_in,
    input  [31:0]       mem_read_data_in,
    input  [4:0]        rd_addr_in,
    input  [2:0]        funct3_in,
    
    // Control signals from MEM/WB
    input               mem_to_reg_in,
    input               reg_write_in,
    input               valid_in,
    
    // Outputs to register file
    output              reg_write_en,
    output [4:0]        rd_addr,
    output [31:0]       rd_data,
    
    // For forwarding (MEM/WB result)
    output [31:0]       forward_data
);

`include "opcode.vh"

//////////////////////////////////////////////////////////////
// Load data alignment and sign extension
//////////////////////////////////////////////////////////////
reg [31:0] load_data;
wire [1:0] byte_offset = alu_result_in[1:0];

always @(*) begin
    load_data = mem_read_data_in;
    
    if (mem_to_reg_in) begin
        case (funct3_in)
            LB: begin
                // Load byte (signed)
                case (byte_offset)
                    2'b00: load_data = {{24{mem_read_data_in[7]}},  mem_read_data_in[7:0]};
                    2'b01: load_data = {{24{mem_read_data_in[15]}}, mem_read_data_in[15:8]};
                    2'b10: load_data = {{24{mem_read_data_in[23]}}, mem_read_data_in[23:16]};
                    2'b11: load_data = {{24{mem_read_data_in[31]}}, mem_read_data_in[31:24]};
                endcase
            end
            
            LH: begin
                // Load halfword (signed)
                if (byte_offset[1])
                    load_data = {{16{mem_read_data_in[31]}}, mem_read_data_in[31:16]};
                else
                    load_data = {{16{mem_read_data_in[15]}}, mem_read_data_in[15:0]};
            end
            
            LW: begin
                // Load word
                load_data = mem_read_data_in;
            end
            
            LBU: begin
                // Load byte unsigned
                case (byte_offset)
                    2'b00: load_data = {24'h0, mem_read_data_in[7:0]};
                    2'b01: load_data = {24'h0, mem_read_data_in[15:8]};
                    2'b10: load_data = {24'h0, mem_read_data_in[23:16]};
                    2'b11: load_data = {24'h0, mem_read_data_in[31:24]};
                endcase
            end
            
            LHU: begin
                // Load halfword unsigned
                if (byte_offset[1])
                    load_data = {16'h0, mem_read_data_in[31:16]};
                else
                    load_data = {16'h0, mem_read_data_in[15:0]};
            end
            
            default: load_data = mem_read_data_in;
        endcase
    end
end

//////////////////////////////////////////////////////////////
// Write-back multiplexer
//////////////////////////////////////////////////////////////
wire [31:0] writeback_data;

assign writeback_data = mem_to_reg_in ? load_data : alu_result_in;

//////////////////////////////////////////////////////////////
// Register file write signals
//////////////////////////////////////////////////////////////
assign reg_write_en = reg_write_in && valid_in && (rd_addr_in != 5'd0);
assign rd_addr      = rd_addr_in;
assign rd_data      = writeback_data;

// Forwarding data
assign forward_data = writeback_data;

endmodule
