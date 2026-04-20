`timescale 1ns/1ps
//////////////////////////////////////////////////////////////
// Stage 4: Memory (MEM)
// Includes integrated data memory
//////////////////////////////////////////////////////////////
module memory
#(
    parameter [31:0] RESET = 32'h0000_0000
)
(
    input               clk,
    input               reset,
    
    // Pipeline control
    input               stall,
    
    // From EX/MEM register
    input  [31:0]       pc_plus4_in,
    input  [31:0]       alu_result_in,
    input  [31:0]       rs2_data_in,
    input  [4:0]        rd_addr_in,
    input  [2:0]        funct3_in,
    
    // Control signals from EX/MEM
    input               mem_write_in,
    input               mem_read_in,
    input               mem_to_reg_in,
    input               reg_write_in,
    input               valid_in,

    // External write port to dmem (from FPGA UART DMA)
    input               dmem_we_ext,
    input  [9:0]        dmem_addr_ext,
    input  [31:0]       dmem_wdata_ext,
    
    // IO Interface for CPU
    input  [31:0]       io_rdata,
    output [9:0]        io_addr,
    output              io_we,
    output [31:0]       io_wdata,
    
    // Outputs to MEM/WB register
    output reg [31:0]   pc_plus4_out,
    output reg [31:0]   alu_result_out,
    output reg [31:0]   mem_read_data_out,
    output reg [4:0]    rd_addr_out,
    output reg [2:0]    funct3_out,
    
    // Control signals to MEM/WB
    output reg          mem_to_reg_out,
    output reg          reg_write_out,
    output reg          valid_out,
    
    // For forwarding (EX/MEM result)
    output [31:0]       forward_data
);

`include "opcode.vh"

//////////////////////////////////////////////////////////////
// Data Memory (Integrated)
//////////////////////////////////////////////////////////////
(* ram_style = "distributed" *)
reg [31:0] dmem [0:1023];  // 4KB data memory

initial begin
    $readmemh("dmem.hex", dmem);
end

// Memory address and index
wire [31:0] mem_addr = alu_result_in;
wire [9:0]  mem_index = mem_addr[11:2];
wire [1:0]  byte_offset = mem_addr[1:0];

//////////////////////////////////////////////////////////////
// Store data alignment
//////////////////////////////////////////////////////////////
reg [31:0] write_data;
reg [3:0]  write_strobe;

always @(*) begin
    write_data   = 32'h0;
    write_strobe = 4'b0000;
    
    if (mem_write_in && valid_in) begin
        case (funct3_in)
            SB: begin
                // Store byte
                write_data = {4{rs2_data_in[7:0]}};
                case (byte_offset)
                    2'b00: write_strobe = 4'b0001;
                    2'b01: write_strobe = 4'b0010;
                    2'b10: write_strobe = 4'b0100;
                    2'b11: write_strobe = 4'b1000;
                endcase
            end
            
            SH: begin
                // Store halfword
                write_data = {2{rs2_data_in[15:0]}};
                write_strobe = byte_offset[1] ? 4'b1100 : 4'b0011;
            end
            
            SW: begin
                // Store word
                write_data = rs2_data_in;
                write_strobe = 4'b1111;
            end
            
            default: begin
                write_data = 32'h0;
                write_strobe = 4'b0000;
            end
        endcase
    end
end

wire is_io = (alu_result_in[31:28] == 4'h4);
assign io_addr = alu_result_in[11:2];
assign io_we = is_io && mem_write_in && valid_in;
assign io_wdata = write_data;

// Synchronous write to data memory
always @(posedge clk) begin
    if (dmem_we_ext) begin
        dmem[dmem_addr_ext] <= dmem_wdata_ext;
    end else if (!is_io && mem_write_in && valid_in) begin
        if (write_strobe[0]) dmem[mem_index][7:0]   <= write_data[7:0];
        if (write_strobe[1]) dmem[mem_index][15:8]  <= write_data[15:8];
        if (write_strobe[2]) dmem[mem_index][23:16] <= write_data[23:16];
        if (write_strobe[3]) dmem[mem_index][31:24] <= write_data[31:24];
    end
end

// Combinational read from data memory
wire [31:0] mem_read_data = is_io ? io_rdata : dmem[mem_index];

//////////////////////////////////////////////////////////////
// Forwarding data (ALU result from this stage)
//////////////////////////////////////////////////////////////
assign forward_data = alu_result_in;

//////////////////////////////////////////////////////////////
// MEM/WB Pipeline Register
//////////////////////////////////////////////////////////////
always @(posedge clk or negedge reset) begin
    if (!reset) begin
        pc_plus4_out      <= 32'h0;
        alu_result_out    <= 32'h0;
        mem_read_data_out <= 32'h0;
        rd_addr_out       <= 5'h0;
        funct3_out        <= 3'h0;
        mem_to_reg_out    <= 1'b0;
        reg_write_out     <= 1'b0;
        valid_out         <= 1'b0;
    end
    else if (!stall) begin
        pc_plus4_out      <= pc_plus4_in;
        alu_result_out    <= alu_result_in;
        mem_read_data_out <= mem_read_data;
        rd_addr_out       <= rd_addr_in;
        funct3_out        <= funct3_in;
        mem_to_reg_out    <= mem_to_reg_in;
        reg_write_out     <= reg_write_in;
        valid_out         <= valid_in;
    end
end

endmodule
