`timescale 1ns/1ps
//////////////////////////////////////////////////////////////
// Multicycle Multiply/Divide Unit for RV32M
// 
// Multiply: 4 cycles (pipelined style)
// Divide:   34 cycles (iterative non-restoring)
//////////////////////////////////////////////////////////////
module mul_div (
    input               clk,
    input               reset,
    
    // Control
    input               start,          // Start operation
    input               is_mul,         // 1 = multiply, 0 = divide
    input  [2:0]        funct3,         // Operation subtype
    input  [31:0]       operand_a,      // rs1
    input  [31:0]       operand_b,      // rs2
    
    // Outputs
    output reg [31:0]   result,         // Result
    output reg          busy,           // Operation in progress
    output reg          done            // Result ready (1 cycle pulse)
);

`include "opcode.vh"

//////////////////////////////////////////////////////////////
// State Machine
//////////////////////////////////////////////////////////////
localparam IDLE     = 2'b00;
localparam MUL_BUSY = 2'b01;
localparam DIV_BUSY = 2'b10;
localparam FINISH   = 2'b11;

reg [1:0] state, next_state;
reg [5:0] cycle_count;

//////////////////////////////////////////////////////////////
// Latched Operands (captured on start)
//////////////////////////////////////////////////////////////
reg [31:0] latched_a;
reg [31:0] latched_b;
reg [2:0]  latched_funct3;
reg        latched_is_mul;

//////////////////////////////////////////////////////////////
// Operand Sign Handling (using latched values)
//////////////////////////////////////////////////////////////
wire a_signed = (latched_funct3 == MUL) || (latched_funct3 == MULH) || (latched_funct3 == MULHSU) || 
                (latched_funct3 == DIV) || (latched_funct3 == REM);
wire b_signed = (latched_funct3 == MUL) || (latched_funct3 == MULH) || 
                (latched_funct3 == DIV) || (latched_funct3 == REM);

wire a_neg = a_signed && latched_a[31];
wire b_neg = b_signed && latched_b[31];

// Absolute values for computation
wire [31:0] abs_a = a_neg ? (~latched_a + 1) : latched_a;
wire [31:0] abs_b = b_neg ? (~latched_b + 1) : latched_b;

//////////////////////////////////////////////////////////////
// Multiply Logic (4 cycles using 16-bit partial products)
//////////////////////////////////////////////////////////////
reg [63:0] mul_result;
reg [63:0] partial_prod [0:3];
reg [1:0]  mul_cycle;

// Partial products (computed combinationally but accumulated over cycles)
wire [31:0] pp0 = abs_a[15:0]  * abs_b[15:0];   // AL * BL
wire [31:0] pp1 = abs_a[31:16] * abs_b[15:0];   // AH * BL  
wire [31:0] pp2 = abs_a[15:0]  * abs_b[31:16];  // AL * BH
wire [31:0] pp3 = abs_a[31:16] * abs_b[31:16];  // AH * BH

//////////////////////////////////////////////////////////////
// Divide Logic (Restoring division, 32 cycles)
//////////////////////////////////////////////////////////////
reg [31:0] quotient;
reg [31:0] remainder;
reg [5:0]  div_cycle;
reg        div_by_zero;

//////////////////////////////////////////////////////////////
// Result Sign Correction
//////////////////////////////////////////////////////////////
wire mul_result_neg = a_neg ^ b_neg;
wire div_result_neg = a_neg ^ b_neg;
wire rem_result_neg = a_neg;  // Remainder has sign of dividend

//////////////////////////////////////////////////////////////
// State Machine
//////////////////////////////////////////////////////////////
always @(posedge clk or negedge reset) begin
    if (!reset) begin
        state <= IDLE;
        latched_a <= 32'd0;
        latched_b <= 32'd0;
        latched_funct3 <= 3'd0;
        latched_is_mul <= 1'b0;
    end else begin
        state <= next_state;
        
        // Latch operands on start
        if (state == IDLE && start) begin
            latched_a <= operand_a;
            latched_b <= operand_b;
            latched_funct3 <= funct3;
            latched_is_mul <= is_mul;
        end
    end
end

always @(*) begin
    next_state = state;
    
    case (state)
        IDLE: begin
            if (start) begin
                next_state = is_mul ? MUL_BUSY : DIV_BUSY;
            end
        end
        
        MUL_BUSY: begin
            if (mul_cycle == 2'd3) begin
                next_state = FINISH;
            end
        end
        
        DIV_BUSY: begin
            if (div_cycle == 6'd33 || div_by_zero) begin
                next_state = FINISH;
            end
        end
        
        FINISH: begin
            next_state = IDLE;
        end
    endcase
end

//////////////////////////////////////////////////////////////
// Multiply Operation
//////////////////////////////////////////////////////////////
always @(posedge clk or negedge reset) begin
    if (!reset) begin
        mul_cycle <= 2'd0;
        mul_result <= 64'd0;
    end else if (state == IDLE && start && is_mul) begin
        // Initialize
        mul_cycle <= 2'd0;
        mul_result <= 64'd0;
    end else if (state == MUL_BUSY) begin
        mul_cycle <= mul_cycle + 1;
        
        case (mul_cycle)
            2'd0: mul_result <= {32'd0, pp0};
            2'd1: mul_result <= mul_result + ({16'd0, pp1, 16'd0});
            2'd2: mul_result <= mul_result + ({16'd0, pp2, 16'd0});
            2'd3: mul_result <= mul_result + ({pp3, 32'd0});
        endcase
    end
end

//////////////////////////////////////////////////////////////
// Divide Operation (Restoring division algorithm)
//////////////////////////////////////////////////////////////
reg [63:0] div_accumulator;  // Upper 32 bits = partial remainder, lower 32 = quotient
reg [31:0] div_divisor;
wire [63:0] div_shifted;
wire [32:0] div_diff;

assign div_shifted = {div_accumulator[62:0], 1'b0};
assign div_diff = {1'b0, div_shifted[63:32]} - {1'b0, div_divisor};

always @(posedge clk or negedge reset) begin
    if (!reset) begin
        div_cycle <= 6'd0;
        div_accumulator <= 64'd0;
        div_divisor <= 32'd0;
        quotient <= 32'd0;
        remainder <= 32'd0;
        div_by_zero <= 1'b0;
    end else if (state == IDLE && start && !is_mul) begin
        // Initialize division - use input operand values directly since we're starting
        // Compute absolute values inline using input funct3 (not latched yet)
        div_cycle <= 6'd0;
        div_accumulator <= {32'd0, ((funct3 == DIV || funct3 == REM) && operand_a[31]) ? (~operand_a + 32'd1) : operand_a};
        div_divisor <= ((funct3 == DIV || funct3 == REM) && operand_b[31]) ? (~operand_b + 32'd1) : operand_b;
        quotient <= 32'd0;
        remainder <= 32'd0;
        div_by_zero <= (operand_b == 32'd0);
    end else if (state == DIV_BUSY && !div_by_zero) begin
        div_cycle <= div_cycle + 1;
        
        if (div_cycle < 6'd32) begin
            if (!div_diff[32]) begin
                // Subtraction successful (remainder >= divisor)
                div_accumulator <= {div_diff[31:0], div_shifted[31:1], 1'b1};
            end else begin
                // Subtraction failed, restore
                div_accumulator <= div_shifted;
            end
        end else if (div_cycle == 6'd32) begin
            // Final result
            quotient <= div_accumulator[31:0];
            remainder <= div_accumulator[63:32];
        end
    end
end

//////////////////////////////////////////////////////////////
// Output Logic
//////////////////////////////////////////////////////////////
reg [63:0] final_mul_result;
reg [31:0] final_quotient;
reg [31:0] final_remainder;

always @(*) begin
    // Sign correction for multiply
    if (mul_result_neg && (latched_funct3 != MULHU))
        final_mul_result = ~mul_result + 1;
    else
        final_mul_result = mul_result;
    
    // Sign correction for divide
    if (div_result_neg)
        final_quotient = ~quotient + 1;
    else
        final_quotient = quotient;
    
    if (rem_result_neg)
        final_remainder = ~remainder + 1;
    else
        final_remainder = remainder;
end

always @(posedge clk or negedge reset) begin
    if (!reset) begin
        result <= 32'd0;
        busy <= 1'b0;
        done <= 1'b0;
    end else begin
        done <= 1'b0;  // Default: done is a pulse
        
        case (state)
            IDLE: begin
                if (start) begin
                    busy <= 1'b1;
                end
            end
            
            FINISH: begin
                busy <= 1'b0;
                done <= 1'b1;
                
                // Select result based on operation (use latched values)
                case (latched_funct3)
                    MUL:     result <= final_mul_result[31:0];
                    MULH:    result <= final_mul_result[63:32];
                    MULHSU:  result <= final_mul_result[63:32];
                    MULHU:   result <= mul_result[63:32];  // Unsigned, no correction
                    DIV:     result <= div_by_zero ? 32'hFFFFFFFF : final_quotient;
                    DIVU:    result <= div_by_zero ? 32'hFFFFFFFF : quotient;
                    REM:     result <= div_by_zero ? latched_a : final_remainder;
                    REMU:    result <= div_by_zero ? latched_a : remainder;
                    default: result <= 32'd0;
                endcase
            end
            
            default: begin
                // Stay busy during MUL_BUSY and DIV_BUSY
            end
        endcase
    end
end

endmodule
