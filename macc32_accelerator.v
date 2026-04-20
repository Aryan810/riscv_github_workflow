`timescale 1ns / 1ps

module macc32_accelerator (
    input clk,
    input rst_n,
    input rst_accel,
    input load_a,
    input load_b,
    input calc_start,
    input [31:0] rs1_data, // 32-bit data
    input [4:0]  rs2_addr, // index to load into (0-15) or get from
    
    output reg busy,
    output reg done,
    output reg [31:0] get_data
);

    // Internal state: 16 elements of A, 16 elements of B
    reg signed [31:0] a_buf [0:15];
    reg signed [31:0] b_buf [0:15];
    
    // 16 accumulators for C
    reg signed [31:0] c_buf [0:15];

    // 4x1 Vector Multiplier (4 multipliers)
    wire signed [31:0] mult_out_0;
    wire signed [31:0] mult_out_1;
    wire signed [31:0] mult_out_2;
    wire signed [31:0] mult_out_3;

    // Active broadcast A and column B values
    reg signed [31:0] a_broadcast;
    reg signed [31:0] b_vec_0;
    reg signed [31:0] b_vec_1;
    reg signed [31:0] b_vec_2;
    reg signed [31:0] b_vec_3;

    assign mult_out_0 = a_broadcast * b_vec_0;
    assign mult_out_1 = a_broadcast * b_vec_1;
    assign mult_out_2 = a_broadcast * b_vec_2;
    assign mult_out_3 = a_broadcast * b_vec_3;

    reg [4:0] calc_state; // 0 to 16

    integer i;

    always @(posedge clk) begin
        if (!rst_n || rst_accel) begin
            busy <= 0;
            done <= 0;
            calc_state <= 0;
            for (i = 0; i < 16; i = i + 1) begin
                c_buf[i] <= 32'd0;
                a_buf[i] <= 32'd0; // Optional, but clean
                b_buf[i] <= 32'd0;
            end
        end else begin
            if (calc_start && !busy && !done) begin
                busy <= 1;
                done <= 0;
                calc_state <= 0;
            end else if (busy) begin
                // State machine to walk through the 4x4 matrix multiplication
                // We compute one row of C at a time (4 elements).
                // To compute row r of C (C[r][0..3]):
                // C[r][c] += A[r][k] * B[k][c] for k=0..3
                
                // Let's do it slightly differently to maximize our 4x1 vector:
                // We have 4 multipliers. We can compute C[r][0], C[r][1], C[r][2], C[r][3] simultaneously.
                // In state 's' (0 to 15):
                // r = s / 4
                // k = s % 4
                // We broadcast A[r][k]. We multiply it by B[k][0], B[k][1], B[k][2], B[k][3].
                // We add the results to C[r][0], C[r][1], C[r][2], C[r][3].
                
                if (calc_state < 16) begin
                    // Assign inputs for THIS cycle (combinational read)
                    a_broadcast = a_buf[ (calc_state[3:2] * 4) + calc_state[1:0] ]; // A[r][k]
                    b_vec_0     = b_buf[ (calc_state[1:0] * 4) + 0 ]; // B[k][0]
                    b_vec_1     = b_buf[ (calc_state[1:0] * 4) + 1 ]; // B[k][1]
                    b_vec_2     = b_buf[ (calc_state[1:0] * 4) + 2 ]; // B[k][2]
                    b_vec_3     = b_buf[ (calc_state[1:0] * 4) + 3 ]; // B[k][3]

                    // Accumulate
                    c_buf[ (calc_state[3:2] * 4) + 0 ] <= c_buf[ (calc_state[3:2] * 4) + 0 ] + mult_out_0;
                    c_buf[ (calc_state[3:2] * 4) + 1 ] <= c_buf[ (calc_state[3:2] * 4) + 1 ] + mult_out_1;
                    c_buf[ (calc_state[3:2] * 4) + 2 ] <= c_buf[ (calc_state[3:2] * 4) + 2 ] + mult_out_2;
                    c_buf[ (calc_state[3:2] * 4) + 3 ] <= c_buf[ (calc_state[3:2] * 4) + 3 ] + mult_out_3;

                    calc_state <= calc_state + 1;
                end else begin
                    busy <= 0;
                    calc_state <= 0;
                    done <= 1;
                end
            end else if (done) begin
                done <= 0;
            end
            
            if (load_a) begin
                a_buf[rs2_addr[3:0]] <= rs1_data;
            end
            if (load_b) begin
                b_buf[rs2_addr[3:0]] <= rs1_data;
            end
        end
    end

    // Readout logic
    always @(*) begin
        get_data = c_buf[rs2_addr[3:0]];
    end

endmodule