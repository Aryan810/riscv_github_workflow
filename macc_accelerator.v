`timescale 1ns / 1ps

module macc_accelerator (
    input clk,
    input rst_n, // using active low or high? Wait, I should check the project's reset convention.
    input rst_accel, // from macc.rst
    input load_a,
    input load_b,
    input calc_start,
    input [31:0] rs1_data, // for loadA and loadB
    input [4:0] get_idx,   // rs2 field for macc.get
    
    output reg busy, // high for 10 cycles during calc
    output reg done, // high for 1 cycle when calc finishes
    output reg [31:0] get_data
);

    // Internal state
    reg [7:0] a_buf [0:3][0:3];
    reg [7:0] b_buf [0:3][0:3];
    
    reg [1:0] a_load_cnt;
    reg [1:0] b_load_cnt;
    
    // 4x4 PE Array wires
    wire signed [7:0] a_wire [0:3][0:4];
    wire signed [7:0] b_wire [0:4][0:3];
    wire signed [31:0] c_wire [0:3][0:3];
    
    // Generate PEs
    genvar i, j;
    generate
        for (i = 0; i < 4; i = i + 1) begin : row
            for (j = 0; j < 4; j = j + 1) begin : col
                pe pe_inst (
                    .clk(clk),
                    .rst(rst_n == 0 || rst_accel),
                    .en(busy), // PE is only active when calculating
                    .a_in(a_wire[i][j]),
                    .b_in(b_wire[i][j]),
                    .a_out(a_wire[i][j+1]),
                    .b_out(b_wire[i+1][j]),
                    .c_out(c_wire[i][j])
                );
            end
        end
    endgenerate
    
    // Busy logic and calculation control
    reg [3:0] calc_cnt;
    
    always @(posedge clk) begin
        if (!rst_n || rst_accel) begin
            busy <= 0;
            done <= 0;
            calc_cnt <= 0;
            a_load_cnt <= 0;
            b_load_cnt <= 0;
            // Clear buffers not strictly necessary, they will be overwritten
        end else begin
            if (calc_start && !busy && !done) begin
                busy <= 1;
                done <= 0;
                calc_cnt <= 1; // 1 to 10
            end else if (busy) begin
                if (calc_cnt == 10) begin
                    busy <= 0;
                    calc_cnt <= 0;
                    done <= 1;
                end else begin
                    calc_cnt <= calc_cnt + 1;
                end
            end else if (done) begin
                done <= 0;
            end
            
            if (load_a) begin
                a_buf[a_load_cnt][0] <= rs1_data[7:0];
                a_buf[a_load_cnt][1] <= rs1_data[15:8];
                a_buf[a_load_cnt][2] <= rs1_data[23:16];
                a_buf[a_load_cnt][3] <= rs1_data[31:24];
                a_load_cnt <= a_load_cnt + 1;
            end
            if (load_b) begin
                b_buf[0][b_load_cnt] <= rs1_data[7:0];
                b_buf[1][b_load_cnt] <= rs1_data[15:8];
                b_buf[2][b_load_cnt] <= rs1_data[23:16];
                b_buf[3][b_load_cnt] <= rs1_data[31:24];
                b_load_cnt <= b_load_cnt + 1;
            end
        end
    end

    // Data feeding logic for Systolic Array
    // At cycle k (1 to 7), we feed a_buf[i][k-1-i] and b_buf[k-1-j][j]
    // If indices are out of bounds, feed 0.
    reg [7:0] a_feed [0:3];
    reg [7:0] b_feed [0:3];
    integer r, c;
    always @(*) begin
        for (r = 0; r < 4; r = r + 1) begin
            if (calc_cnt >= (r + 1) && calc_cnt <= (r + 4)) begin
                a_feed[r] = a_buf[r][calc_cnt - r - 1];
            end else begin
                a_feed[r] = 8'd0;
            end
        end
        for (c = 0; c < 4; c = c + 1) begin
            if (calc_cnt >= (c + 1) && calc_cnt <= (c + 4)) begin
                b_feed[c] = b_buf[calc_cnt - c - 1][c];
            end else begin
                b_feed[c] = 8'd0;
            end
        end
    end

    // Connect feeds to the array edge
    assign a_wire[0][0] = a_feed[0];
    assign a_wire[1][0] = a_feed[1];
    assign a_wire[2][0] = a_feed[2];
    assign a_wire[3][0] = a_feed[3];
    
    assign b_wire[0][0] = b_feed[0];
    assign b_wire[0][1] = b_feed[1];
    assign b_wire[0][2] = b_feed[2];
    assign b_wire[0][3] = b_feed[3];

    // Readout logic (get_idx is 0-15)
    always @(*) begin
        case (get_idx)
            4'd0: get_data = c_wire[0][0];
            4'd1: get_data = c_wire[0][1];
            4'd2: get_data = c_wire[0][2];
            4'd3: get_data = c_wire[0][3];
            4'd4: get_data = c_wire[1][0];
            4'd5: get_data = c_wire[1][1];
            4'd6: get_data = c_wire[1][2];
            4'd7: get_data = c_wire[1][3];
            4'd8: get_data = c_wire[2][0];
            4'd9: get_data = c_wire[2][1];
            4'd10: get_data = c_wire[2][2];
            4'd11: get_data = c_wire[2][3];
            4'd12: get_data = c_wire[3][0];
            4'd13: get_data = c_wire[3][1];
            4'd14: get_data = c_wire[3][2];
            4'd15: get_data = c_wire[3][3];
            default: get_data = 32'd0;
        endcase
    end

endmodule
