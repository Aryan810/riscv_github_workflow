module pe (
    input clk,
    input rst,
    input en,
    input signed [7:0] a_in,
    input signed [7:0] b_in,
    output reg signed [7:0] a_out,
    output reg signed [7:0] b_out,
    output reg signed [31:0] c_out
);

    always @(posedge clk) begin
        if (rst) begin
            a_out <= 8'd0;
            b_out <= 8'd0;
            c_out <= 32'd0;
        end else if (en) begin
            a_out <= a_in;
            b_out <= b_in;
            c_out <= c_out + (a_in * b_in);
        end
    end

endmodule
