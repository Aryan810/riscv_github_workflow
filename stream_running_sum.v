`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////
// Streaming running-sum unit
// - Accepts one signed sample at a time with in_valid
// - Produces updated sum and 1-cycle out_valid pulse per sample
//////////////////////////////////////////////////////////////
module stream_running_sum #(
    parameter IN_WIDTH  = 16,
    parameter SUM_WIDTH = 32
)(
    input  wire                          clk,
    input  wire                          reset_n,   // Active-low
    input  wire                          clear,     // Synchronous clear
    input  wire                          in_valid,
    input  wire signed [IN_WIDTH-1:0]    in_data,
    output wire                          in_ready,
    output reg                           out_valid,
    output reg  signed [SUM_WIDTH-1:0]   sum_out
);

assign in_ready = 1'b1;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        sum_out   <= {SUM_WIDTH{1'b0}};
        out_valid <= 1'b0;
    end else begin
        out_valid <= 1'b0;

        if (clear) begin
            sum_out <= {SUM_WIDTH{1'b0}};
        end else if (in_valid && in_ready) begin
            sum_out   <= sum_out + $signed(in_data);
            out_valid <= 1'b1;
        end
    end
end

endmodule

