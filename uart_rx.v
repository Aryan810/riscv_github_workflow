`timescale 1ns / 1ps

module uart_rx #(
    parameter CLKS_PER_BIT = 868 // 100 MHz / 115200 baud = ~868
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,
    output reg  [7:0] rx_data,
    output reg        rx_valid
);

    localparam s_IDLE         = 3'b000;
    localparam s_RX_START_BIT = 3'b001;
    localparam s_RX_DATA_BITS = 3'b010;
    localparam s_RX_STOP_BIT  = 3'b011;
    localparam s_CLEANUP      = 3'b100;
    
    reg [2:0] r_SM_Main;
    reg [9:0] r_Clock_Count;
    reg [2:0] r_Bit_Index;
    reg       r_Rx_Data_R;
    reg       r_Rx_Data;
    
    // Double-register the incoming data to prevent metastability
    always @(posedge clk) begin
        r_Rx_Data_R <= rx;
        r_Rx_Data   <= r_Rx_Data_R;
    end
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_SM_Main <= s_IDLE;
            r_Clock_Count <= 0;
            r_Bit_Index <= 0;
            rx_data <= 0;
            rx_valid <= 0;
        end else begin
            case (r_SM_Main)
                s_IDLE: begin
                    rx_valid <= 0;
                    r_Clock_Count <= 0;
                    r_Bit_Index <= 0;
                    if (r_Rx_Data == 1'b0) r_SM_Main <= s_RX_START_BIT;
                    else r_SM_Main <= s_IDLE;
                end
                
                s_RX_START_BIT: begin
                    if (r_Clock_Count == (CLKS_PER_BIT-1)/2) begin
                        if (r_Rx_Data == 1'b0) begin
                            r_Clock_Count <= 0;
                            r_SM_Main <= s_RX_DATA_BITS;
                        end else r_SM_Main <= s_IDLE;
                    end else begin
                        r_Clock_Count <= r_Clock_Count + 1;
                        r_SM_Main <= s_RX_START_BIT;
                    end
                end
                
                s_RX_DATA_BITS: begin
                    if (r_Clock_Count < CLKS_PER_BIT-1) begin
                        r_Clock_Count <= r_Clock_Count + 1;
                        r_SM_Main <= s_RX_DATA_BITS;
                    end else begin
                        r_Clock_Count <= 0;
                        rx_data[r_Bit_Index] <= r_Rx_Data;
                        if (r_Bit_Index < 7) begin
                            r_Bit_Index <= r_Bit_Index + 1;
                            r_SM_Main <= s_RX_DATA_BITS;
                        end else begin
                            r_Bit_Index <= 0;
                            r_SM_Main <= s_RX_STOP_BIT;
                        end
                    end
                end
                
                s_RX_STOP_BIT: begin
                    if (r_Clock_Count < CLKS_PER_BIT-1) begin
                        r_Clock_Count <= r_Clock_Count + 1;
                        r_SM_Main <= s_RX_STOP_BIT;
                    end else begin
                        rx_valid <= 1;
                        r_Clock_Count <= 0;
                        r_SM_Main <= s_CLEANUP;
                    end
                end
                
                s_CLEANUP: begin
                    r_SM_Main <= s_IDLE;
                    rx_valid <= 0;
                end
                
                default: r_SM_Main <= s_IDLE;
            endcase
        end
    end
endmodule
