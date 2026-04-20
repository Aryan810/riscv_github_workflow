`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////
// FPGA Top Module: CPU-driven operation engine
// - sw[15:0] are switch inputs
// - BTNC executes matrix multiplication
// - RxD receives UART DMA
//////////////////////////////////////////////////////////////
module top_fpga (
    input  wire        clk,      // 100 MHz board clock
    input  wire        reset,    // Active-low reset
    input  wire [15:0] sw,       // sw[15:0]=switches
    input  wire        btnl,     // unused
    input  wire        btnc,     // execute
    input  wire        btnr,     // unused
    input  wire        btnu,     // unused
    input  wire        btnd,     // unused
    input  wire        RxD,      // UART RX
    output wire [15:0] led,
    output reg  [7:0]  an,       // Active-low anodes
    output reg  [6:0]  seg,      // Active-low segments a..g
    output wire        dp        // Active-low decimal point
);

// -----------------------------------------------------------
// UART RX Module
// -----------------------------------------------------------
wire [7:0] rx_data;
wire       rx_valid;

uart_rx u_uart (
    .clk(clk),
    .rst_n(reset),
    .rx(RxD),
    .rx_data(rx_data),
    .rx_valid(rx_valid)
);

// -----------------------------------------------------------
// UART DMA State Machine
// Loads bytes into dmem starting at addr 0
// -----------------------------------------------------------
reg        dmem_we_ext;
reg [9:0]  dmem_addr_ext;
reg [31:0] dmem_wdata_ext;

reg [7:0]  byte_cnt;
reg [31:0] word_buffer;
reg [9:0]  word_addr;

always @(posedge clk or negedge reset) begin
    if (!reset) begin
        dmem_we_ext <= 1'b0;
        dmem_addr_ext <= 10'd0;
        dmem_wdata_ext <= 32'd0;
        byte_cnt <= 0;
        word_buffer <= 32'd0;
        word_addr <= 10'd0;
    end else begin
        dmem_we_ext <= 1'b0;
        
        if (rx_valid) begin
            word_buffer <= {rx_data, word_buffer[31:8]}; // Little endian packing
            byte_cnt <= byte_cnt + 1;
            
            if (byte_cnt == 3) begin
                byte_cnt <= 0;
                dmem_we_ext <= 1'b1;
                dmem_addr_ext <= word_addr;
                dmem_wdata_ext <= {rx_data, word_buffer[31:8]};
                
                if (word_addr == 10'd32) begin
                    word_addr <= 10'd0;
                end else begin
                    word_addr <= word_addr + 1'b1;
                end
            end
        end
    end
end

// -----------------------------------------------------------
// CPU instance
// -----------------------------------------------------------
wire        exception;
wire [31:0] pc_display;
wire [31:0] wb_data_display;

wire [9:0]  io_addr;
wire        io_we;
wire [31:0] io_wdata;
reg  [31:0] io_rdata;

pipe #(.RESET(32'h0000_0000)) u_pipe (
    .clk            (clk),
    .reset          (reset),
    .stall          (1'b0),
    .dmem_we_ext    (dmem_we_ext),
    .dmem_addr_ext  (dmem_addr_ext),
    .dmem_wdata_ext (dmem_wdata_ext),
    .io_rdata       (io_rdata),
    .io_addr        (io_addr),
    .io_we          (io_we),
    .io_wdata       (io_wdata),
    .exception      (exception),
    .pc_out         (pc_display),
    .wb_data_out    (wb_data_display)
);

// -----------------------------------------------------------
// Button pulse generation
// -----------------------------------------------------------
wire btnc_rise;

button_step_fsm #(.DB_CYCLES(1_000_000)) u_btnc_step (
    .clk      (clk),
    .reset_n  (reset),
    .btn_in   (btnc),
    .step_pulse(btnc_rise)
);

// -----------------------------------------------------------
// MMIO Logic
// -----------------------------------------------------------
reg btnc_flag;
reg [31:0] display_value;

always @(posedge clk or negedge reset) begin
    if (!reset) begin
        btnc_flag <= 1'b0;
        display_value <= 32'd0;
    end else begin
        // Hardware sets flag on button press
        if (btnc_rise) begin
            btnc_flag <= 1'b1;
        end
        // CPU can clear flag or write to display
        if (io_we) begin
            if (io_addr == 10'd1) begin
                btnc_flag <= io_wdata[0];
            end else if (io_addr == 10'd2) begin
                display_value <= io_wdata;
            end
        end
    end
end

always @(*) begin
    case (io_addr)
        10'd0: io_rdata = {16'd0, sw};
        10'd1: io_rdata = {31'd0, btnc_flag};
        10'd2: io_rdata = display_value;
        default: io_rdata = 32'd0;
    endcase
end

// LED indicators
assign led[15] = btnc_flag;
assign led[14:0] = 15'd0;
assign dp = 1'b1;

// -----------------------------------------------------------
// 7-Segment Display Multiplexing
// -----------------------------------------------------------
reg [31:0] abs_value;
reg [3:0] digit7, digit6, digit5, digit4, digit3, digit2, digit1, digit0;
reg       show_minus;
reg [2:0] scan_sel;
reg [16:0] scan_counter;
reg [3:0] cur_digit;
reg       cur_minus;
reg [6:0] cur_seg;

always @(*) begin
    if (display_value[31]) begin
        abs_value  = (~display_value) + 1'b1;
        show_minus = 1'b1;
    end else begin
        abs_value  = display_value;
        show_minus = 1'b0;
    end

    digit0 =  abs_value % 10;
    digit1 = (abs_value / 10) % 10;
    digit2 = (abs_value / 100) % 10;
    digit3 = (abs_value / 1000) % 10;
    digit4 = (abs_value / 10000) % 10;
    digit5 = (abs_value / 100000) % 10;
    digit6 = (abs_value / 1000000) % 10;
    digit7 = (abs_value / 10000000) % 10;
end

always @(posedge clk) begin
    if (!reset) begin
        scan_counter <= 17'd0;
        scan_sel     <= 3'd0;
    end else begin
        if (scan_counter == 17'd99999) begin
            scan_counter <= 17'd0;
            scan_sel     <= scan_sel + 3'd1;
        end else begin
            scan_counter <= scan_counter + 17'd1;
        end
    end
end

always @(*) begin
    an        = 8'b1111_1111;
    cur_digit = 4'd0;
    cur_minus = 1'b0;
    cur_seg   = 7'b111_1111;

    case (scan_sel)
        3'd0: begin an = 8'b1111_1110; cur_digit = digit0; end
        3'd1: begin an = 8'b1111_1101; cur_digit = digit1; end
        3'd2: begin an = 8'b1111_1011; cur_digit = digit2; end
        3'd3: begin an = 8'b1111_0111; cur_digit = digit3; end
        3'd4: begin an = 8'b1110_1111; cur_digit = digit4; end
        3'd5: begin an = 8'b1101_1111; cur_digit = digit5; end
        3'd6: begin an = 8'b1011_1111; cur_digit = digit6; end
        3'd7: begin
            an = 8'b0111_1111;
            if (show_minus)
                cur_minus = 1'b1;
            else
                cur_digit = digit7;
        end
    endcase

    if (cur_minus) begin
        cur_seg = 7'b111_1110; // '-'
    end else begin
        case (cur_digit)
            4'd0: cur_seg = 7'b100_0000;
            4'd1: cur_seg = 7'b111_1001;
            4'd2: cur_seg = 7'b010_0100;
            4'd3: cur_seg = 7'b011_0000;
            4'd4: cur_seg = 7'b001_1001;
            4'd5: cur_seg = 7'b001_0010;
            4'd6: cur_seg = 7'b000_0010;
            4'd7: cur_seg = 7'b111_1000;
            4'd8: cur_seg = 7'b000_0000;
            4'd9: cur_seg = 7'b001_0000;
            default: cur_seg = 7'b111_1111;
        endcase
    end

    seg = cur_seg;
end

endmodule

module button_step_fsm #(
    parameter integer DB_CYCLES = 1_000_000
)(
    input  wire clk,
    input  wire reset_n,
    input  wire btn_in,
    output reg  step_pulse
);
localparam [1:0] S_IDLE             = 2'd0;
localparam [1:0] S_DEBOUNCE_PRESS   = 2'd1;
localparam [1:0] S_PRESSED          = 2'd2;
localparam [1:0] S_DEBOUNCE_RELEASE = 2'd3;

reg [1:0] state;
reg [19:0] db_counter;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        state      <= S_IDLE;
        db_counter <= 20'd0;
        step_pulse <= 1'b0;
    end else begin
        step_pulse <= 1'b0;
        case (state)
            S_IDLE: begin
                db_counter <= 20'd0;
                if (btn_in) begin
                    state <= S_DEBOUNCE_PRESS;
                end
            end

            S_DEBOUNCE_PRESS: begin
                if (!btn_in) begin
                    state      <= S_IDLE;
                    db_counter <= 20'd0;
                end else if (db_counter == DB_CYCLES - 1) begin
                    state      <= S_PRESSED;
                    db_counter <= 20'd0;
                    step_pulse <= 1'b1;
                end else begin
                    db_counter <= db_counter + 1'b1;
                end
            end

            S_PRESSED: begin
                if (!btn_in) begin
                    state <= S_DEBOUNCE_RELEASE;
                end
            end

            S_DEBOUNCE_RELEASE: begin
                if (btn_in) begin
                    state      <= S_PRESSED;
                    db_counter <= 20'd0;
                end else if (db_counter == DB_CYCLES - 1) begin
                    state      <= S_IDLE;
                    db_counter <= 20'd0;
                end else begin
                    db_counter <= db_counter + 1'b1;
                end
            end

            default: begin
                state      <= S_IDLE;
                db_counter <= 20'd0;
            end
        endcase
    end
end

endmodule