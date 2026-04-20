`timescale 1ns / 1ps

module rv32b_alu (
    input [31:0] rs1_data,
    input [31:0] rs2_data, // for R-type rs2_data, for I-type immediate
    input [4:0]  shamt,    // rs2_addr or lower 5 bits of immediate
    input [2:0]  funct3,
    input [6:0]  funct7,
    input        is_rtype,
    input        is_itype,
    output reg [31:0] result,
    output reg valid
);

    wire [31:0] andn_res = rs1_data & ~rs2_data;
    wire [31:0] orn_res  = rs1_data | ~rs2_data;
    wire [31:0] xnor_res = rs1_data ^ ~rs2_data;

    wire [31:0] sh1add_res = (rs1_data << 1) + rs2_data;
    wire [31:0] sh2add_res = (rs1_data << 2) + rs2_data;
    wire [31:0] sh3add_res = (rs1_data << 3) + rs2_data;

    wire signed [31:0] rs1_signed = rs1_data;
    wire signed [31:0] rs2_signed = rs2_data;
    
    wire [31:0] max_res  = (rs1_signed > rs2_signed) ? rs1_data : rs2_data;
    wire [31:0] min_res  = (rs1_signed < rs2_signed) ? rs1_data : rs2_data;
    wire [31:0] maxu_res = (rs1_data > rs2_data) ? rs1_data : rs2_data;
    wire [31:0] minu_res = (rs1_data < rs2_data) ? rs1_data : rs2_data;

    wire [31:0] sext_b_res = {{24{rs1_data[7]}}, rs1_data[7:0]};
    wire [31:0] sext_h_res = {{16{rs1_data[15]}}, rs1_data[15:0]};

    wire [31:0] rol_res = (rs1_data << rs2_data[4:0]) | (rs1_data >> (32 - rs2_data[4:0]));
    wire [31:0] ror_res = (rs1_data >> rs2_data[4:0]) | (rs1_data << (32 - rs2_data[4:0]));
    wire [31:0] rori_res = (rs1_data >> shamt) | (rs1_data << (32 - shamt));

    wire [31:0] bclr_res = rs1_data & ~(32'h1 << rs2_data[4:0]);
    wire [31:0] bext_res = (rs1_data >> rs2_data[4:0]) & 32'h1;
    wire [31:0] binv_res = rs1_data ^ (32'h1 << rs2_data[4:0]);
    wire [31:0] bset_res = rs1_data | (32'h1 << rs2_data[4:0]);

    wire [31:0] bclri_res = rs1_data & ~(32'h1 << shamt);
    wire [31:0] bexti_res = (rs1_data >> shamt) & 32'h1;
    wire [31:0] binvi_res = rs1_data ^ (32'h1 << shamt);
    wire [31:0] bseti_res = rs1_data | (32'h1 << shamt);

    wire [31:0] rev8_res = {rs1_data[7:0], rs1_data[15:8], rs1_data[23:16], rs1_data[31:24]};
    
    // orc.b: bitwise OR-combine byte
    wire [31:0] orc_b_res = {
        {8{|rs1_data[31:24]}},
        {8{|rs1_data[23:16]}},
        {8{|rs1_data[15:8]}},
        {8{|rs1_data[7:0]}}
    };

    // CLZ, CTZ, CPOP logic
    integer i;
    reg [5:0] clz_count, ctz_count, cpop_count;
    reg clz_done, ctz_done;
    
    always @(*) begin
        clz_count = 0;
        clz_done = 0;
        for (i = 31; i >= 0; i = i - 1) begin
            if (!clz_done) begin
                if (rs1_data[i] == 1'b0) clz_count = clz_count + 1;
                else clz_done = 1;
            end
        end
        
        ctz_count = 0;
        ctz_done = 0;
        for (i = 0; i <= 31; i = i + 1) begin
            if (!ctz_done) begin
                if (rs1_data[i] == 1'b0) ctz_count = ctz_count + 1;
                else ctz_done = 1;
            end
        end
        
        cpop_count = 0;
        for (i = 0; i <= 31; i = i + 1) begin
            if (rs1_data[i] == 1'b1) cpop_count = cpop_count + 1;
        end
    end

    always @(*) begin
        result = 32'h0;
        valid = 1'b0;

        if (is_rtype) begin
            case (funct7)
                7'b0100000: begin // Logical with negate
                    case (funct3)
                        3'b111: begin result = andn_res; valid = 1'b1; end
                        3'b110: begin result = orn_res; valid = 1'b1; end
                        3'b100: begin result = xnor_res; valid = 1'b1; end
                    endcase
                end
                7'b0010000: begin // Shift with add
                    case (funct3)
                        3'b010: begin result = sh1add_res; valid = 1'b1; end
                        3'b100: begin result = sh2add_res; valid = 1'b1; end
                        3'b110: begin result = sh3add_res; valid = 1'b1; end
                    endcase
                end
                7'b0000101: begin // Min/Max
                    case (funct3)
                        3'b110: begin result = max_res; valid = 1'b1; end
                        3'b111: begin result = maxu_res; valid = 1'b1; end
                        3'b100: begin result = min_res; valid = 1'b1; end
                        3'b101: begin result = minu_res; valid = 1'b1; end
                    endcase
                end
                7'b0110000: begin // Rotate / sext
                    case (funct3)
                        3'b001: begin 
                            if (shamt == 5'b00100) begin result = sext_b_res; valid = 1'b1; end
                            else if (shamt == 5'b00101) begin result = sext_h_res; valid = 1'b1; end
                            else begin result = rol_res; valid = 1'b1; end
                        end
                        3'b101: begin result = ror_res; valid = 1'b1; end
                    endcase
                end
                7'b0100100: begin // Single bit
                    case (funct3)
                        3'b001: begin result = bclr_res; valid = 1'b1; end
                        3'b101: begin result = bext_res; valid = 1'b1; end
                    endcase
                end
                7'b0110100: begin // Single bit
                    case (funct3)
                        3'b001: begin result = binv_res; valid = 1'b1; end
                    endcase
                end
                7'b0010100: begin // Single bit
                    case (funct3)
                        3'b001: begin result = bset_res; valid = 1'b1; end
                    endcase
                end
                default: ; // To prevent latch inference
            endcase
        end else if (is_itype) begin
            case (funct7)
                7'b0110000: begin
                    case (funct3)
                        3'b001: begin
                            if (shamt == 5'b00000) begin result = {26'd0, clz_count}; valid = 1'b1; end
                            else if (shamt == 5'b00001) begin result = {26'd0, ctz_count}; valid = 1'b1; end
                            else if (shamt == 5'b00010) begin result = {26'd0, cpop_count}; valid = 1'b1; end
                        end
                        3'b101: begin result = rori_res; valid = 1'b1; end
                    endcase
                end
                7'b0100100: begin
                    case (funct3)
                        3'b001: begin result = bclri_res; valid = 1'b1; end
                        3'b101: begin result = bexti_res; valid = 1'b1; end
                    endcase
                end
                7'b0110100: begin
                    case (funct3)
                        3'b001: begin result = binvi_res; valid = 1'b1; end
                        3'b101: begin 
                            if (shamt == 5'b11000) begin result = rev8_res; valid = 1'b1; end 
                        end
                    endcase
                end
                7'b0010100: begin
                    case (funct3)
                        3'b001: begin result = bseti_res; valid = 1'b1; end
                        3'b101: begin 
                            if (shamt == 5'b00111) begin result = orc_b_res; valid = 1'b1; end
                        end
                    endcase
                end
                default: ;
            endcase
        end
    end

endmodule