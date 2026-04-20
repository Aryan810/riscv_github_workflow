//  OPCODE and parameter definitions

`define OPCODE      6:0
`define FUNC3       14:12
`define SUBTYPE     30
`define RD          11:7
`define RS1         19:15
`define RS2         24:20
`define FUNC7       31:25


localparam [ 6: 0] FUNC7_M = 7'b0000001;


localparam  [31: 0] NOP        = 32'h0000_0013;     // addi x0, x0, 0

// OPCODE, INST[6:0]
localparam  [ 6: 0] LUI     = 7'b0110111,        // U-type
                    AUIPC   = 7'b0010111,        // U-type
                    JAL     = 7'b1101111,        // J-type
                    JALR    = 7'b1100111,        // I-type
                    BRANCH  = 7'b1100011,        // B-type
                    LOAD    = 7'b0000011,        // I-type
                    STORE   = 7'b0100011,        // S-type
                    ARITHI  = 7'b0010011,        // I-type
                    ARITHR  = 7'b0110011;        // R-type



// FUNC3, INST[14:12], INST[6:0] = 7'b1100011
localparam  [ 2: 0] BEQ     = 3'b000,
                    BNE     = 3'b001,
                    BLT     = 3'b100,
                    BGE     = 3'b101,
                    BLTU    = 3'b110,
                    BGEU    = 3'b111;

// FUNC3, INST[14:12], INST[6:0] = 7'b0000011
localparam  [ 2: 0] LB      = 3'b000,
                    LH      = 3'b001,
                    LW      = 3'b010,
                    LBU     = 3'b100,
                    LHU     = 3'b101;

// FUNC3, INST[14:12], INST[6:0] = 7'b0100011
localparam  [ 2: 0] SB      = 3'b000,
                    SH      = 3'b001,
                    SW      = 3'b010;
                    
// FUNC3, INST[14:12], INST[6:0] = 7'b0110011, 7'b0010011
localparam  [ 2: 0] ADD     = 3'b000,    // inst[30] == 0: ADD, inst[31] == 1: SUB
                    SLL     = 3'b001,
                    SLT     = 3'b010,
                    SLTU    = 3'b011,
                    XOR     = 3'b100,
                    SR      = 3'b101,    // inst[30] == 0: SRL, inst[31] == 1: SRA
                    OR      = 3'b110,
                    AND     = 3'b111;

// RV32M FUNC3, OPCODE = 7'b0110011, FUNC7 = 7'b0000001
localparam  [ 2: 0] MUL     = 3'b000, // multiple values of two registers, return lower 32 bits to dest reg
                    MULH    = 3'b001, // multiple values of two registers, return upper 32 bits to dest reg
                    MULHSU  = 3'b010, // multiple values of two registers, one is signed and the other is unsigned, return upper 32 bits to dest reg
                    MULHU   = 3'b011, // multiple values of two registers, both unsigned, return upper 32 bits to dest reg.
                    DIV     = 3'b100, // divide value of one register by the other, return quotient to dest reg.
                    DIVU    = 3'b101, // divide value of one register by the other, return quotient to dest reg, both values are unsigned.
                    REM     = 3'b110, // divide value of one register by the other, return remainder to dest reg.
                    REMU    = 3'b111; // divide value of one register by the other, return remainder to dest reg, both values are unsigned.

// MACC Accelerator (Custom-0)
localparam  [ 6: 0] CUSTOM_0 = 7'b0001011;
// 8-bit MACC (funct7 = 0000000)
localparam  [ 2: 0] MACC_RST   = 3'b000,
                    MACC_LOADA = 3'b001,
                    MACC_LOADB = 3'b010,
                    MACC_CALC  = 3'b011,
                    MACC_GET   = 3'b100;
// 32-bit MACC (funct7 = 0000001)
localparam  [ 2: 0] MACC32_RST   = 3'b000,
                    MACC32_LOADA = 3'b001,
                    MACC32_LOADB = 3'b010,
                    MACC32_CALC  = 3'b011,
                    MACC32_GET   = 3'b100;

