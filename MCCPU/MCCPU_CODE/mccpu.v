module mccpu(clk, rst, readdata, instr, PC, MemWrite, writedata, adr, reg_sel, reg_data, STOREwhb);

    input         clk;       // clock
    input         rst;       // reset
    input  [31:0] readdata;  // data from data memory
    
    output [31:0] instr;     // instruction
    output [31:0] PC;        // PC address
    output        MemWrite;  // memory write
    output [31:0] writedata; // data to data memory
    output [31:0] adr;       // memory address
    
    input  [4:0]  reg_sel;   // register selection (for debug use)
    output [31:0] reg_data;  // selected register data (for debug use)
    output [1:0]  STOREwhb;  // select data for store to dmem

    wire        RegWrite;    // control signal to register write
    wire        PCWrite;     // control signal for PC write
    wire        IRWrite;     // control signal for IR write
    wire        EXTOp;       // control signal to signed extension
    wire [3:0]  ALUOp;       // ALU opertion
    wire [1:0]  PCSource;    // next PC operation
    wire        IorD;        // memory access for instruction or data
    
    wire [1:0]  WDSel;       // (register) write data selection
    wire [1:0]  GPRSel;      // general purpose register selection
    wire [2:0]  WRITEwhb;    // select data for write in rf

    wire [1:0]  ALUSrcA;     // ALU source for ALUA
    wire [1:0]  ALUSrcB;     // ALU source for ALUB
    wire        Zero;        // ALU ouput zero
    
    wire [31:0] aluresult;   // aluresult -> aluout
    wire [31:0] aluout;      // alu out
    
    wire [4:0]  rs;          // rs
    wire [4:0]  rt;          // rt
    wire [4:0]  rd;          // rd
    wire [5:0]  Op;          // opcode
    wire [5:0]  Funct;       // funct
    wire [15:0] Imm16;       // 16-bit immediate
    wire [31:0] Imm32;       // 32-bit immediate
    wire [25:0] IMM;         // 26-bit immediate (address)
    wire [4:0]  A3;          // register address for write
    wire [31:0] WD;          // register write data
    wire [31:0] RD1;         // register data specified by rs
    wire [31:0] RD2;         // register data specified by rt
    wire [31:0] ALUA;        // operator for ALU A
    wire [31:0] ALUB;        // operator for ALU B
    wire [31:0] shamt;       // shamt
    wire [31:0] Aout;        // register A output
    wire [31:0] Bout;        // register B output
    wire [31:0] IRout;       // register IR output
    wire [31:0] MDRout;      // register MDR output
    wire [31:0] NPC;         // next PC

    assign instr = IRout;
    assign Op = instr[31:26];  // instruction
    assign Funct = instr[5:0]; // funct
    assign rs = instr[25:21];  // rs
    assign rt = instr[20:16];  // rt
    assign rd = instr[15:11];  // rd
    assign Imm16 = instr[15:0];// 16-bit immediate
    assign IMM = instr[25:0];  // 26-bit immediate
    assign shamt = {27'b0, instr[10:6]}; //shamt

    assign writedata = Bout;
    
    // instantiation of control unit
    ctrl U_CTRL (
        .clk(clk), .rst(rst), .Zero(Zero),
        .Op(Op),.Funct(Funct),
        .PCWrite(PCWrite), .IorD(IorD),
        .MemWrite(MemWrite), .IRWrite(IRWrite), .RegWrite(RegWrite),
        .EXTOp(EXTOp), .ALUSrcA(ALUSrcA), .ALUSrcB(ALUSrcB), .ALUOp(ALUOp),
        .PCSource(PCSource),
        .GPRSel(GPRSel), .WDSel(WDSel), 
        .WRITEwhb(WRITEwhb), .STOREwhb(STOREwhb)
    );

    // instantiation of NPC
    mux4 #(32) U_MUX4_NPC (
        .d0(aluresult), .d1(aluout), .d2({PC[31:28], IMM, 2'b00}), .d3(RD1), 
        .s(PCSource), .y(NPC)
    );

    // instantiation of PC
    enreg #(32) U_REG_PC (
        .clk(clk), .rst(rst), .enwrite(PCWrite), .in(NPC), .out(PC)
    );

    mux2 #(32) U_MUX_ADR (
        .d0(PC), .d1(aluout), .s(IorD), .y(adr)
    );

    // reg for IR
    enreg #(32) U_REG_IR (
        .clk(clk), .rst(rst), .enwrite(IRWrite), .in(readdata), .out(IRout)
    );

    // reg for MDR
    register #(32) U_REG_MDR (
        .clk(clk), .rst(rst), .in(readdata), .out(MDRout)
    );

    // instantiation of register file
    RF U_RF (
        .clk(clk), .rst(rst), .RFWr(RegWrite), 
        .A1(rs), .A2(rt), .A3(A3), 
        .WD(WD), 
        .WRITEwhb(WRITEwhb), .memaddr(aluout),
        .RD1(RD1), .RD2(RD2),
        .reg_sel(reg_sel),
        .reg_data(reg_data)
    );
    
    // mux for register data to write
    mux4 #(5) U_MUX4_GPR_A3 (
        .d0(rd), .d1(rt), .d2(5'b11111), .d3(5'b00000), .s(GPRSel), .y(A3)
    );
    
    // mux for register address to write
    mux4 #(32) U_MUX4_GPR_WD (
        .d0(aluout), .d1(MDRout), .d2(PC), .d3(32'b0), .s(WDSel), .y(WD)
    );

    // ext for signed extension or zero extension
    EXT U_EXT (
        .Imm16(Imm16), .EXTOp(EXTOp), .Imm32(Imm32) 
    );

    // mux for ALU ALUA
    mux4 #(32) U_MUX_ALU_A (
        .d0(PC), .d1(Aout), .d2(shamt), .d3(32'b0), .s(ALUSrcA), .y(ALUA)
    );

    // mux for ALU ALUB
    mux4 #(32) U_MUX_ALU_B (
        .d0(Bout), .d1(4), .d2(Imm32), .d3({{14{IMM[15]}}, IMM[15:0], 2'b00}), .s(ALUSrcB), .y(ALUB)
    );
    
    // instantiation of alu
    alu U_ALU ( 
        .A(ALUA), .B(ALUB), .ALUOp(ALUOp), .C(aluresult), .Zero(Zero)
    );

    // reg for ALUA
    register #(32) U_REG_A (
        .clk(clk), .rst(rst), .in(RD1), .out(Aout)
    );

    // reg for ALUB
    register #(32) U_REG_B (
        .clk(clk), .rst(rst), .in(RD2), .out(Bout)
    );

    // reg for ALUout
    register #(32) U_REG_ALUout (
        .clk(clk), .rst(rst), .in(aluresult), .out(aluout)
    );

endmodule