
// IorD;     // 0-memory to IR, 1-memory to MDR
// ALUSrcA;  // 0-PC, 1-Aout, 2-shamt, 3-32'b0, ALU source for A 
// ALUSrcB;  // 0-Bout, 1-4, 2-Imm32, 3-branch offset, ALU source for B

// ALU control signal
`define ALU_NOP  4'b0000 
`define ALU_ADD  4'b0001
`define ALU_SUB  4'b0010 
`define ALU_AND  4'b0011
`define ALU_OR   4'b0100
`define ALU_SLT  4'b0101
`define ALU_SLTU 4'b0110
`define ALU_NOR  4'b0111
`define ALU_SLL  4'b1000
`define ALU_SRL  4'b1001
`define ALU_SLLV 4'b1010
`define ALU_SRLV 4'b1011
`define ALU_LUI  4'b1100
`define ALU_XOR  4'b1101
`define ALU_SRAV 4'b1110

// RF WRITEwhb
`define RF_LW   3'b000
`define RF_LH   3'b001
`define RF_LHU  3'b010
`define RF_LB   3'b011
`define RF_LBU  3'b100

// RF STOREwhb
`define STOREwhb_SW  2'b00
`define STOREwhb_SH  2'b01
`define STOREwhb_SB  2'b10

// PCSource; // next pc, 0-ALU(PC+4), 1-ALUOut, 2-JUMP address, 3-register
// GPRSel_RD    2'b00
// GPRSel_RT    2'b01
// GPRSel_31    2'b10

// WDSel_FromALU 2'b00
// WDSel_FromMEM 2'b01
// WDSel_FromPC  2'b10 