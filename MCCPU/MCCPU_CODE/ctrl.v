module ctrl(clk, rst, Zero,
            Op, Funct,
            PCWrite, IorD,
            MemWrite, IRWrite, RegWrite,
            EXTOp, ALUSrcA, ALUSrcB, ALUOp,
            PCSource,
            GPRSel, WDSel, 
            WRITEwhb, STOREwhb
            );
    
    input        clk, rst, Zero;
    input  [5:0] Op;            // opcode
    input  [5:0] Funct;         // funct
    
    output reg       PCWrite;   // control signal for PCs write
    output reg       IorD;      // 0-memory access for instruction, 1-memory access for data
    output reg       MemWrite;  // control signal for memory write
    output reg       IRWrite;   // control signal for register IR write
    output reg       RegWrite;  // control signal for register write

    output reg       EXTOp;     // control signal to signed extension
    output reg [1:0] ALUSrcA;   // 0-PC, 1-Aout, 2-shamt, 3-32'b0, ALU source for A 
    output reg [1:0] ALUSrcB;   // 0-Bout, 1-4, 2-Imm32, 3-3-branch offset, ALU source for B 
    output reg [3:0] ALUOp;     // ALU opertion
    output reg [1:0] PCSource;  // next pc, 0-ALU(PC+4), 1-ALUOut, 2-JUMP address, 3-register

    output reg [1:0] GPRSel;    // general purpose register selection
    output reg [1:0] WDSel;     // (register) write data selection
    output     [2:0] WRITEwhb;  // select signal to load word | half word | byte
    output     [1:0] STOREwhb;  // select signal to store word | half word | byte

    parameter  [2:0]    sif  = 3'b000,                // IF  state
                        sid  = 3'b001,                // ID  state
                        sexe = 3'b010,                // EXE state
                        smem = 3'b011,                // MEM state
                        swb  = 3'b100;                // WB  state
    // r format
    wire rtype  = ~|Op;
    wire i_add  = rtype& Funct[5]&~Funct[4]&~Funct[3]&~Funct[2]&~Funct[1]&~Funct[0]; // add
    wire i_sub  = rtype& Funct[5]&~Funct[4]&~Funct[3]&~Funct[2]& Funct[1]&~Funct[0]; // sub
    wire i_and  = rtype& Funct[5]&~Funct[4]&~Funct[3]& Funct[2]&~Funct[1]&~Funct[0]; // and
    wire i_or   = rtype& Funct[5]&~Funct[4]&~Funct[3]& Funct[2]&~Funct[1]& Funct[0]; // or
    wire i_slt  = rtype& Funct[5]&~Funct[4]& Funct[3]&~Funct[2]& Funct[1]&~Funct[0]; // slt
    wire i_sltu = rtype& Funct[5]&~Funct[4]& Funct[3]&~Funct[2]& Funct[1]& Funct[0]; // sltu
    wire i_addu = rtype& Funct[5]&~Funct[4]&~Funct[3]&~Funct[2]&~Funct[1]& Funct[0]; // addu
    wire i_subu = rtype& Funct[5]&~Funct[4]&~Funct[3]&~Funct[2]& Funct[1]& Funct[0]; // subu

    // r extend
    wire i_sll  = rtype&~Funct[5]&~Funct[4]&~Funct[3]&~Funct[2]&~Funct[1]&~Funct[0]; // sll  000000
    wire i_nor  = rtype& Funct[5]&~Funct[4]&~Funct[3]& Funct[2]& Funct[1]& Funct[0]; // nor  100111
    wire i_srl  = rtype&~Funct[5]&~Funct[4]&~Funct[3]&~Funct[2]& Funct[1]&~Funct[0]; // srl  000010
    wire i_sllv = rtype&~Funct[5]&~Funct[4]&~Funct[3]& Funct[2]&~Funct[1]&~Funct[0]; // sllv 000100
    wire i_srlv = rtype&~Funct[5]&~Funct[4]&~Funct[3]& Funct[2]& Funct[1]&~Funct[0]; // srlv 000110
    wire i_jr   = rtype&~Funct[5]&~Funct[4]& Funct[3]&~Funct[2]&~Funct[1]&~Funct[0]; // jr   001000
    wire i_jalr = rtype&~Funct[5]&~Funct[4]& Funct[3]&~Funct[2]&~Funct[1]& Funct[0]; // jalr 001001
    wire i_xor  = rtype& Funct[5]&~Funct[4]&~Funct[3]& Funct[2]& Funct[1]&~Funct[0]; // xor  100110
    wire i_sra  = rtype&~Funct[5]&~Funct[4]&~Funct[3]&~Funct[2]& Funct[1]& Funct[0]; // sra  000011
    wire i_srav = rtype&~Funct[5]&~Funct[4]&~Funct[3]& Funct[2]& Funct[1]& Funct[0]; // srav 000111

    // i format
    wire i_addi = ~Op[5]&~Op[4]& Op[3]&~Op[2]&~Op[1]&~Op[0]; // addi
    wire i_ori  = ~Op[5]&~Op[4]& Op[3]& Op[2]&~Op[1]& Op[0]; // ori
    wire i_lw   =  Op[5]&~Op[4]&~Op[3]&~Op[2]& Op[1]& Op[0]; // lw
    wire i_sw   =  Op[5]&~Op[4]& Op[3]&~Op[2]& Op[1]& Op[0]; // sw
    wire i_beq  = ~Op[5]&~Op[4]&~Op[3]& Op[2]&~Op[1]&~Op[0]; // beq

    // i extened
    wire i_bne  = ~Op[5]&~Op[4]&~Op[3]& Op[2]&~Op[1]& Op[0]; // bne  000101
    wire i_slti = ~Op[5]&~Op[4]& Op[3]&~Op[2]& Op[1]&~Op[0]; // slti 001010
    wire i_lui  = ~Op[5]&~Op[4]& Op[3]& Op[2]& Op[1]& Op[0]; // lui  001111
    wire i_andi = ~Op[5]&~Op[4]& Op[3]& Op[2]&~Op[1]&~Op[0]; // andi 001100
    wire i_lb   =  Op[5]&~Op[4]&~Op[3]&~Op[2]&~Op[1]&~Op[0]; // lb   100000
    wire i_lbu  =  Op[5]&~Op[4]&~Op[3]& Op[2]&~Op[1]&~Op[0]; // lbu  100100
    wire i_lh   =  Op[5]&~Op[4]&~Op[3]&~Op[2]&~Op[1]& Op[0]; // lh   100001
    wire i_lhu  =  Op[5]&~Op[4]&~Op[3]& Op[2]&~Op[1]& Op[0]; // lhu  100101
    wire i_sb   =  Op[5]&~Op[4]& Op[3]&~Op[2]&~Op[1]&~Op[0]; // sb   101000
    wire i_sh   =  Op[5]&~Op[4]& Op[3]&~Op[2]&~Op[1]& Op[0]; // sh   101001
    
    // j format
    wire i_j    = ~Op[5]&~Op[4]&~Op[3]&~Op[2]& Op[1]&~Op[0];  // j
    
    // j extend
    wire i_jal  = ~Op[5]&~Op[4]&~Op[3]&~Op[2]& Op[1]& Op[0];  //jal 000011
    
    assign WRITEwhb[0] = i_lh | i_lb;
    assign WRITEwhb[1] = i_lhu | i_lb;
    assign WRITEwhb[2] = i_lbu;

    assign STOREwhb[0] = i_sh;
    assign STOREwhb[1] = i_sb;

    /*************************************************/
    /******               FSM                   ******/
    reg [2:0] state;
    reg [2:0] nextstate;

    always @(posedge clk or posedge rst)
    begin
        if ( rst )
            state <= sif;
        else
            state <= nextstate;
    end

    /*************************************************/
    /******         Control Signal              ******/
    always @(*) begin
        PCWrite = 0;
        IorD = 0;           // 0-memory access for instruction
        MemWrite = 0;
        IRWrite = 0;
        RegWrite = 0;
        EXTOp = 1;
        ALUSrcA = 2'b01;    // 1-RD1
        ALUSrcB = 2'b00;    // 0-RD2
        ALUOp = 4'b0001;    // ALU_ADD
        PCSource = 2'b00;   // 0-ALU(PC+4)
        GPRSel = 0;         // 0-RD
        WDSel = 0;          // 0-ALU
    
        case(state)
            sif: begin             // Instruction Fecth
                nextstate = sid;
                ALUSrcA = 2'b00;    // PC
                ALUSrcB = 2'b01;    // 4
                PCWrite = 1;
                IRWrite = 1;
            end

            sid: begin             // Instruction Decode
            if (i_j) begin
                nextstate = sif;
                PCSource = 2'b10;   // 2-JUMP address
                PCWrite = 1;
            end else if (i_jr) begin
                nextstate = sif;
                PCSource = 2'b11;   // 3-register
                PCWrite = 1;
            end else if (i_jal) begin
                nextstate = sif;
                PCSource = 2'b10;   // 2-JUMP address
                PCWrite = 1;
                GPRSel = 2'b10;     // $31
                WDSel = 2'b10;      // PC
                RegWrite = 1;
            end else if (i_jalr) begin
                nextstate = sif;
                PCSource = 2'b11;   // 3-register
                PCWrite = 1;
                GPRSel = 2'b00;     // RD
                WDSel = 2'b10;      // PC
                RegWrite = 1;
            end else begin
                nextstate = sexe;
                ALUSrcA = 2'b00;    // PC
                ALUSrcB = 2'b11;    // branch offset
            end
            end

            sexe: begin            // Instruction Execute
            ALUOp[0] = i_add | i_lw | i_sw | i_addi | i_and | i_slt | i_addu | i_nor | i_srl | i_srlv | i_slti | i_andi | i_xor | i_lb | i_lbu | i_lh | i_lhu | i_sb | i_sh;
            ALUOp[1] = i_sub | i_beq | i_and | i_sltu | i_subu | i_nor | i_sllv | i_srlv | i_bne | i_andi | i_srav | i_sra;
            ALUOp[2] = i_or | i_ori | i_slt | i_sltu | i_nor | i_lui | i_slti | i_xor | i_srav | i_sra;
            ALUOp[3] = i_sll | i_srl | i_sllv | i_srlv | i_lui | i_xor | i_srav | i_sra;
            if (i_beq | i_bne) begin
                nextstate = sif;
                PCSource = 2'b01;   // branch address
                PCWrite = (i_beq & Zero) | (i_bne & ~Zero);
            end else if (i_lw | i_lh | i_lhu | i_lb | i_lbu | i_sw | i_sh | i_sb) begin
                nextstate = smem;
                ALUSrcB = 2'b10;    // select offset imm
            end else if (~rtype) begin   // i format
                nextstate = swb;
                ALUSrcB = 2'b10;    // select immediate
                if (i_andi) begin
                    EXTOp = 0;
                end else if (i_ori) begin
                    EXTOp = 0;      // zero extension
                end
            end else if (i_sll | i_srl | i_sra) begin
                nextstate = swb;
                ALUSrcA = 2'b10;    // 2-shamt
            end else begin          // r format
                nextstate = swb;
            end
            end

            smem: begin
            IorD = 1;               // 1-memory access for data
            if (i_lw | i_lh | i_lhu | i_lb | i_lbu)
                nextstate = swb;
            else begin
                nextstate = sif;
                MemWrite = 1;
            end
            end

            swb: begin               // Write Back
            nextstate = sif;
            if (i_lw | i_lh | i_lhu | i_lb | i_lbu)
                WDSel = 2'b01;      // WDSel_FromMEM
            if (~rtype)
                GPRSel = 2'b01;     // RT
            RegWrite = 1;
            end

            default:
            nextstate = sif;
        endcase
    end
endmodule