`include "ctrl_encode_def.v"

module alu(A, B, ALUOp, C, Zero);
    
    input  signed [31:0] A, B;
    input         [3:0]  ALUOp;
    output signed [31:0] C;
    output Zero;
    
    reg [31:0] C;
    reg [31:0] L;
    integer    i;
    
    always @( * ) begin
		L = {{25{1'b0}}, A[4:0]};
		case ( ALUOp )
            `ALU_NOP:   C = A;                          // NOP
            `ALU_ADD:   C = A + B;                      // ADD
            `ALU_SUB:   C = A - B;                      // SUB
            `ALU_AND:   C = A & B;                      // AND/ANDI
            `ALU_OR:    C = A | B;                      // OR/ORI
            `ALU_SLT:   C = (A < B) ? 32'd1 : 32'd0;    // SLT/SLTI
            `ALU_SLTU:  C = ({1'b0, A} < {1'b0, B}) ? 32'd1 : 32'd0;
            `ALU_NOR:   C = ~ (A | B);                  // NOR
            `ALU_SLL:   C = B << A;                     // SLL
            `ALU_SRL:   C = B >> A;                     // SRL
            `ALU_SLLV:  C = B << L;                     // SLLV
            `ALU_SRLV:  C = B >> L; 					// SRLV
            `ALU_LUI:   C = B << 16;                    // LUI
            `ALU_XOR:   C = A ^ B;                      // XOR
            `ALU_SRAV:  begin                           // SRAV/SRA
                            C = B >> L;
                            for (i = 31; i > 31 - L; i = i-1)
                                C[i] = B[31];
                        end
            default:    C = A;                          // Undefined
        endcase
    end // end always
    
    assign Zero = (C == 32'b0);

endmodule