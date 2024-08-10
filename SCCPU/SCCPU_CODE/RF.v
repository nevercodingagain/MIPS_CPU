`include "ctrl_encode_def.v"

module RF(  input         clk, 
            input         rst,
            input         RFWr,         //使能写信号
            input  [4:0]  A1, A2, A3,   //读寄存器序号A1, A2, 写寄存器序号A3
            input  [31:0] WD,           //写入的数据
            input  [2:0]  WRITEwhb,     //write word | byte | half byte
            input  [31:0] memaddr,      //load WD form mem[memaddr]
            output [31:0] RD1, RD2,     //读出的数据, rs, rt
            input  [4:0]  reg_sel,      //register selection (for debug use)
            output [31:0] reg_data);    //selected register data (for debug use)
    
    reg [31:0] rf[31:0];
    reg [31:0] WD_SEL;
    integer i;

    initial 
        WD_SEL = 0;
    
    always @(posedge clk, posedge rst)
        if (rst) begin    //  reset
        for (i=1; i<32; i=i+1)
            rf[i] = 0;
        end
        
        else 
        if (RFWr) begin
            case( WRITEwhb )
                `RF_LW: WD_SEL = WD;
                `RF_LH: WD_SEL = {{16{WD[16*memaddr[1]+15]}}, WD[16*memaddr[1]+15 -: 16]};
                `RF_LHU:WD_SEL = {{16{1'b0}}, WD[16*memaddr[1]+15 -: 16]};
                `RF_LB: WD_SEL = {{24{WD[8*memaddr[1:0]+7]}}, WD[8*memaddr[1:0]+7 -: 8]};
                `RF_LBU:WD_SEL = {{24{1'b0}}, WD[8*memaddr[1:0]+7 -: 8]};
                default: WD_SEL = WD;
            endcase
            rf[A3] <= WD_SEL;
        end

    assign RD1 = (A1 != 0) ? rf[A1] : 0;
    assign RD2 = (A2 != 0) ? rf[A2] : 0;
    assign reg_data = (reg_sel != 0) ? rf[reg_sel] : 0;

endmodule
