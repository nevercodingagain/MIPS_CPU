`include "ctrl_encode_def.v"

// data memory
module mem(clk, MEMWr, address, din, STOREwhb, dout);
    input          clk;
    input          MEMWr;
    input  [8:0]   address;     // byte address
    input  [31:0]  din;
    input  [1:0]   STOREwhb;    // store word | byte | half byte
    output [31:0]  dout;
    
    reg  [31:0] dmem[127:0];
    reg  [31:0] tempreg;        // change data before, and store later
    wire [8:2]  addr;           // word address
    wire [31:0] addrByte;       // change last 2 bit to 0, byte address

    assign addr = address[8:2];
    assign addrByte = addr<<2;
    assign dout = dmem[addrByte[8:2]];
    
    always @(posedge clk)
        if (MEMWr) begin
            tempreg = dmem[addrByte[8:2]][31:0];
            case (STOREwhb)
                `STOREwhb_SW:   tempreg = din;
                `STOREwhb_SH:   tempreg[16*address[1] + 15 -: 16] = din[15:0];
                `STOREwhb_SB:   tempreg[8*address[1:0] + 7 -:  8] = din[7:0];
            endcase
            dmem[addrByte[8:2]][31:0] = tempreg[31:0];
        end
    
endmodule    
