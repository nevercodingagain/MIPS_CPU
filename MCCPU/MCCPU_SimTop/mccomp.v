module mccomp(clk, rstn, reg_sel, reg_data);
    input          clk;
    input          rstn;
    input  [4:0]   reg_sel;
    output [31:0]  reg_data;
    
    wire [31:0]    instr;
    wire [31:0]    PC;
    wire           MemWrite;
    wire [31:0]    mem_addr, mem_din, mem_dout;
    wire [1:0]     STOREwhb;
    
    wire rst = ~rstn;
    
    // instantiation of multi-cycle CPU   
    mccpu U_MCCPU(
            .clk(clk),                 // input:  cpu clock
            .rst(rst),                 // input:  reset
            .readdata(mem_dout),       // input:  data to cpu  
            .instr(instr),             // input:  instruction
            .PC(PC),                   // output: PC
            .MemWrite(MemWrite),       // output: memory write signal
            .writedata(mem_din),       // output: data from cpu to memory
            .adr(mem_addr),            // output: address from cpu to memory
            .reg_sel(reg_sel),         // input:  register selection
            .reg_data(reg_data),       // output: register data
            .STOREwhb(STOREwhb)
            );
    
    // instantiation of memory  
    mem    U_MEM(
            .clk(clk),                  // input:  cpu clock
            .MEMWr(MemWrite),           // input:  ram write
            .address(mem_addr[8:0]),    // input:  ram address
            .din(mem_din),              // input:  data to ram
            .STOREwhb(STOREwhb),        // input:  store data choices
            .dout(mem_dout)             // output: data from ram
            );
endmodule

