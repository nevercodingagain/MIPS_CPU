
// testbench for simulation
module sccomp_tb();
    
    reg  clk, rstn;
    reg  [4:0] reg_sel;
    wire [31:0] reg_data;
        
    // instantiation of sccomp
    sccomp U_SCCOMP(
        .clk(clk), .rstn(rstn), .reg_sel(reg_sel), .reg_data(reg_data) 
    );
    
    initial begin
        $readmemh( "mipstest_sim.dat" , U_SCCOMP.U_IM.ROM); // load instructions into instruction memory
    //    $monitor("PC = 0x%8X, instr = 0x%8X", U_SCCOMP.PC, U_SCCOMP.instr); // used for debug
        clk = 1;
        rstn = 1;
        #50;
        rstn = 0;
        #50 ;
        rstn = 1;
        #50 ;
        reg_sel = 7;
    end
    
    always begin
        #(50) clk = ~clk;
    end //end always
    
endmodule