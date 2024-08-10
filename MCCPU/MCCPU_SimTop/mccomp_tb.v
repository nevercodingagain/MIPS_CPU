
// testbench for simulation
module mccomp_tb();
    
    reg  clk, rstn;
    reg  [4:0] reg_sel;
    wire [31:0] reg_data;
        
    // instantiation of mccomp
    mccomp U_MCCOMP(
        .clk(clk), .rstn(rstn), .reg_sel(reg_sel), .reg_data(reg_data) 
    );
    
    initial begin
        $readmemh( "mipstest_sim.dat" , U_MCCOMP.U_MEM.dmem); // load instructions into instruction memory
        clk = 1;
        rstn = 1;
        #50;
        rstn = 0;
        #50;
        rstn = 1;
        reg_sel = 7;
    end
    
    always begin
        #(50) clk = ~clk;
    end //end always
    
endmodule