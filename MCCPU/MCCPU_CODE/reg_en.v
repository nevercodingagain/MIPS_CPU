module enreg #(parameter WIDTH = 8)
    (
    input   clk, 
    input   rst, 
    input   enwrite,
    input   [WIDTH-1:0] in,
    output  [WIDTH-1:0] out);

    reg     [WIDTH-1:0] temp;
    always@(posedge clk, posedge rst) begin
        if (rst)
            temp <= 0;
        else if (enwrite)
            temp <= in;
    end

    assign out = temp;
    
endmodule