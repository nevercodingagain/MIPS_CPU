module register #(parameter WIDTH = 8)
    (
    input clk, 
    input rst, 
    input   [WIDTH-1:0] in, 
    output  [WIDTH-1:0] out);

    reg     [WIDTH-1:0] temp;
    always@(posedge clk, posedge rst) begin
        if (rst)
            temp <= 0;
        else
            temp <= in;
    end

    assign out = temp;
    
endmodule