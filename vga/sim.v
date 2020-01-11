module sim;
    reg clk;

    wire reset, b1, b2, b3, b4;
    wire [7:0] r, g, b;
    wire [14:0] scores;
    wire h, v, bn;

    game vga(clk, reset, b1, b2, b3, b4, r, g, b, h, v, bn, scores);

    initial begin
        clk = 1;
        while (1)
            #2  clk = ~clk;
    end

endmodule