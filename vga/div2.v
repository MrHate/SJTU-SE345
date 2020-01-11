module div2 (q,clk); //   输出q，输入时钟CLK，同步复位信号RESET.
    output q;
    input clk;
    reg q;
    always @ (posedge clk)
      q<=~q; // 否则q信号翻转
endmodule