module cla32 (pc,x1,x2,p4);
   input [31:0] pc;
   input [31:0] x1,x2,p4;
   

   assign p4 = pc + 32'b4;   
   assign adr = p4 + offset;
   
endmodule 