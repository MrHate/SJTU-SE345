module reg_if_id(pc4,ins,wpcir,clk,clrn,dpc4,inst);
	input[31:0] pc4,ins;
	input wpcir,clk,clrn;
	output[31:0] dpc4,inst;
	dffe32 pc_plus_4(pc4,clk,clrn,wpcir,dpc4);
	dffe32 inst_gen(ins,clk,clrn,wpcir,inst);
endmodule