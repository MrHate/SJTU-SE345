module reg_id_exe(dwreg, dm2reg, dwmem, djal, daluc, daluimm, dshift,
						dpc4, da, db, dimm, drn,
						clk, clrn,
						ewreg, em2reg, ewmem, ejal, ealuc, ealuimm, eshift,
						epc4, ea, eb, eimm, ern);
	input[31:0] da,db,dimm,dpc4;
	input[4:0] drn;
	input[3:0] daluc;
	input dwreg, dm2reg, dwmem, daluimm, dshift, djal, clk, clrn;
	
	output reg[31:0] ea,eb,eimm,epc4;
	output reg[4:0] ern;
	output reg[3:0] ealuc;
	output reg ewreg,em2reg,ewmem,ealuimm,eshift,ejal;
	
	always@(negedge clrn or posedge clk)
	if(clrn == 0)begin
		ewreg <= 0;
		em2reg <= 0;
		ewmem <= 0;
		ewmem <= 0;
		ealuc <= 0;
		ealuimm <= 0;
		ea <= 0;
		eb <= 0;
		eimm <= 0;
		ern <= 0;
		eshift <= 0;
		ejal <= 0;
		epc4 <= 0;
	end
	else begin
		ewreg <= dwreg;
		em2reg <= dm2reg;
		ewmem <= dwmem;
		ealuc <= daluc;
		ealuimm <= daluimm;
		ea <= da;
		eb <= db;
		eimm <= dimm;
		ern <= drn;
		eshift <= dshift;
		ejal <= djal;
		epc4 <= dpc4;
	end
endmodule