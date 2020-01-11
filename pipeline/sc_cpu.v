module sc_cpu (clock,resetn,
					ins,mmo,pc,
					mwmem,malu,mb);
   input [31:0] ins,mmo;
   input clock,resetn;
   output [31:0] pc,malu,mb;
   output mwmem;
	
	// pc
	wire[31:0] npc;
	
	// IF
	wire[31:0] inst,dpc4,pc4,bpc;
	wire wpcir;
	
	// ID
	wire[4:0] drn;
	wire[31:0] da,db,q1,q2;
	wire[1:0] pcsource,fwda,fwdb;
	wire[2:0] daluc;
	wire 			  regrt,sext,rsrtequ,e,dwreg,dm2reg,dwmem,dshift,djal,daluimm;
   wire [15:0]   dext;                // high 16 sign bit
   wire [31:0]   dimm; // sign extend to high 16
   wire [31:0]   offset;   //offset(include sign extend)
   
	wire[5:0] func = inst[5:0];
	wire[5:0] op = inst[31:26];
	wire[4:0] rs = inst[25:21];
	wire[4:0] rt = inst[20:16];
	wire[4:0] rd = inst[15:11];
   wire [31:0] jpc = {dpc4[31:28],inst[25:0],2'b00}; // j address 
	
	// EXE
	wire ewmem,ejal,ealuimm,eshift,ewreg,em2reg;
	wire[2:0] ealuc;
	wire[4:0] ern,ern0;
	wire[31:0] alua,alub,ea,eb;
	wire[31:0] eimm,epc4,epc8,ealu,ealu_z;
	wire[31:0] sa = {27'b0,eimm[10:6]};
	wire z;
	
	// MEM
	wire[31:0] mmo,malu,mb;
	wire[4:0] mrn;
	wire mwreg,mm2reg,mwmem;
	
	// WB
	wire wwreg,wm2reg;
	wire[4:0] wrn;
	wire[31:0] wdi,wmo,walu;
                        
	// pc
   dffe32 ip (npc,clock,resetn,wpcir,pc);
	
	// IF stage
   assign pc4 = pc + 32'h4;
   mux4x32 nextpc(pc4,bpc,da,jpc,pcsource,npc);
	
	reg_if_id reg_fd(pc4,ins,wpcir,clock,resetn,dpc4,inst);
	
	// ID stage
   sc_cu cu (mwreg, mrn, ern, ewreg, em2reg, mm2reg, rsrtequ, func,
                op, rs, rt, dwreg, dm2reg, dwmem, daluc, regrt, daluimm,
                fwda, fwdb, wpcir, sext, pcsource, dshift, djal);
   regfile rf (rs,rt,wdi,wrn,wwreg,clock,resetn,q1,q2);
	mux4x32 id_alu_a(q1,ealu,malu,mmo,fwda,da);
	mux4x32 id_alu_b(q2,ealu,malu,mmo,fwdb,db);
   mux2x5 reg_wn (rd,rt,regrt,drn);
	assign rsrtequ = ~|(da^db);
	assign e = sext & inst[15]; 
	assign dext = {16{e}};
	assign dimm = {dext,inst[15:0]};
	assign offset = {dimm[29:0],2'b00};
	cla32 br_addr(dpc4,offset,1'b0,bpc);
	
	reg_id_exe reg_ie(dwreg, dm2reg, dwmem, djal, daluc, daluimm, dshift,
						dpc4, da, db, dimm, drn,
						clock, resetn,
						ewreg, em2reg, ewmem, ejal, ealuc, ealuimm, eshift,
						epc4, ea, eb, eimm, ern0);
	
	// EXE stage
   mux2x32 exe_alu_a (ea,sa,eshift,alua);
   mux2x32 exe_alu_b (eb,eimm,ealuimm,alub);
   alu al_unit (alua,alub,ealuc,ealu_z,z);
	assign epc8 = epc4 + 32'h4;
	mux2x32 save_pc8(ealu_z,epc8,ejal,ealu);
	assign ern = ern0 | {5{ejal}};
	
	reg_exe_mem reg_em(ewreg,em2reg,ewmem,
						 ealu,eb,ern,
						 clock,resetn,
						 mwreg,mm2reg,mwmem,
						 malu,mb,mrn);
						 
	// MEM stage
	// malu -> aluout
	// eb -> datain
	// mwmem -> we
	// dataout -> mmo
	
	reg_mem_wb reg_mw(mwreg,mm2reg,
						malu,mmo,mrn,
						clock,resetn,
						wwreg,wm2reg,
						walu,wmo,wrn);
						
	// WB stage
	mux2x32 reg_di(walu,wmo,wm2reg,wdi);
	
endmodule
