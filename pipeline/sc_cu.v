module sc_cu (mwreg,mrn,ern,ewreg,em2reg,mm2reg,rsrtequ,func,op,rs,rt,
					wreg,m2reg,wmem,aluc,regrt,aluimm,fwda,fwdb,wpcir,sext,
					pcsource,shift,jal);
   input  [5:0] op,func;
   input mwreg,ewreg,em2reg,mm2reg,rsrtequ;
	input[4:0] mrn,ern,rs,rt;
	
   output       wreg,regrt,jal,m2reg,shift,aluimm,sext,wmem,wpcir;
   output [3:0] aluc;
   output [1:0] pcsource;
	output reg[1:0] fwda,fwdb;
	 
   wire r_type = ~|op;
   wire i_add = r_type & func[5] & ~func[4] & ~func[3] &
                ~func[2] & ~func[1] & ~func[0];          //100000
   wire i_sub = r_type & func[5] & ~func[4] & ~func[3] &
                ~func[2] &  func[1] & ~func[0];          //100010
      
   //  please complete the deleted code.
   
   wire i_and = r_type &  func[5] & ~func[4] & ~func[3] &  func[2] & ~func[1] & ~func[0]; //100100
   wire i_or  = r_type &  func[5] & ~func[4] & ~func[3] &  func[2] & ~func[1] &  func[0]; //100101

   wire i_xor = r_type & func == 6'b100110;
   wire i_sll = r_type & func == 6'b000000;
   wire i_srl = r_type & func == 6'b000010;                                                                        
   wire i_sra = r_type & func == 6'b000011;
   wire i_jr  = r_type & func == 6'b001000;
                
   wire i_addi = op == 6'b001000;
   wire i_andi = op == 6'b001100;
   
   wire i_ori  = op == 6'b001101;
   wire i_xori = op == 6'b001110;
   wire i_lw   = op == 6'b100011;
   wire i_sw   = op == 6'b101011;
   wire i_beq  = op == 6'b000100;
   wire i_bne  = op == 6'b000101;
   wire i_lui  = op == 6'b001111;
   wire i_j    = op == 6'b000010;
   wire i_jal  = op == 6'b000011;
   
   wire i_rs = i_add | i_sub | i_and | i_or | i_xor | i_jr | i_addi |
                i_andi | i_ori | i_xori | i_lw | i_sw | i_beq | i_bne;
    
   wire i_rt = i_add | i_sub | i_and | i_or | i_xor | i_sll | i_srl |
                i_sra | i_sw | i_beq | i_bne;
    
   assign wpcir = ~(ewreg & em2reg & (ern != 0) & (i_rs & (ern == rs) |
                                                    i_rt & (ern == rt)));
   assign pcsource[1] = i_jr | i_j | i_jal;
   assign pcsource[0] = ( i_beq & rsrtequ ) | (i_bne & ~rsrtequ) | i_j | i_jal ;
   
   assign wreg = (i_add | i_sub | i_and | i_or   | i_xor  |
                 i_sll | i_srl | i_sra | i_addi | i_andi |
                 i_ori | i_xori | i_lw | i_lui  | i_jal) & wpcir;
   
   assign aluc[3] = i_sra;
   assign aluc[2] = i_sub | i_or | i_lui | i_srl | i_sra | i_ori | i_bne | i_beq;
   assign aluc[1] = i_xor | i_lui | i_sll | i_srl | i_sra | i_xori;
   assign aluc[0] = i_and | i_or | i_sll | i_srl | i_sra | i_andi | i_ori;
   assign shift   = i_sll | i_srl | i_sra ;

   assign aluimm  = i_addi | i_andi | i_ori | i_xori | i_lw | i_sw | i_lui;
   assign sext    = i_addi | i_lw | i_sw | i_beq | i_bne;
   assign wmem    = i_sw & wpcir;
   assign m2reg   = i_lw;
   assign regrt   = i_addi | i_andi | i_ori | i_xori | i_lw | i_lui;
   assign jal     = i_jal;
	
	 always @ (ewreg or mwreg or ern or mrn or em2reg or mm2reg or rs or rt) begin
        fwda = 2'b00;
        fwdb = 2'b00;

        if (ewreg & (ern != 0) & (ern == rs) & ~em2reg) begin
            fwda = 2'b01;   // exe_alu
        end else begin
            if (mwreg & (mrn != 0) & (mrn == rs) & ~mm2reg) begin
                fwda = 2'b10; // mem_alu
            end else begin
                if (mwreg & (mrn != 0) & (mrn == rs) & mm2reg) begin
                    fwda = 2'b11;   // mem_lw
                end
            end
        end

        // we put condition in order so there is no conflict.
        if (ewreg & (ern != 0) & (ern == rt) & ~em2reg) begin
            fwdb = 2'b01; // exe_alu
        end else begin
            if (mwreg & (mrn != 0) & (mrn == rt) & ~mm2reg) begin
                fwdb = 2'b10; // mem_alu
            end else begin
                if (mwreg & (mrn != 0) & (mrn == rt) & mm2reg) begin
                    fwdb = 2'b11; // mem_lw
                end
            end
        end
    end


endmodule
