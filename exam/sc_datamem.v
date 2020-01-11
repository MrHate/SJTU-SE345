module sc_datamem (addr,datain,dataout,we,clock,mem_clk,dmem_clk,
						hex0,hex1,hex2,hex3,hex4,hex5,
						switch_input);
 
   input  [31:0]  addr;
   input  [31:0]  datain;
   
   input          we, clock,mem_clk;
   output [31:0]  dataout;
	wire [31:0] mem_dataout;
	reg [31:0] io_dataout;
   output         dmem_clk;
	
	input [7:0] switch_input;
	output reg [6:0] hex0,hex1,hex2,hex3,hex4,hex5;
   
   wire           dmem_clk;    
   wire           write_enable; 
   assign         write_enable = we & ~clock; 
   
   assign         dmem_clk = mem_clk & ( ~ clock) ; 
   
   lpm_ram_dq_dram  dram(addr[6:2],dmem_clk,datain,write_enable,mem_dataout );
	mux2x32 mux_mem_io_dataout(mem_dataout,io_dataout,addr[31],dataout);
	
	reg [6:0] ledsegments_low,ledsegments_high;
	reg [5:0] decimal_low,decimal_high;
	always@(posedge dmem_clk) begin
		if(we & addr[31]) begin
		
			decimal_low = datain[6:0] % 10;
			decimal_high = datain[6:0] / 10;
			
			hex0 <= 0;
			hex1 <= 0;
			hex2 <= 0;
			hex3 <= 0;
			hex4 <= 0;
			hex5 <= 0;
			
			case(decimal_low)
				0:ledsegments_low = 7'b100_0000;
				1:ledsegments_low = 7'b111_1001;
				2:ledsegments_low = 7'b010_0100;
				3:ledsegments_low = 7'b011_0000;
				4:ledsegments_low = 7'b001_1001;
				5:ledsegments_low = 7'b001_0010;
				6:ledsegments_low = 7'b000_0010;
				7:ledsegments_low = 7'b111_1000;
				8:ledsegments_low = 7'b000_0000;
				9:ledsegments_low = 7'b001_0000;
				default:ledsegments_low = 7'b111_1111;
			endcase
			case(decimal_high)
				0:ledsegments_high = 7'b100_0000;
				1:ledsegments_high = 7'b111_1001;
				2:ledsegments_high = 7'b010_0100;
				3:ledsegments_high = 7'b011_0000;
				4:ledsegments_high = 7'b001_1001;
				5:ledsegments_high = 7'b001_0010;
				6:ledsegments_high = 7'b000_0010;
				7:ledsegments_high = 7'b111_1000;
				8:ledsegments_high = 7'b000_0000;
				9:ledsegments_high = 7'b001_0000;
				default:ledsegments_high = 7'b111_1111;
			endcase
			
			case(addr)
				32'hffffff00:	begin
					hex5 <= ledsegments_low;
					hex0 <= 0;
				end
				32'hffffff10:	begin
					hex3 <= 0;
					hex2 <= 0;
				end
				32'hffffff20:	begin
					//hex5 <= 7'b1111111;
					hex1 <= 0;
					hex4 <= 0;
				end
			endcase
		end
	end

	always@(posedge dmem_clk) begin
		case(addr)
			32'hffffff60: io_dataout <= {24'b0,switch_input[7:0]};
			32'hffffff64: io_dataout <= {24'b0,switch_input[7:0]};
		endcase
	end
endmodule 