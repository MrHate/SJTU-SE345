module io_output_reg(addr,datain,write_io_enable,io_clk,clm,out_port0,out_port1);
	input [31:0] addr,datain;
	input	write_io_enable,io_clk;
	input clm;
	
	output [31:0] out_port0,out_port1;
	
	reg [31:0] out_port0,out_port1;
	
	always@(posedge io_clk or negedge clm) begin
		if(clm==0) begin
			out_port0 <= 0;
			out_port1 <= 0;
		end
		else
			if(write_io_enable == 1)
				case (addr[7:2])
					6'b100000: out_port0<=datain;
					6'b100001: out_port1<=datain;
				endcase
	end
endmodule	