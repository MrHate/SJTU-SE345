module stopwatch_01(clk,key_reset,key_start_pause,key_display_stop,
							hex0,hex1,hex2,hex3,hex4,hex5,
							led0,led1,led2,led3);
	input	clk,key_reset,key_start_pause,key_display_stop;
	output [6:0] hex0,hex1,hex2,hex3,hex4,hex5;
	output led0,led1,led2,led3;
	reg led0,led1,led2,led3;
	
	reg display_work;
	reg counter_work;
	
	parameter DELAY_TIME = 10000000;
	
	reg [3:0] minute_display_high;
	reg [3:0] minute_display_low;
	reg [3:0] second_display_high;
	reg [3:0] second_display_low;
	reg [3:0] msecond_display_high;
	reg [3:0] msecond_display_low;
	
	reg [3:0] minute_counter_high;
	reg [3:0] minute_counter_low;
	reg [3:0] second_counter_high;
	reg [3:0] second_counter_low;
	reg [3:0] msecond_counter_high;
	reg [3:0] msecond_counter_low;
	
	reg [31:0] counter_50M;
	
	reg reset_1_time;
	reg [31:0] counter_reset;
	reg start_1_time;
	reg [31:0] counter_start;
	reg display_1_time;
	reg [31:0] counter_display;
	
	reg start;
	reg display;
	
	sevenseg LED8_minute_display_high(minute_display_high,hex5);
	sevenseg LED8_minute_display_low(minute_display_low,hex4);
	sevenseg LED8_second_display_high(second_display_high,hex3);
	sevenseg LED8_second_display_low(second_display_low,hex2);
	sevenseg LED8_msecond_display_high(msecond_display_high,hex1);
	sevenseg LED8_msecond_display_low(msecond_display_low,hex0);
	
	initial
	begin
		display_work = 0;
		counter_work = 0;
		
		minute_display_high = 0;
		minute_display_low = 0;
		second_display_high = 0;
		second_display_low = 0;
		msecond_display_high = 0;
		msecond_display_low = 0;
		
		minute_counter_high = 0;
		minute_counter_low = 0;
		second_counter_high = 0;
		second_counter_low = 0;
		msecond_counter_high = 0;
		msecond_counter_low = 0;
		
		counter_50M = 0;
		
		reset_1_time = 0;
		counter_reset = 0;
		start_1_time = 0;
		counter_start = 0;
		display_1_time = 0;
		counter_display = 0;
		
		start = 0;
		display = 0;
	end
		
	// process with input and display at each clk posedge
	always@(posedge clk)
	begin
		
		// increase the minimal counter when counter_work is true
		if(counter_work)counter_50M = counter_50M + 1;
		
		// update LEDs when display_work is true
		if(display_work)
		begin
			minute_display_high  = minute_counter_high;
			minute_display_low   = minute_counter_low;
			second_display_high  = second_counter_high;
			second_display_low   = second_counter_low;
			msecond_display_high = msecond_counter_high;
			msecond_display_low  = msecond_counter_low;
		end
		
		// check input with joggle removing
		// it looks like triggering on the button pressed down rather up
		if(!key_display_stop && !counter_display)counter_display = 1;
		if(counter_display)counter_display = counter_display + 1;
		if(!key_reset && !counter_reset)counter_reset = 1;
		if(counter_reset)counter_reset = counter_reset + 1;
		if(!key_start_pause&& !counter_start)counter_start = 1;
		if(counter_start)counter_start = counter_start + 1;
		
		if(counter_reset == DELAY_TIME)
		begin
			minute_display_high = 0;
			minute_display_low = 0;
			second_display_high = 0;
			second_display_low = 0;
			msecond_display_high = 0;
			msecond_display_low = 0;
			
			minute_counter_high = 0;
			minute_counter_low = 0;
			second_counter_high = 0;
			second_counter_low = 0;
			msecond_counter_high = 0;
			msecond_counter_low = 0;
			
			counter_50M = 0;
		end
		
		if(counter_display == DELAY_TIME)
		begin
			counter_display = 0;
			display_work = !display_work;
		end
		
		if(counter_start == DELAY_TIME)
		begin
			counter_start = 0;
			counter_work = !counter_work;
		end
		
		// process counter
		if(counter_50M == 500000)
		begin
			counter_50M = 0;
			msecond_counter_low = msecond_counter_low + 1;
			
			if(msecond_counter_low == 10)
			begin
				msecond_counter_high = msecond_counter_high + 1;
				msecond_counter_low = 0;
			end
			
			if(msecond_counter_high == 10)
			begin
				second_counter_low = second_counter_low + 1;
				msecond_counter_high = 0;
			end
			
			if(second_counter_low == 10)
			begin
				second_counter_high = second_counter_high + 1;
				second_counter_low = 0;
			end
			
			if(second_counter_high == 6)
			begin
				minute_counter_low = minute_counter_low + 1;
				second_counter_high = 0;
			end
			
			if(minute_counter_low == 10)
			begin
				minute_counter_high = minute_counter_high + 1;
				minute_counter_low = 0;
			end
			
			if(minute_counter_high == 10)
			begin
				minute_counter_high = 0;
			end
		end
	end
	
endmodule

module sevenseg(data,ledsegments);
	input [3:0] data;
	output ledsegments;
	reg [6:0] ledsegments;
	
	always@(*)
	case(data)
		0:ledsegments = 7'b100_0000;
		1:ledsegments = 7'b111_1001;
		2:ledsegments = 7'b010_0100;
		3:ledsegments = 7'b011_0000;
		4:ledsegments = 7'b001_1001;
		5:ledsegments = 7'b001_0010;
		6:ledsegments = 7'b000_0010;
		7:ledsegments = 7'b111_1000;
		8:ledsegments = 7'b000_0000;
		9:ledsegments = 7'b001_0000;
		default:ledsegments = 7'b111_1111;
	endcase 
endmodule
