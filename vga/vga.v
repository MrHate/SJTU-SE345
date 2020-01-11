module vga(clk, sync_n, b1, b2, b3, r, g, b, h, v, bn, 
	hex5, hex4, hex3, hex2, hex1, hex0);

    input clk, sync_n, b1, b2, b3;
		output [6:0] hex5, hex4, hex3, hex2, hex1, hex0;
    output reg [7:0] r, g, b;
    output wire [41:0] scores_out;
    output h, v, bn;
    
    wire reg clk_v;
    wire[9:0] scanner_x, scanner_y;

    reg [5:0] rand_num;
    reg [6:0] score1, score2, timer1, timer2;
	 
		parameter integer tree_x = 280, tree_y = 0, tree_width = 80, tree_height = 480;
		parameter integer left_branch_x = 200, right_branch_x = 360, branch_width = 80, branch_height = 30, branch_interval = 48;
		parameter integer player_height = 180, player_width = 40, player_left_x = 220, player_right_x = 400, player_y = 300;
		parameter integer button_delay = 5000000;

		reg [6:0] left_branch_y, right_branch_y;
		reg player_pos, game_over = 0;
		reg [31:0] counter_b1, counter_b2, counter_b3, counter_rest, counter_over_delay;
		reg [5:0] rest_time;

    vga_gen vga_gen(clk_v, sync_n, scanner_y, scanner_x, h, v, bn);
    sevenseg s1(score1, hex1);
    sevenseg s2(score2, hex0);
    sevenseg t1(timer1, hex6);
    sevenseg t2(timer2, hex5);

    initial begin
			clk_v = 0;
			r = 0;
			g = 0;
			b = 0;

			rand_num = 34;
			game_over = 0;
			left_branch_y = 10'b0111010;
			right_branch_y = 10'b0000101;
			player_pos = 0;
			counter_b1 = 0;
			counter_b2 = 0;
			counter_b3 = 0;
			score1 = 0;
			score2 = 0;
			rest_time = 30;
			counter_rest = 0;
    end
	 
always @ (rest_time) begin 
	timer1 <= rest_time % 10;
	timer2 <= rest_time / 10;
end

always @ (posedge clk) begin
	// generate 25MHz
	clk_v <= ~clk_v;

	// update timer
	counter_rest <= counter_rest + 1;
	if( counter_rest == 50000000 && rest_time > 0 && !game_over) begin
		counter_rest <= 0;
		rest_time <= rest_time - 1;
	end
	
	// check buttons
	if (!b1 && !counter_b1) counter_b1 <= 1;
	if (counter_b1) counter_b1 <= counter_b1 + 1;
	if(counter_b1 == button_delay) begin
		if(b1 && !game_over) begin
			player_pos <= 0;
			counter_b1 <= 0;
			score1 <= score1 + 1;
			left_branch_y <= left_branch_y << 1 | (1 & rand_num);
			right_branch_y <= right_branch_y << 1 | (1 & ~rand_num);
		end
		else counter_b1 <= 1;
	end
	
	if (!b2 && !counter_b2) counter_b2 <= 1;
	if (counter_b2) counter_b2 <= counter_b2 + 1;
	if(counter_b2 == button_delay) begin
		if(b2 && !game_over) begin
			player_pos <= 1;
			counter_b2 <= 0;
			score1 <= score1 + 1;
			left_branch_y <= left_branch_y << 1 | (1 & rand_num);
			right_branch_y <= right_branch_y << 1 | (1 & ~rand_num);
		end
		else counter_b2 <= 1;
	end
	
	if (!b3 && !counter_b3) counter_b3 <= 1;
	if (counter_b3) counter_b3 <= counter_b3 + 1;
	if(counter_b3 == button_delay) begin
		if(b3) begin	 
			 game_over <= 0;
			 left_branch_y <= 10'b0111010;
			 right_branch_y <= 10'b0000101;
			 player_pos <= 0;
			 counter_b1 <= 0;
			 counter_b2 <= 0;
			 counter_b3 <= 0;
			 score1 <= 0;
			 score2 <= 0;
			 rest_time <= 30;
			 counter_rest <= 0;
		end
		else counter_b3 <= 1;
	end
	
	// check collision
	if(rest_time <= 0 || ((left_branch_y >> 6) & ~player_pos) || ((right_branch_y >> 6) & player_pos)) begin 
		game_over <= 1;
		//rest_time <= 0;
	end
	
	if (score1 == 10) begin
		score2 <= score2 + 1;
		score1 <= 0;
	end
end

// button pressed and evaluate
integer i,ky;
always @ (negedge clk) begin
   
	// update random number
	rand_num <= { rand_num[1:0], ~rand_num[5:2] };

	// draw background
	r <= 8'hc2;
	g <= 8'hff;
	b <= 8'hf8;
 
	// draw trunk
	if(scanner_x >= tree_x && scanner_x <= tree_x + tree_width) begin
		r <= 8'h56;
		g <= 8'h3f;
		b <= 8'h12;
	end
	
	// draw branches
	for(i=0;i<10;i=i+1)
		if((left_branch_y >> i) & 1) begin
			ky = i * branch_interval;
			if(scanner_x >= left_branch_x && scanner_x <= left_branch_x + branch_width && scanner_y >= ky && scanner_y <= ky + branch_height) begin
				r <= 8'h56;
				g <= 8'h3f;
				b <= 8'h12;
			end
		end
		else if((right_branch_y >> i) & 1) begin
			ky = i * branch_interval;
			if(scanner_x >= right_branch_x && scanner_x <= right_branch_x + branch_width && scanner_y >= ky && scanner_y <= ky + branch_height) begin
				r <= 8'h56;
				g <= 8'h3f;
				b <= 8'h12;
			end
		end
	
	// draw player
	if(scanner_y >= player_y && scanner_y <= player_y + player_height)
		if(!player_pos && scanner_x >= player_left_x && scanner_x <= player_left_x + player_width) begin
			r <= 8'hff;
			g <= 8'h42;
			b <= 8'h44;
		end
		else if(player_pos && scanner_x >= player_right_x && scanner_x <= player_right_x + player_width) begin
			r <= 8'hff;
			g <= 8'h42;
			b <= 8'h44;
		end
	
		
	if(game_over) begin 
		r <= 8'hff;
		g <= 8'h00;
		b <= 8'h00;
	end
	
end

endmodule

