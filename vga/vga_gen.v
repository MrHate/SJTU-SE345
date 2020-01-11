module vga_gen (clk, sync_n, row_i, col_i, sync_h, sync_v, blank_n);
    input clk, sync_n;

    output           sync_h, sync_v, blank_n;
    output reg [9:0] row_i, col_i;

    parameter integer visible_area_h = 640;
    parameter integer front_porch_h = 16;
    parameter integer sync_pulse_h = 96;
    parameter integer back_porch_h = 48;
    parameter integer whole_line_h = 800;

    parameter integer visible_area_v= 480;
    parameter integer front_porch_v= 10;
    parameter integer sync_pulse_v= 2;
    parameter integer back_porch_v= 33;
    parameter integer whole_line_v = 525;

    // sync/blank signals
    assign sync_v = ~(row_i >= visible_area_v + front_porch_v && row_i < visible_area_v + front_porch_v + sync_pulse_v);
    assign sync_h = ~(col_i >= visible_area_h + front_porch_h && col_i < visible_area_h + front_porch_h + sync_pulse_h);
    assign blank_n = row_i < visible_area_v && col_i < visible_area_h;

    initial begin
        row_i = 0;
        col_i = 0;
    end
    
    always @ (posedge clk) begin
        if (sync_n == 0) begin  // sync_n is avtive-low
            row_i <= 0; col_i <= 0;
        end else begin // update output index
            col_i = col_i + 1;
            if (col_i == whole_line_h) begin  // begin new line
                col_i = 0;
                row_i = row_i + 1;
                if (row_i == whole_line_v) begin  // begin new frame
                    row_i = 0;
                end
            end
        end
    end
endmodule
