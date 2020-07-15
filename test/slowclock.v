reg [5:0] slowclk_cnt;
assign slowclk_ce = slowclk_cnt == 6'h20;

always @(posedge clk) begin
	if (rst)
		slowclk_cnt <= 6'b0;
	else
		slowclk_cnt <= slowclk_cnt + 6'd1;
end
