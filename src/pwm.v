module pwm #(
	parameter BASE_ADDR = 5'h0
) (
	input rst,
	input clk,

	input [4:0] csr_a,
	input [7:0] csr_di,
	input csr_we,
	output reg [7:0] csr_do,

	input pwm_ce,
	output pwm_out
);

reg pwm_en;
reg [1:0] pwm_scale;
reg [6:0] duty_cycle;
always @(posedge clk) begin
	if (rst) begin
		pwm_en <= 1'b0;
		pwm_scale <= 2'b0;
		duty_cycle <= 7'h00;
	end else begin
		csr_do <= 8'b0;
		if (csr_a == BASE_ADDR) begin
			csr_do <= {pwm_en, 5'b0, pwm_scale};
			if (csr_we)
				{pwm_en, pwm_scale} <= {csr_di[7], csr_di[1:0]};
		end
		if (csr_a == BASE_ADDR + 1) begin
			csr_do <= {1'b0, duty_cycle};
			if (csr_we)
				duty_cycle <= csr_di[6:0];
		end
	end
end

reg pwm_reset;
always @(*) begin
	case (pwm_scale[1:0])
		2'b00: pwm_reset = pwm_counter[7];
		2'b01: pwm_reset = pwm_counter[6];
		2'b10: pwm_reset = pwm_counter[5];
		2'b11: pwm_reset = pwm_counter[4];
	endcase
end

reg [7:0] pwm_counter;
always @(posedge clk) begin
	if (rst || pwm_reset)
		pwm_counter <= 8'd1;
	else if (pwm_ce)
		pwm_counter <= pwm_counter + 8'b1;
end

wire pwm_match = pwm_counter == duty_cycle;
reg pwm_out_int;
always @(posedge clk) begin
	if (rst || pwm_reset)
		pwm_out_int <= 1'b1;
	else if (pwm_match)
		pwm_out_int <= 1'b0;
end
assign pwm_out = |duty_cycle & pwm_en & pwm_out_int;

endmodule
