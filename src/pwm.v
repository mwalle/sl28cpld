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
	output reg pwm_en,
	output pwm_out
);


always @(*) begin
	csr_do = 8'b0;
	case (csr_a)
		BASE_ADDR + 5'h0: csr_do = {pwm_en, 5'b0, pwm_scale};
		BASE_ADDR + 5'h1: csr_do = {1'b0, duty_cycle};
	endcase
end

reg [1:0] pwm_scale;
reg [6:0] duty_cycle;
always @(posedge clk) begin
	if (rst) begin
		pwm_en <= 1'b0;
		pwm_scale <= 2'b0;
		duty_cycle <= 7'h00;
	end else begin
		if (csr_we) begin
			case (csr_a)
				BASE_ADDR + 5'h0: {pwm_en, pwm_scale} <= {csr_di[7], csr_di[1:0]};
				BASE_ADDR + 5'h1: duty_cycle <= csr_di[6:0];
			endcase
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
