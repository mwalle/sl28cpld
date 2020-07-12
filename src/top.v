module sl28_top(
	output HEALTHY_LED,

	inout I2C_LOCAL_SDA_3V3,
	input I2C_LOCAL_SCL_3V3,

	output PWR_FORCE_DISABLE_n,
	output LCD0_BKLT_PWM_3V3
);

wire clk;

assign PWR_FORCE_DISABLE_n = 1'bz;

altufm_none ufm(
	.arclk(1'b0),
	.ardin(1'b0),
	.arshft(1'b0),
	.drclk(1'b0),
	.drdin(1'b0),
	.drshft(1'b0),
	.erase(1'b0),
	.oscena(1'b1),
	.program(1'b0),

	.busy(),
	.drdout(),
	.osc(clk),
	.rtpbusy()
);

reg [22:0] counter;
always @(posedge clk)
	counter <= counter + 23'b1;

assign HEALTHY_LED = counter[22];

wire [4:0] csr_a;
wire [7:0] csr_di;
wire csr_we;
wire [7:0] csr_do_pwm0;
wire [7:0] csr_do_pwm1;
wire [7:0] csr_do;

assign csr_do = csr_do_pwm0 |
		csr_do_pwm1;

i2c_slave i2c_slave(
	.rst(1'b0),
	.clk(clk),

	.sda(I2C_LOCAL_SDA_3V3),
	.scl(I2C_LOCAL_SCL_3V3),

	.csr_a(csr_a),
	.csr_di(csr_do),
	.csr_we(csr_we),
	.csr_do(csr_di)
);


reg [7:0] clk_count;
reg pwm_ce;
always @(posedge clk) begin
	clk_count <= clk_count + 8'h1;
	pwm_ce <= 1'b0;
	if (clk_count == 8'h90) begin
		clk_count <= 8'h0;
		pwm_ce <= 1'b1;
	end
end

wire pwm0_out;
wire pwm0_en;
pwm #(
	.BASE_ADDR(5'hc)
) pwm0 (
	.rst(1'b0),
	.clk(clk),
	.pwm_ce(pwm_ce),

	.csr_a(csr_a),
	.csr_di(csr_di),
	.csr_we(csr_we),
	.csr_do(csr_do_pwm0),
	.pwm_en(pwm0_en),
	.pwm_out(pwm0_out)
);

pwm #(
	.BASE_ADDR(5'he)
) pwm1 (
	.rst(1'b0),
	.clk(clk),
	.pwm_ce(pwm_ce),

	.csr_a(csr_a),
	.csr_di(csr_di),
	.csr_we(csr_we),
	.csr_do(csr_do_pwm1),
	.pwm_out(LCD0_BKLT_PWM_3V3)
);

endmodule
