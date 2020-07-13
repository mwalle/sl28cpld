module sl28_top(
	output HEALTHY_LED,

	/* I2C bus */
	inout I2C_LOCAL_SDA_3V3,
	input I2C_LOCAL_SCL_3V3,

	/* reset */
	input PORESET_n,

	/* interrupt */
	output CPLD_INTERRUPT_CFG_RCW_SRC2,

	output PWR_FORCE_DISABLE_n,
	output LCD0_BKLT_PWM_3V3,


	/* GPIO */
	inout GPIO0_CAM0_PWR_n,
	inout GPIO1_CAM1_PWR_n,
	inout GPIO2_CAM0_RST_n,
	inout GPIO3_CAM1_RST_n,
	inout GPIO4_HDA_RST_n,
	inout GPIO5_PWM_OUT,
	inout GPIO6_TACHIN,
	inout GPIO7,
	inout GPIO8,
	inout GPIO9,
	inout GPIO10,
	inout GPIO11
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

/* generate slow clocks */
reg [7:0] cnt_32khz;
reg ce_32khz;
always @(posedge clk) begin
	cnt_32khz <= cnt_32khz + 8'd1;
	ce_32khz <= 1'b0;
	if (cnt_32khz == 8'h90) begin
		cnt_32khz <= 8'd0;
		ce_32khz <= 1'b1;
	end
end

reg [14:0] cnt_1s;
reg ce_1s;
always @(posedge clk) begin
	ce_1s <= 1'b0;
	if (ce_32khz)
		cnt_1s <= cnt_1s + 15'd1;
	if (cnt_1s == 15'h7d00) begin
		cnt_1s <= 15'd0;
		ce_1s <= 1'b1;
	end
end

reg rst0, rst;
always @(posedge clk)
	{rst0, rst} <= {rst, ~PORESET_n};

reg healthy_led;
always @(posedge clk) begin
	if (ce_1s)
		healthy_led <= ~healthy_led;
end
assign HEALTHY_LED = healthy_led;

wire [4:0] csr_a;
wire [7:0] csr_di;
wire csr_we;
wire [7:0] csr_do_pwm0;
wire [7:0] csr_do_pwm1;
wire [7:0] csr_do_gpio0;
wire [7:0] csr_do_gpio1;
wire [7:0] csr_do;

assign csr_do = csr_do_pwm0 |
		csr_do_pwm1 |
		csr_do_gpio0 |
		csr_do_gpio1;

i2c_slave i2c_slave(
	.rst(rst),
	.clk(clk),

	.sda(I2C_LOCAL_SDA_3V3),
	.scl(I2C_LOCAL_SCL_3V3),

	.csr_a(csr_a),
	.csr_di(csr_do),
	.csr_we(csr_we),
	.csr_do(csr_di)
);


pwm #(
	.BASE_ADDR(5'hc)
) pwm0 (
	.rst(rst),
	.clk(clk),
	.pwm_ce(ce_32khz),

	.csr_a(csr_a),
	.csr_di(csr_di),
	.csr_we(csr_we),
	.csr_do(csr_do_pwm0),
	.pwm_out(LCD0_BKLT_PWM_3V3)
);

wire pwm1_out;
wire pwm1_en;
pwm #(
	.BASE_ADDR(5'he)
) pwm1 (
	.rst(rst),
	.clk(clk),
	.pwm_ce(ce_32khz),

	.csr_a(csr_a),
	.csr_di(csr_di),
	.csr_we(csr_we),
	.csr_do(csr_do_pwm1),
	.pwm_en(pwm1_en),
	.pwm_out(pwm1_out)
);

wire [7:0] gpio0_out;
wire [7:0] gpio0_in;
wire [7:0] gpio0_oe;
wire gpio0_irq;
gpio #(
	.BASE_ADDR(5'h10)
) gpio0 (
	.rst(rst),
	.clk(clk),

	.csr_a(csr_a),
	.csr_di(csr_di),
	.csr_we(csr_we),
	.csr_do(csr_do_gpio0),

	.out(gpio0_out),
	.in(gpio0_in),
	.oe(gpio0_oe),
	.irq(gpio0_irq)
);
assign GPIO0_CAM0_PWR_n = gpio0_oe[0] ? gpio0_out[0] : 1'bz;
assign GPIO1_CAM1_PWR_n = gpio0_oe[1] ? gpio0_out[1] : 1'bz;
assign GPIO2_CAM0_RST_n = gpio0_oe[2] ? gpio0_out[2] : 1'bz;
assign GPIO3_CAM1_RST_n = gpio0_oe[3] ? gpio0_out[3] : 1'bz;
assign GPIO4_HDA_RST_n = gpio0_oe[4] ? gpio0_out[4] : 1'bz;
assign GPIO5_PWM_OUT = pwm1_en ? pwm1_out : (gpio0_oe[5] ? gpio0_out[5] : 1'bz);
assign GPIO6_TACHIN = gpio0_oe[6] ? gpio0_out[6] : 1'bz;
assign GPIO7 = gpio0_oe[7] ? gpio0_out[7] : 1'bz;
assign gpio0_in[0] = GPIO0_CAM0_PWR_n;
assign gpio0_in[1] = GPIO1_CAM1_PWR_n;
assign gpio0_in[2] = GPIO2_CAM0_RST_n;
assign gpio0_in[3] = GPIO3_CAM1_RST_n;
assign gpio0_in[4] = GPIO4_HDA_RST_n;
assign gpio0_in[5] = GPIO5_PWM_OUT;
assign gpio0_in[6] = GPIO6_TACHIN;
assign gpio0_in[7] = GPIO7;

wire [7:0] gpio1_out;
wire [7:0] gpio1_in;
wire [7:0] gpio1_oe;
wire gpio1_irq;
gpio #(
	.BASE_ADDR(5'h15),
	.NUM_GPIOS(4)
) gpio1 (
	.rst(rst),
	.clk(clk),

	.csr_a(csr_a),
	.csr_di(csr_di),
	.csr_we(csr_we),
	.csr_do(csr_do_gpio1),

	.out(gpio1_out),
	.in(gpio1_in),
	.oe(gpio1_oe),
	.irq(gpio1_irq)
);
assign GPIO8 = gpio1_oe[0] ? gpio1_out[0] : 1'bz;
assign GPIO9 = gpio1_oe[1] ? gpio1_out[1] : 1'bz;
assign GPIO10 = gpio1_oe[2] ? gpio1_out[2] : 1'bz;
assign GPIO11 = gpio1_oe[3] ? gpio1_out[3] : 1'bz;
assign gpio1_in[0] = GPIO8;
assign gpio1_in[1] = GPIO9;
assign gpio1_in[2] = GPIO10;
assign gpio1_in[3] = GPIO11;

assign CPLD_INTERRUPT_CFG_RCW_SRC2 = gpio0_irq | gpio1_irq;

endmodule
