module sl28_top(
	output HEALTHY_LED,

	/* I2C bus */
	inout I2C_LOCAL_SDA_3V3,
	input I2C_LOCAL_SCL_3V3,

	/* reset */
	input PORESET_n,

	/* board control & interrupt */
	output PWR_FORCE_DISABLE_n,
	output reg SER2_TX_CFG_RCW_SRC0,
	output reg SER1_TX_CFG_RCW_SRC1,
	output reg CPLD_INTERRUPT_CFG_RCW_SRC2,

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
	inout GPIO11,

	/* GPO */
	output LCD0_VDD_EN_3V3,
	output LCD0_BKLT_EN_3V3,
	output EMMC_RST_n,
	output PTN3460_RST_n,
	output PTN3460_PD_n,
	output SDIO_PWR_EN,

	/* GPI */
	input POWER_BTN_n,
	input FORCE_RECOV_n,
	input SLEEP_n,
	input BATLOW_n,
	input LID_n,
	input CHARGING_n,
	input CHARGER_PRSNT_n,

	/* USB control */
	inout USB3_EN_OC_n,
	input USB3_DRVVBUS,
	output USB3_PWRFAULT_3V3,

	/* PWM */
	output LCD0_BKLT_PWM_3V3
);

usbfixer usbfixer (
	.usb_en_oc_n(USB3_EN_OC_n),
	.usb_drvvbus(USB3_DRVVBUS),
	.usb_pwrfault(USB3_PWRFAULT_3V3)
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

wire ce_32khz;
wire ce_8hz;
wire ce_1hz;
clockgen clockgen (
	.clk(clk),
	.ce_32khz(ce_32khz),
	.ce_8hz(ce_8hz),
	.ce_1hz(ce_1hz)
);

wire force_recovery = ~FORCE_RECOV_n;
reg rst0, rst;
always @(posedge clk)
	{rst0, rst} <= {rst, ~PORESET_n};
wire drive_rcw_src = !force_recovery & (rst0 | rst);

wire irq_out;
always @(*) begin
	SER2_TX_CFG_RCW_SRC0 = 1'bz;
	SER1_TX_CFG_RCW_SRC1 = 1'bz;
	CPLD_INTERRUPT_CFG_RCW_SRC2 = 1'bz;

	if (!force_recovery) begin
		if (drive_rcw_src) begin
			SER2_TX_CFG_RCW_SRC0 = 1'b0;
			SER1_TX_CFG_RCW_SRC1 = 1'b1;
			CPLD_INTERRUPT_CFG_RCW_SRC2 = 1'b0;
		end
		CPLD_INTERRUPT_CFG_RCW_SRC2 = irq_out;
	end
end

reg healthy_led;
always @(posedge clk) begin
	if (ce_1hz)
		healthy_led <= ~healthy_led;
end
assign HEALTHY_LED = healthy_led;

wire [4:0] csr_a;
wire [7:0] csr_di;
wire csr_we;
wire [7:0] csr_do;
reg [7:0] csr_do_rcw_ctrl;
wire [7:0] csr_do_pwm0;
wire [7:0] csr_do_pwm1;
wire [7:0] csr_do_gpio0;
wire [7:0] csr_do_gpio1;
wire [7:0] csr_do_gpo;
wire [7:0] csr_do_gpi;
wire [7:0] csr_do_tacho;
assign csr_do = csr_do_rcw_ctrl |
		csr_do_pwm0 |
		csr_do_pwm1 |
		csr_do_gpio0 |
		csr_do_gpio1 |
		csr_do_gpo |
		csr_do_tacho |
		csr_do_gpi;

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

gpo #(
	.BASE_ADDR(5'h1a),
	.NUM_GPIOS(6),
	.DFL_STATE(5'b110001)
) gpo (
	.rst(rst),
	.clk(clk),

	.csr_a(csr_a),
	.csr_di(csr_di),
	.csr_we(csr_we),
	.csr_do(csr_do_gpo),

	.out({
		SDIO_PWR_EN,
		PTN3460_PD_n,
		PTN3460_RST_n,
		EMMC_RST_n,
		LCD0_BKLT_EN_3V3,
		LCD0_VDD_EN_3V3
	})
);

gpi #(
	.BASE_ADDR(5'h1b),
	.NUM_GPIOS(7)
) gpi (
	.rst(rst),
	.clk(clk),

	.csr_a(csr_a),
	.csr_di(csr_di),
	.csr_we(csr_we),
	.csr_do(csr_do_gpi),

	.in({
		CHARGER_PRSNT_n,
		CHARGING_n,
		LID_n,
		BATLOW_n,
		SLEEP_n,
		FORCE_RECOV_n,
		POWER_BTN_n
	})
);

tacho #(
	.BASE_ADDR(5'hb)
) tacho (
	.rst(rst),
	.clk(clk),

	.csr_a(csr_a),
	.csr_di(csr_di),
	.csr_we(csr_we),
	.csr_do(csr_do_tacho),

	.ce_1hz(ce_1hz),
	.tacho_in(GPIO6_TACHIN)
);

always @(posedge clk) begin
	csr_do_rcw_ctrl = 8'h00;
	if (csr_a[4:1] == 4'b0)
		csr_do_rcw_ctrl = 8'hff;
end

assign irq_out = gpio0_irq | gpio1_irq;

endmodule
