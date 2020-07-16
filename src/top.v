module sl28_top #(
	CPLD_VERSION = 8'h20
) (
	output HEALTHY_LED_n,

	/* I2C bus */
	inout I2C_LOCAL_SDA_3V3,
	inout I2C_LOCAL_SCL_3V3,

	/* reset */
	input PORESET_n,

	/* board control & interrupt */
	output PWR_FORCE_DISABLE_n,
	output SER2_TX_CFG_RCW_SRC0,
	output SER1_TX_CFG_RCW_SRC1,
	output CPLD_INTERRUPT_CFG_RCW_SRC2,
	output TA_PROG_SFP_n,
	output SPI_FLASH_DISABLE_WP,
	output CARRIER_STBY_n_3V3,
	output PCIE_A_RST_n,
	output PCIE_B_RST_n,
	output PCIE_C_RST_n,

	/* watchdog */
	output WDT_TIME_OUT_n,

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

	/* interrupts inputs */
	input SMB_ALERT_1V8_n,
	input RTC_INT_n,

	/* USB control */
	inout USB3_EN_OC_n,
	input USB3_DRVVBUS,
	output USB3_PWRFAULT_3V3,

	/* PWM */
	output LCD0_BKLT_PWM_3V3,

	/* board config */
	input [5:0] BOARD_CONFIG
);

usbfixer usbfixer (
	.usb_en_oc_n(USB3_EN_OC_n),
	.usb_drvvbus(USB3_DRVVBUS),
	.usb_pwrfault(USB3_PWRFAULT_3V3)
);

wire clk;

reg [1:0] start_cnt = 2'b0;
always @(posedge clk)
	start_cnt <= {start_cnt[0], 1'b1};
wire start_strobe = start_cnt[0] ^ start_cnt[1];

wire pwr_enable;
power_fsm #(
	.LONG_PRESS_DELAY(3'd5)
) power_fsm (
	.clk(clk),
	.ce_1hz(ce_1hz),
	.ce_8hz(ce_8hz),

	.start(start_strobe),
	.initial_pwr_off(1'b0),
	.pwr_btn(power_btn),
	.pwr_enable(pwr_enable)
);
assign PWR_FORCE_DISABLE_n = pwr_enable ? 1'bz : 1'b0;

wire ce_32khz;
wire ce_8hz;
wire ce_1hz;
clockgen clockgen (
	.clk(clk),
	.ce_32khz(ce_32khz),
	.ce_8hz(ce_8hz),
	.ce_1hz(ce_1hz)
);

wire rst0, rst, rst_posedge;
sync_edge sync_edge_poreset (
	.clk(clk),

	.in(~PORESET_n),
	.out(rst),
	.out0(rst0),
	.out_posedge(rst_posedge)
);

reg force_recovery;
initial force_recovery = 1'b0;
always @(posedge clk) begin
	if (rst_posedge)
		force_recovery <= force_recov;
end
wire drive_rcw_src = rst | rst0;

wire [2:0] rcw_src = 3'b010;
wire irq_out;
assign SER2_TX_CFG_RCW_SRC0 = force_recovery ? 1'bz : drive_rcw_src ? rcw_src[0] : 1'bz;
assign SER1_TX_CFG_RCW_SRC1 = force_recovery ? 1'bz : drive_rcw_src ? rcw_src[1] : 1'bz;
assign CPLD_INTERRUPT_CFG_RCW_SRC2 = force_recovery ? 1'bz : drive_rcw_src ? rcw_src[2] : irq_out;

wire i2c_bus_reset_sda_out;
wire i2c_bus_reset_scl_out;
i2c_bus_reset i2c_bus_reset (
	.clk(clk),
	.ce(ce_32khz),

	.sda(I2C_LOCAL_SDA_3V3),
	.sda_out(i2c_bus_reset_sda_out),
	.scl_out(i2c_bus_reset_scl_out),

	.start(rst_posedge)
);

reg healthy_led;
always @(posedge clk) begin
	if (ce_1hz)
		healthy_led <= ~healthy_led;
end
assign HEALTHY_LED_n = healthy_led | ~pwr_enable;

wire [4:0] csr_a;
wire [7:0] csr_di;
wire csr_we;
wire [7:0] csr_do;
wire [7:0] csr_do_cfg_ctrl;
wire [7:0] csr_do_cpld_version;
wire [7:0] csr_do_wdt;
wire [7:0] csr_do_brd_variant;
wire [7:0] csr_do_misc_ctrl;
wire [7:0] csr_do_tacho;
wire [7:0] csr_do_pwm0;
wire [7:0] csr_do_pwm1;
wire [7:0] csr_do_gpio0;
wire [7:0] csr_do_gpio1;
wire [7:0] csr_do_gpo;
wire [7:0] csr_do_gpi;
wire [7:0] csr_do_intc;
assign csr_do = csr_do_cfg_ctrl |
		csr_do_cpld_version |
		csr_do_wdt |
		csr_do_brd_variant |
		csr_do_misc_ctrl |
		csr_do_tacho |
		csr_do_pwm0 |
		csr_do_pwm1 |
		csr_do_gpio0 |
		csr_do_gpio1 |
		csr_do_gpo |
		csr_do_gpi |
		csr_do_intc;

wire i2c_slave_sda_out;
i2c_slave i2c_slave(
	.rst(rst),
	.clk(clk),

	.sda(I2C_LOCAL_SDA_3V3),
	.sda_out(i2c_slave_sda_out),
	.scl(I2C_LOCAL_SCL_3V3),

	.csr_a(csr_a),
	.csr_di(csr_do),
	.csr_we(csr_we),
	.csr_do(csr_di)
);

cfg_ctrl_altera_ufm #(
	.BASE_ADDR(5'h0)
) cfg_ctrl (
	.rst(rst),
	.clk(clk),

	.csr_a(csr_a),
	.csr_di(csr_di),
	.csr_we(csr_we),
	.csr_do(csr_do_cfg_ctrl),

	.start(1'b0),
	.done(),
	.osc(clk)
);

ro_reg #(
	.BASE_ADDR(5'h3)
) cpld_version (
	.csr_a(csr_a),
	.csr_di(csr_di),
	.csr_we(csr_we),
	.csr_do(csr_do_cpld_version),

	.in(CPLD_VERSION)
);

wire [1:0] wdt_out;
wire wdt_irq;
watchdog #(
	.BASE_ADDR(5'h4),
	.DFL_TIMEOUT(8'h06),
	.DFL_OE(2'b01)
) watchdog (
	.rst(rst),
	.clk(clk),
	.ce(ce_1hz),

	.csr_a(csr_a),
	.csr_di(csr_di),
	.csr_we(csr_we),
	.csr_do(csr_do_wdt),

	.wdt_out(wdt_out),
	.force_recovery_mode(),
	.irq(wdt_irq)
);
assign WDT_TIME_OUT_n = ~wdt_out[1];

ro_reg #(
	.BASE_ADDR(5'h9)
) brd_variant (
	.csr_a(csr_a),
	.csr_di(csr_di),
	.csr_we(csr_we),
	.csr_do(csr_do_brd_variant),

	.in({2'b0, BOARD_CONFIG})
);

wire [5:0] misc_out;
gpo #(
	.BASE_ADDR(5'ha),
	.NUM_GPIOS(6)
) misc_ctrl (
	.rst(rst),
	.clk(clk),

	.csr_a(csr_a),
	.csr_di(csr_di),
	.csr_we(csr_we),
	.csr_do(csr_do_misc_ctrl),

	.out(misc_out)
);
wire ta_prog_sfp = misc_out[0];
wire spi_flash_disable_wp = misc_out[1];
wire carrier_standby = misc_out[2];
wire pcie_a_rst = rst | misc_out[3];
wire pcie_b_rst = rst | misc_out[4];
wire pcie_c_rst = rst | misc_out[5];

assign TA_PROG_SFP_n = ta_prog_sfp ? 1'b0 : 1'bz;
assign SPI_FLASH_DISABLE_WP = spi_flash_disable_wp ? 1'b1 : 1'bz;
assign CARRIER_STBY_n_3V3 = carrier_standby ? 1'bz : 1'b1;
assign PCIE_A_RST_n = ~pcie_a_rst;
assign PCIE_B_RST_n = ~pcie_b_rst;
assign PCIE_C_RST_n = ~pcie_c_rst;

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
	.DFL_STATE(6'b110001)
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

wire sleep;
wire sleep_negedge;
sync_edge sync_edge_sleep (
	.clk(clk),

	.in(SLEEP_n),
	.out(sleep),
	.out_negedge(sleep_negedge)
);

wire power_btn;
wire power_btn_edge;
sync_edge sync_edge_power_btn (
	.clk(clk),

	.in(~POWER_BTN_n),
	.out(power_btn),
	.out_edge(power_btn_edge)
);

wire rtc_int_negedge;
sync_edge sync_edge_rtc_int (
	.clk(clk),

	.in(RTC_INT_n),
	.out_negedge(rtc_int_negedge)
);

wire smb_alert_negedge;
sync_edge sync_edge_smb_alert (
	.clk(clk),

	.in(SMB_ALERT_1V8_n),
	.out_negedge(smb_alert_negedge)
);

wire charger_prsnt;
sync_edge sync_edge_charger_prsnt (
	.clk(clk),

	.in(~CHARGER_PRSNT_n),
	.out(charger_prsnt)
);

wire charging;
sync_edge sync_edge_charging (
	.clk(clk),

	.in(~CHARGING_n),
	.out(charging)
);

wire lid;
sync_edge sync_edge_lid (
	.clk(clk),

	.in(~LID_n),
	.out(lid)
);

wire batlow;
sync_edge sync_edge_batlow (
	.clk(clk),

	.in(~BATLOW_n),
	.out(batlow)
);

wire force_recov;
sync_edge sync_edge_force_recov (
	.clk(clk),

	.in(~FORCE_RECOV_n),
	.out(force_recov)
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
		charger_prsnt,
		charging,
		lid,
		batlow,
		sleep,
		force_recov,
		power_btn
	})
);

wire intc_irq;
intc #(
	.BASE_ADDR(5'h1c),
	.NUM_INTS(7)
) intc (
	.rst(rst),
	.clk(clk),

	.csr_a(csr_a),
	.csr_di(csr_di),
	.csr_we(csr_we),
	.csr_do(csr_do_intc),

	.int({
		wdt_irq,
		sleep_negedge,
		power_btn_edge,
		1'b0, /* ESPI_ALERT1_n not supported */
		1'b0, /* ESPI_ALERT0_n not supported */
		smb_alert_negedge,
		rtc_int_negedge
	}),
	.irq(intc_irq)
);

wire sda_out = i2c_bus_reset_sda_out & i2c_slave_sda_out;
wire scl_out = i2c_bus_reset_scl_out;
assign I2C_LOCAL_SDA_3V3 = sda_out ? 1'bz : 1'b0;
assign I2C_LOCAL_SCL_3V3 = scl_out ? 1'bz : 1'b0;

assign irq_out = intc_irq | gpio0_irq | gpio1_irq;

endmodule
