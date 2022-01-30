// SPDX-License-Identifier: CERN-OHL-P-2.0
/*
 * sl28 top module
 *
 * Copyright (c) 2020-2022 Michael Walle <michael@walle.cc>
 */

module sl28_top #(
	parameter CPLD_VERSION = 8'hff
) (
	output HEALTHY_LED_n,

	/* I2C bus */
	inout I2C_LOCAL_SDA_3V3,
	inout I2C_LOCAL_SCL_3V3,

	/* reset */
	input PORESET_n,
	inout RESET_REQ_n,
	output HRESET_n,

	/* peripheral resets */
	output RESET_OUT_3V3_n,
	output PCIE_A_RST_n,
	output PCIE_B_RST_n,
	output PCIE_C_RST_n,
	output USB2517_RST_n,
	output PTN3460_PD_n,
	output PTN3460_RST_n,
	output SDIO_PWR_EN,
	output EMMC_RST_n,
	output GBE_FORCE_RST_n,
	output ESPI_RESET_3V3_n,

	/* board control & interrupt */
	output PWR_FORCE_DISABLE_n,
	output SER2_TX_CFG_RCW_SRC0,
	output SER1_TX_CFG_RCW_SRC1,
	inout CPLD_INTERRUPT_CFG_RCW_SRC2,
	output TA_PROG_SFP_n,
	output SPI_FLASH_DISABLE_WP,
	output CARRIER_STBY_n_3V3,
	output I2C_SDA_5P49V6967_SEL0,
	output I2C_SDA_5P49V6967_SEL1,
	output CLKGEN_5P49V6967_OEA_n,
	output CLKGEN_5P49V6967_OEB_n,

	/* SMARC boot selection */
	input BOOT_SEL0_n,
	input BOOT_SEL1_n,
	input BOOT_SEL2_n,

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
	input WOL_INT_GBE_n,
	input PCIE_WAKE_n,

	/* USB control */
	inout USB3_EN_OC_n,
	input USB3_DRVVBUS,
	output USB3_PWRFAULT_3V3,

	/* PWM */
	output LCD0_BKLT_PWM_3V3,

	/* I2C fixer */
	output I2C_GP_SCL,
	inout I2C_GP_SDA,
	output I2C_PM_SCL,
	inout I2C_PM_SDA,

	/* board config */
	inout [3:0] BOARD_CONFIG
);

wire usbfixer_disabled;
usbfixer usbfixer (
	.enable(!usbfixer_disabled),
	.usb_en_oc_n(USB3_EN_OC_n),
	.usb_drvvbus(USB3_DRVVBUS),
	.usb_pwrfault(USB3_PWRFAULT_3V3)
);

wire clk;

wire wol_int_negedge;
sync_edge sync_edge_wol_int (
	.clk(clk),

	.in(WOL_INT_GBE_n),
	.out(),
	.out0(),
	.out_edge(),
	.out_negedge(wol_int_negedge),
	.out_posedge()
);

wire pcie_wake_negedge;
sync_edge sync_edge_pcie_wake (
	.clk(clk),

	.in(PCIE_WAKE_n),
	.out(),
	.out0(),
	.out_edge(),
	.out_negedge(pcie_wake_negedge),
	.out_posedge()
);

wire initial_pwr_off;
wire power_on;
wire power_off;
wire pwr_enable;
wire cfg_read_done;
power_fsm #(
	.LONG_PRESS_DELAY(3'd5)
) power_fsm (
	.clk(clk),
	.ce_1hz(ce_1hz),
	.ce_8hz(ce_8hz),

	.start(cfg_read_done),
	.initial_pwr_off(initial_pwr_off),
	.pwr_on(power_on),
	.pwr_off(power_off),
	.pwr_btn(power_btn),
	.pwr_enable(pwr_enable)
);
assign PWR_FORCE_DISABLE_n = pwr_enable ? 1'bz : 1'b0;
assign power_off = power_off_by_reg | power_off_by_reset;
assign power_on = rtc_int_negedge | wol_int_negedge | pcie_wake_negedge;

reg is_power_off_req;
always @(posedge clk) begin
	if (rst)
		is_power_off_req <= 1'b0;
	else if (!irq_out && !drive_rcw_src & !failsafe_mode)
		is_power_off_req <= ~CPLD_INTERRUPT_CFG_RCW_SRC2;
end

wire reset_req_out;
reset_req #(
	.EXTEND_COUNT(3'd5)
) reset_req (
	.clk(clk),
	.ce(ce_8hz),

	.reset_req_n(RESET_REQ_n),
	.enable(!is_power_off_req),
	.reset_req_out(reset_req_out)
);

wire cpu_reset;
/*
 * The RESET_REQ# output of the SoC is a push-pull output driver. Due to
 * the wiring on the board, we cannot win if we pull the line low. As
 * a workaround, pull HRESET# low, too. This will make the SoC release
 * the RESET_REQ# line.
 */
assign RESET_REQ_n = (reset_req_out | cpu_reset) ? 1'b0 : 1'bz;
assign HRESET_n = (reset_req_out | cpu_reset) ? 1'b0 : 1'bz;

wire power_off_by_reset;
async_negedge_latch_sync_out power_off_latch (
	.clk(clk),

	.in(RESET_REQ_n),
	.enable(is_power_off_req),
	.clear(power_off_by_reset),
	.async_out(),
	.sync_out(),
	.sync_out_edge(power_off_by_reset)
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

wire rst0, rst, rst_posedge;
sync_edge sync_edge_poreset (
	.clk(clk),

	.in(~PORESET_n),
	.out(rst),
	.out0(rst0),
	.out_edge(),
	.out_posedge(rst_posedge),
	.out_negedge()
);
assign ESPI_RESET_3V3_n = ~rst;
assign RESET_OUT_3V3_n = ~rst;

reg failsafe_mode;
reg latched_force_recovery;
initial failsafe_mode = 1'b0;
initial latched_force_recovery = 1'b0;
always @(posedge clk) begin
	if (rst_posedge) begin
		failsafe_mode <= force_recovery | wdt_failsafe_mode;
		latched_force_recovery <= force_recovery;
	end
end
wire drive_rcw_src = pwr_enable & (rst | rst0);

wire irq_out;
wire emmc_boot;
wire [2:0] rcw_src = emmc_boot ? 3'b001 : 3'b010;
assign SER2_TX_CFG_RCW_SRC0 = failsafe_mode ? 1'bz :
			      drive_rcw_src ? rcw_src[0] :
			      1'bz;
assign SER1_TX_CFG_RCW_SRC1 = failsafe_mode ? 1'bz :
			      drive_rcw_src ? rcw_src[1] :
			      1'bz;

/*
 * CPLD_INTERRUPT_CFG_RCW_SRC2 is bidirectional. It is either an interrupt
 * pulse driven by the CPLD or an input to signal an poweroff driven by the
 * SoC. Therefore, this needs to be an open drain output. But this is not
 * possible if FORCE_RECOV# is asserted, because in this case the pull-up
 * resistor on this signal is disabled. Thus, the power-off request will
 * not work in recovery mode, because we need to be push-pull.
 */
assign CPLD_INTERRUPT_CFG_RCW_SRC2 = drive_rcw_src ? (failsafe_mode ? 1'bz : rcw_src[2]) :
				     force_recovery ? ~irq_out :
				     irq_out ? 1'b0 : 1'bz;

wire i2c_bus_reset_sda_out;
wire i2c_bus_reset_scl_out;
i2c_bus_reset i2c_local_bus_reset (
	.clk(clk),
	.ce(ce_32khz),

	.sda(I2C_LOCAL_SDA_3V3),
	.sda_out(i2c_bus_reset_sda_out),
	.scl_out(i2c_bus_reset_scl_out),

	.start(rst_posedge)
);

wire i2c_gp_sda_out;
wire i2c_gp_scl_out;
i2c_bus_reset i2c_bus_gp_reset (
	.clk(clk),
	.ce(ce_32khz),

	.sda(I2C_GP_SDA),
	.sda_out(i2c_gp_sda_out),
	.scl_out(i2c_gp_scl_out),

	.start(rst_posedge)
);
assign I2C_GP_SDA = (!i2c_gp_sda_out) ? 1'b0 : 1'bz;
assign I2C_GP_SCL = (!i2c_gp_scl_out) ? 1'b0 : 1'bz;

wire i2c_pm_sda_out;
wire i2c_pm_scl_out;
i2c_bus_reset i2c_bus_pm_reset (
	.clk(clk),
	.ce(ce_32khz),

	.sda(I2C_PM_SDA),
	.sda_out(i2c_pm_sda_out),
	.scl_out(i2c_pm_scl_out),

	.start(rst_posedge)
);
assign I2C_PM_SDA = (!i2c_pm_sda_out) ? 1'b0 : 1'bz;
assign I2C_PM_SCL = (!i2c_pm_scl_out) ? 1'b0 : 1'bz;

reg healthy_led;
wire healthy_led_ce = failsafe_mode ? ce_8hz : ce_1hz;
always @(posedge clk) begin
	if (healthy_led_ce)
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
wire [7:0] csr_do_brd_control;
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
		csr_do_brd_control |
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
wire [2:0] csr_a_unused;
i2c_slave i2c_slave(
	.rst(rst),
	.clk(clk),

	.sda(I2C_LOCAL_SDA_3V3),
	.sda_out(i2c_slave_sda_out),
	.scl(I2C_LOCAL_SCL_3V3),

	.csr_a({csr_a_unused, csr_a}),
	.csr_di(csr_do),
	.csr_we(csr_we),
	.csr_do(csr_di)
);

wire [15:0] cfg;
cfg_ctrl_altera_ufm #(
	.BASE_ADDR(5'h0)
) cfg_ctrl (
	.rst(rst),
	.clk(clk),

	.csr_a(csr_a),
	.csr_di(csr_di),
	.csr_we(csr_we),
	.csr_do(csr_do_cfg_ctrl),

	.start(rst_posedge),
	.failsafe_mode(failsafe_mode),
	.done(cfg_read_done),
	.cfg(cfg),
	.osc(clk)
);

assign initial_pwr_off = ~cfg[0];
assign emmc_boot = ~cfg[1];
wire watchdog_enabled = ~cfg[2];
wire failsafe_watchdog_disabled = ~cfg[3];
assign I2C_SDA_5P49V6967_SEL0 = ~cfg[4] ? 1'b1 : 1'bz;
assign I2C_SDA_5P49V6967_SEL1 = ~cfg[5] ? 1'b1 : 1'bz;
assign CLKGEN_5P49V6967_OEA_n = ~cfg[6] ? 1'b1 : 1'bz;
assign CLKGEN_5P49V6967_OEB_n = ~cfg[7] ? 1'b1 : 1'bz;
wire keep_gbe_phy_in_reset = ~cfg[8];
wire keep_usb2517_in_reset = ~cfg[9];
wire keep_ptn3460_in_reset = ~cfg[10];
assign usbfixer_disabled = ~cfg[11];
wire gbe_phy_reset_enabled = ~cfg[12];

gpi #(
	.BASE_ADDR(5'h3)
) cpld_version (
	.rst(rst),
	.clk(clk),

	.csr_a(csr_a),
	.csr_di(csr_di),
	.csr_we(csr_we),
	.csr_do(csr_do_cpld_version),

	.in(CPLD_VERSION)
);

wire [1:0] wdt_out;
wire [1:0] wdt_out_strobe;
wire wdt_irq;
wire wdt_failsafe_mode;
wire failsafe_watchdog_en_default = ~failsafe_watchdog_disabled & ~latched_force_recovery;
watchdog #(
	.BASE_ADDR(5'h4),
	.DEFAULT_TIMEOUT(8'h08),
	.DEFAULT_OE(2'b01)
) watchdog (
	.rst(rst),
	.clk(clk),
	.ce(ce_1hz),
	.pwr_is_off(~pwr_enable),

	.csr_a(csr_a),
	.csr_di(csr_di),
	.csr_we(csr_we),
	.csr_do(csr_do_wdt),

	.wdt_en_default({failsafe_watchdog_en_default, watchdog_enabled}),
	.wdt_out(wdt_out),
	.wdt_out_strobe(wdt_out_strobe),
	.failsafe_mode(wdt_failsafe_mode),
	.irq(wdt_irq)
);
assign cpu_reset = wdt_out_strobe[0];
assign WDT_TIME_OUT_n = ~wdt_out[1];

gpi #(
	.BASE_ADDR(5'h8)
) board_control (
	.rst(rst),
	.clk(clk),

	.csr_a(csr_a),
	.csr_di(csr_di),
	.csr_we(csr_we),
	.csr_do(csr_do_brd_control),

	.in({1'b0, BOOT_SEL2_n, BOOT_SEL1_n, BOOT_SEL0_n, 4'b0000})
);
wire power_off_by_reg = (csr_we && csr_a == 5'h8 && csr_di[0]);

config_trits #(
	.BASE_ADDR(5'h9)
) config_trits (
	.rst(rst),
	.clk(clk),
	.ce(ce_32khz),

	.csr_a(csr_a),
	.csr_di(csr_di),
	.csr_we(csr_we),
	.csr_do(csr_do_brd_variant),

	.trits(BOARD_CONFIG)
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
wire carrier_standby = ~rst & misc_out[2];
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

	.pwm_en(),
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

wire [3:0] gpio1_out;
wire [3:0] gpio1_in;
wire [3:0] gpio1_oe;
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

wire ptn3460_powerdown;
wire ptn3460_reset;
wire emmc_reset;
wire sdio_pwr_enable;
gpo #(
	.BASE_ADDR(5'h1a),
	.NUM_GPIOS(6),
	.DFL_STATE(6'b100000)
) gpo (
	.rst(rst),
	.clk(clk),

	.csr_a(csr_a),
	.csr_di(csr_di),
	.csr_we(csr_we),
	.csr_do(csr_do_gpo),

	.out({
		sdio_pwr_enable,
		ptn3460_powerdown,
		ptn3460_reset,
		emmc_reset,
		LCD0_BKLT_EN_3V3,
		LCD0_VDD_EN_3V3
	})
);

assign PTN3460_RST_n = (rst | ptn3460_reset | keep_ptn3460_in_reset) ? 1'b0 : 1'bz;
assign PTN3460_PD_n = (rst | ptn3460_powerdown | keep_ptn3460_in_reset) ? 1'b0 : 1'bz;
assign EMMC_RST_n = (rst | emmc_reset) ? 1'b0 : 1'bz;
assign SDIO_PWR_EN = (rst | ~sdio_pwr_enable) ? 1'b0 : 1'bz;
assign USB2517_RST_n = (rst | keep_usb2517_in_reset) ? 1'b0 : 1'bz;
assign GBE_FORCE_RST_n = ((rst & gbe_phy_reset_enabled) | keep_gbe_phy_in_reset) ? 1'b0 : 1'bz;

wire sleep;
wire sleep_negedge;
sync_edge sync_edge_sleep (
	.clk(clk),

	.in(SLEEP_n),
	.out(sleep),
	.out0(),
	.out_edge(),
	.out_negedge(sleep_negedge),
	.out_posedge()
);

wire power_btn;
wire power_btn_edge;
sync_edge sync_edge_power_btn (
	.clk(clk),

	.in(~POWER_BTN_n),
	.out(power_btn),
	.out0(),
	.out_edge(power_btn_edge),
	.out_negedge(),
	.out_posedge()
);

wire rtc_int_negedge;
sync_edge sync_edge_rtc_int (
	.clk(clk),

	.in(RTC_INT_n),
	.out(),
	.out0(),
	.out_edge(),
	.out_negedge(rtc_int_negedge),
	.out_posedge()
);

wire smb_alert_negedge;
sync_edge sync_edge_smb_alert (
	.clk(clk),

	.in(SMB_ALERT_1V8_n),
	.out(),
	.out0(),
	.out_edge(),
	.out_negedge(smb_alert_negedge),
	.out_posedge()
);

wire charger_prsnt;
sync_edge sync_edge_charger_prsnt (
	.clk(clk),

	.in(CHARGER_PRSNT_n),
	.out(charger_prsnt),
	.out0(),
	.out_edge(),
	.out_negedge(),
	.out_posedge()
);

wire charging;
sync_edge sync_edge_charging (
	.clk(clk),

	.in(CHARGING_n),
	.out(charging),
	.out0(),
	.out_edge(),
	.out_negedge(),
	.out_posedge()
);

wire lid;
sync_edge sync_edge_lid (
	.clk(clk),

	.in(LID_n),
	.out(lid),
	.out0(),
	.out_edge(),
	.out_negedge(),
	.out_posedge()
);

wire batlow;
sync_edge sync_edge_batlow (
	.clk(clk),

	.in(BATLOW_n),
	.out(batlow),
	.out0(),
	.out_edge(),
	.out_negedge(),
	.out_posedge()
);

wire force_recovery;
sync_edge sync_edge_force_recov (
	.clk(clk),

	.in(~FORCE_RECOV_n),
	.out(force_recovery),
	.out0(),
	.out_edge(),
	.out_negedge(),
	.out_posedge()
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
		~force_recovery,
		~power_btn
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

	.irqs_in({
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
