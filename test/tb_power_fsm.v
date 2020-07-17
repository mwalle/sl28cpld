`timescale 1ns/1ns
module tb_power_fsm();

`include "sys.v"
`include "slowclock.v"

reg start;
reg pwr_btn;
reg initial_pwr_off;

power_fsm #(
	.LONG_PRESS_DELAY(3'd5)
) dut (
	.clk(clk),
	.ce_1hz(slowclk_ce),
	.ce_8hz(slowclk_ce),

	.start(start),
	.initial_pwr_off(initial_pwr_off),
	.pwr_btn(pwr_btn),
	.pwr_enable()
);

initial begin
	start = 1'b0;
	pwr_btn = 1'b0;
	initial_pwr_off = 2'b0;
	waitreset;

	waitclock;
	start = 1'b1;
	waitclock;
	start = 1'b0;

	#2000;

	pwr_btn = 1'b1;
	#50000;

	pwr_btn = 1'b0;
	#2000;

	pwr_btn = 1'b1;
	#70000;

	pwr_btn = 1'b0;
	#100000;

	pwr_btn = 1'b1;
	waitclock;
	#5000;

	$finish;
end

initial $dumpfile("power_fsm.lx2");
initial $dumpvars(0, dut);

endmodule
