`timescale 1ns/1ns
module tb_i2c_bus_reset();

`include "sys.v"
`include "slowclock.v"

reg sda;
reg start;

i2c_bus_reset dut(
	.clk(clk),
	.ce(slowclk_ce),

	.sda(sda),
	.sda_out(),
	.scl_out(),

	.start(start)
);

initial begin
	waitreset;
	sda = 1'b0;
	start = 1'b1;
	waitclock;
	start = 1'b0;
	#1000000;
	sda = 1'b1;
	#1000000;
	$finish;
end

initial $dumpfile("i2c_bus_reset.lx2");
initial $dumpvars(0, dut);

endmodule
