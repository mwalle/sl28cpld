`timescale 1ns/1ns
module tb_clockgen();

reg clk;
initial clk = 1'b0;
always #108 clk = ~clk;

clockgen dut(
	.clk(clk)
);

initial dut.cnt_32khz = 0;
initial dut.cnt_8hz = 0;
initial dut.cnt_1hz = 0;

always #2000000000 $finish;

initial $dumpfile("clockgen.lx2");
initial $dumpvars(0, dut);

endmodule
