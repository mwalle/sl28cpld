`timescale 1ns/1ns
module tb_cfg_ctrl_alter_ufm();

`include "sys.v"
`include "csr.v"

reg start;
cfg_ctrl_altera_ufm dut (
	.rst(rst),
	.clk(clk),

	.csr_a(csr_a),
	.csr_di(csr_di),
	.csr_we(csr_we),
	.csr_do(csr_do),

	.start(start),
	.done(),
	.osc()
);

initial begin
	start = 1'b0;
	waitreset;
	waitclock;
	start = 1'b1;
	waitclock;
	start = 1'b0;

	#20000;

	$finish;
end

initial $dumpfile("cfg_ctrl_altera_ufm.lx2");
initial $dumpvars(0, dut);

endmodule

module altufm_none (
	input arclk,
	input ardin,
	input arshft,
	input drclk,
	input drdin,
	input drshft,
	input erase,
	input oscena,
	input program,

	output busy,
	output drdout,
	output osc
);

`include "sys.v"

assign osc = clk;
assign drdout = 1'b1;
assign busy = 1'b0;

endmodule
