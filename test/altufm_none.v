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
