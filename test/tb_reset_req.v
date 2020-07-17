`timescale 1ns/1ns
module tb_reset_req();

`include "sys.v"
`include "slowclock.v"

reg start;
reg pwr_btn;
reg initial_pwr_off;

reg reset_req;
wire reset_req_n;
wire reset_reg_out;
reset_req #(
	.EXTEND_COUNT(3'd5)
) dut (
	.clk(clk),
	.ce(slowclk_ce),

	.reset_req_n(reset_req_n),
	.reset_req_out(reset_req_out)
);
assign reset_req_n = (reset_req_out | reset_req) ? 1'b0 : 1'b1;

initial begin
	reset_req = 1'b0;
	#10000

	reset_req = 1'b1;
	#10
	reset_req = 1'b0;

	#1000000
	$finish;
end

initial $dumpfile("reset_req.lx2");
initial $dumpvars(0, dut);

endmodule
