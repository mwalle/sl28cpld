`timescale 1ns/1ns
module tb_top();

`include "sys.v"

wire PORESET_n;
reg FORCE_RECOV_n;
wire CPLD_INTERRUPT_CFG_RCW_SRC2;
wire RESET_REQ_n;
reg rstn;

sl28_top dut(
	.RESET_REQ_n,
	.PORESET_n,
	.FORCE_RECOV_n,
	.CPLD_INTERRUPT_CFG_RCW_SRC2
);

assign (weak1, weak0) CPLD_INTERRUPT_CFG_RCW_SRC2 = 1'b1;
assign (weak1, weak0) RESET_REQ_n = rstn;

assign #10 PORESET_n = RESET_REQ_n;

initial begin
	rstn = 1'b0;
	FORCE_RECOV_n = 1'b1;
	// wait until the UFM config bits are loaded
	#10000

	// normal mode
	rstn = 1'b1;
	#100000

	// recovery mode
	FORCE_RECOV_n = 1'b0;
	rstn = 1'b0;
	#500
	rstn = 1'b1;
	#100000

	$finish;
end

initial $dumpfile("top.lx2");
initial $dumpvars(0, dut);

endmodule
