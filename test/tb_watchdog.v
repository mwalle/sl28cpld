`timescale 1ns/1ns
module tb_watchdog();

`include "sys.v"
`include "csr.v"
`include "slowclock.v"

watchdog #(
	.DEFAULT_TIMEOUT(8'h3)
) dut(
	.rst(rst),
	.clk(clk),
	.ce(slowclk_ce),

	.csr_a(csr_a),
	.csr_di(csr_di),
	.csr_we(csr_we),
	.csr_do(csr_do),

	.wdt_out(),
	.force_recovery_mode(),
	.irq()
);

initial begin
	waitreset;

	csrread(5'h0);
	csrwrite(5'h0, 8'h41);
	#5000000

	$finish;
end

initial $dumpfile("watchdog.lx2");
initial $dumpvars(0, dut);

endmodule
