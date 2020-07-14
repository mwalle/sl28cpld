`timescale 1ns/1ns
module tb_watchdog();

`include "sys.v"
`include "csr.v"

wire ce;

watchdog #(
	.DFL_TIMEOUT(8'h3)
) dut(
	.rst(rst),
	.clk(clk),

	.csr_a(csr_a),
	.csr_di(csr_di),
	.csr_we(csr_we),
	.csr_do(csr_do),

	.wdt_ce(ce)
);

reg [5:0] slowclk_cnt;
assign ce = slowclk_cnt == 6'h20;

always @(posedge clk) begin
	if (rst)
		slowclk_cnt <= 6'b0;
	else
		slowclk_cnt <= slowclk_cnt + 6'd1;
end

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
