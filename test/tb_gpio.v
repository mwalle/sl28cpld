`timescale 1ns/1ns
module tb_gpio();

`include "sys.v"
`include "csr.v"

wire [7:0] out;
wire [7:0] oe;
reg [7:0] in;

gpio dut(
	.rst(rst),
	.clk(clk),

	.csr_a(csr_a),
	.csr_di(csr_di),
	.csr_we(csr_we),
	.csr_do(csr_do),

	.in(in),
	.out(out),
	.oe(oe),
	.irq()
);

initial begin
	in = 8'h00;
	waitreset;

	csrread(5'h0);
	csrwrite(5'h1, 8'haa);
	csrwrite(5'h0, 8'h0f);
	csrwrite(5'h3, 8'h10);
	in = 8'h20;
	#5000
	in = 8'h30;
	#5000
	in = 8'h00;
	csrread(5'h0);
	csrread(5'h2);

	csrread(5'h4);
	csrwrite(5'h4, 8'h10);
	csrread(5'h4);

	$finish;
end

initial $dumpfile("gpio.lx2");
initial $dumpvars(0, dut);

endmodule
