`timescale 1ns/1ns
module tb_i2c_slave();

`include "sys.v"

wire sda_i;
wire [4:0] csr_a;
reg [7:0] csr_di;
wire csr_we;
wire [7:0] csr_do;
reg scl;

i2c_slave dut(
	.rst(rst),
	.clk(clk),
	.sda(sda_i),
	.sda_out(sda_out),
	.scl(scl),
	.csr_a(csr_a),
	.csr_di(csr_di),
	.csr_we(csr_we),
	.csr_do(csr_do)
);

reg sda;
assign sda_i = sda & sda_out;

task i2c_start;
begin
	#1000;
	scl = 1'b1;
	#1000;
	sda = 1'b0;
	#1000;
	scl = 1'b0;
	#1000;
end
endtask

task i2c_stop;
begin
	#1000;
	sda = 1'b0;
	#1000;
	scl = 1'b1;
	#1000;
	sda = 1'b1;
	#1000;
end
endtask

integer i;
task i2c_write;
	input [8:0] data;
begin

	for (i = 8; i >= 0; i = i - 1) begin
		sda = data[i];
		scl = 1'b0;
		#1000;
		scl = 1'b1;
		#1000;
		scl = 1'b0;
		#250;
	end
end
endtask

initial begin
	sda = 1'b1;
	scl = 1'b1;
	waitreset;

	#1000;

	i2c_start;
	i2c_write({7'h4a, 1'b0, 1'b1});
	i2c_write({8'h1f, 1'b1});
	i2c_write({8'haa, 1'b1});
	i2c_write({8'h55, 1'b1});
	i2c_stop;

	i2c_start;
	i2c_write({7'h4b, 1'b0, 1'b1});
	i2c_write({8'h1f, 1'b1});
	i2c_write({8'haa, 1'b1});
	i2c_stop;


	csr_di = 8'h83;

	i2c_start;
	i2c_write({7'h4a, 1'b0, 1'b1});
	i2c_write({8'h10, 1'b1});
	i2c_start;
	i2c_write({7'h4a, 1'b1, 1'b1});
	i2c_write({8'hff, 1'b0});
	i2c_write({8'hff, 1'b0});
	i2c_write({8'hff, 1'b1});
	i2c_write({8'hff, 1'b1});
	i2c_stop;

	$finish;
end

initial $dumpfile("i2c_slave.lx2");
initial $dumpvars(0, dut);

endmodule
