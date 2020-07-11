`timescale 1ns/1ns
module tb_pwm();

reg clk;
reg rst;
wire pwm_ce;
reg [4:0] csr_a;
reg [7:0] csr_di;
reg csr_we;
wire [7:0] csr_do;
wire pwm_out;

pwm dut(
	.clk(clk),
	.rst(rst),
	.pwm_ce(pwm_ce),
	.csr_a(csr_a),
	.csr_di(csr_di),
	.csr_we(csr_we),
	.csr_do(csr_do),
	.pwm_out(pwm_out)
);

initial clk = 1'b0;
always #100 clk = ~clk;

reg [6:0] pwm_clk_counter;
assign pwm_ce = (pwm_clk_counter[6] & pwm_clk_counter[3]); /* 0x48 */

always @(posedge clk) begin
	if (rst | pwm_ce)
		pwm_clk_counter <= 7'b0;
	else
		pwm_clk_counter <= pwm_clk_counter + 1;
end

task waitclock;
begin
	@(posedge clk)
	#1;
end
endtask

task waitreset;
begin
	@(negedge rst)
	#1;
end
endtask

task csrwrite;
	input [4:0] address;
	input [7:0] data;
begin
	csr_a = address;
	csr_di = data;
	csr_we = 1'b1;
	waitclock;
	csr_we = 1'b0;
end
endtask

initial begin
	rst = 1'b1;
	waitclock;
	rst = 1'b0;
end

initial begin
	waitreset;
	csrwrite(0, 8'h80);
	csrwrite(1, 8'h00);
	#50000
	csrwrite(1, 8'h01);
	#50000
	csrwrite(1, 8'h02);
	#50000
	csrwrite(0, 8'h80);
	csrwrite(1, 8'h1f);
	#50000
	csrwrite(0, 8'h81);
	#50000
	csrwrite(1, 8'h40);
	#50000

	$finish;
end

initial $dumpfile("pwm.lx2");
initial $dumpvars(0, dut);

endmodule
