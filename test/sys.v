reg rst;
reg clk;

initial clk = 1'b0;
always #108 clk = ~clk;

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

initial begin
	rst = 1'b1;
	waitclock;
	rst = 1'b0;
end
