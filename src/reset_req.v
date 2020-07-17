module reset_req #(
	parameter EXTEND_COUNT = 3'h0
) (
	input clk,
	input ce,

	input reset_req_n,
	output reg reset_req_out
);

reg clear;
initial reset_req_out = 1'b0;
always @(negedge reset_req_n, posedge clear) begin
	if (clear)
		reset_req_out <= 1'b0;
	else
		reset_req_out <= 1'b1;
end

wire reset_req_s;
sync_edge sync_edge_reset_req (
	.clk(clk),

	.in(reset_req_out),
	.out(reset_req_s)
);

reg [2:0] reset_req_cnt;
always @(posedge clk) begin
	clear <= 1'b0;
	if (!reset_req_s)
		reset_req_cnt <= 3'b0;
	else if (reset_req_cnt == EXTEND_COUNT)
		clear <= 1'b1;
	else if (ce)
		reset_req_cnt = reset_req_cnt + 3'd1;
end

endmodule
