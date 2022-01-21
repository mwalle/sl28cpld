module async_negedge_latch_sync_out (
	input clk,

	input in,
	input enable,
	input clear,
	output reg async_out,
	output sync_out,
	output sync_out_edge
);

initial async_out = 1'b0;
always @(negedge in, posedge clear) begin
	if (clear)
		async_out <= 1'b0;
	else if (enable)
		async_out <= 1'b1;
end

sync_edge sync_edge_latch (
	.clk(clk),

	.in(async_out),
	.out(sync_out),
	.out0(),
	.out_edge(),
	.out_negedge(),
	.out_posedge(sync_out_edge)
);

endmodule
