// SPDX-License-Identifier: CERN-OHL-P-2.0
/*
 * Board specific reset handling.
 *
 * There is a need to extend an incoming reset pulse to meet the SoC reset
 * input requirements.
 *
 * Copyright (c) 2020-2022 Michael Walle <michael@walle.cc>
 */

module reset_req #(
	parameter EXTEND_COUNT = 3'h0
) (
	input clk,
	input ce,

	input reset_req_n,
	input enable,
	output reset_req_out
);

wire reset_req_sync;
async_negedge_latch_sync_out latch (
	.clk(clk),

	.in(reset_req_n),
	.enable(enable),
	.clear(clear),
	.async_out(reset_req_out),
	.sync_out(reset_req_sync),
	.sync_out_edge()
);

reg clear;
reg [2:0] reset_req_cnt;
always @(posedge clk) begin
	clear <= 1'b0;
	if (!reset_req_sync)
		reset_req_cnt <= 3'b0;
	else if (reset_req_cnt == EXTEND_COUNT)
		clear <= 1'b1;
	else if (ce)
		reset_req_cnt <= reset_req_cnt + 3'd1;
end

endmodule
