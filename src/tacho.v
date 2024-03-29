// SPDX-License-Identifier: CERN-OHL-P-2.0
/*
 * Simple tacho input module.
 *
 * Provides two ranges which are automatically switched based on the input
 * RPM. They have different resolutions.
 *
 * Copyright (c) 2020-2022 Michael Walle <michael@walle.cc>
 */

module tacho #(
	parameter BASE_ADDR = 5'h0
) (
	input rst,
	input clk,

	input [4:0] csr_a,
	input [7:0] csr_di,
	input csr_we,
	output [7:0] csr_do,

	input ce_1hz,
	input tacho_in
);

wire tacho_negedge;
sync_edge sync_edge_tacho_sleep (
	.clk(clk),

	.in(tacho_in),
	.out(),
	.out0(),
	.out_edge(),
	.out_negedge(tacho_negedge),
	.out_posedge()
);

reg [9:0] counter;
reg [9:0] counter0;
always @(posedge clk) begin
	if (rst) begin
		counter <= 10'd0;
		counter0 <= 10'd0;
	end else begin
		if (tacho_negedge)
			counter <= counter + 10'd1;

		if (ce_1hz) begin
			counter0 <= counter;
			counter <= 10'd0;
		end
	end
end

wire tacho_scale = counter0[9] | counter0[8] | counter[7];
wire [6:0] tacho_count = (tacho_scale) ? counter0[9:3] : counter0[6:0];
wire [7:0] data_out = {tacho_scale, tacho_count};
assign csr_do = (csr_a == BASE_ADDR) ? data_out : 8'b0;

endmodule
