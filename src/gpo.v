// SPDX-License-Identifier: CERN-OHL-P-2.0
/*
 * Simple GPO (output only) module.
 *
 * Copyright (c) 2020-2022 Michael Walle <michael@walle.cc>
 */

module gpo #(
	parameter BASE_ADDR = 5'b0,
	parameter NUM_GPIOS = 8,
	parameter DFL_STATE = {NUM_GPIOS {1'b0}}
) (
	input rst,
	input clk,

	input [4:0] csr_a,
	input [7:0] csr_di,
	input csr_we,
	output reg [7:0] csr_do,

	output reg [NUM_GPIOS-1:0] out
);

always @(*) begin
	csr_do = 8'b0;
	if (csr_a == BASE_ADDR)
		csr_do = {{8-NUM_GPIOS {1'b0}}, out};
end

always @(posedge clk) begin
	if (rst)
		out <= DFL_STATE;
	else if (csr_a == BASE_ADDR & csr_we)
		out <= csr_di[NUM_GPIOS-1:0];
end

endmodule
