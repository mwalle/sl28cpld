// SPDX-License-Identifier: CERN-OHL-P-2.0
/*
 * Simple GPI (input only) module.
 *
 * Copyright (c) 2020-2022 Michael Walle <michael@walle.cc>
 */

module gpi #(
	parameter BASE_ADDR = 5'b0,
	parameter NUM_GPIOS = 8
) (
	input rst,
	input clk,

	input [4:0] csr_a,
	input [7:0] csr_di,
	input csr_we,
	output reg [7:0] csr_do,

	input [NUM_GPIOS-1:0] in
);

always @(*) begin
	csr_do = 8'b0;
	if (csr_a == BASE_ADDR)
		csr_do = {{8-NUM_GPIOS {1'b0}}, in};
end

endmodule
