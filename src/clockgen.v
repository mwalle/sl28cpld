// SPDX-License-Identifier: CERN-OHL-P-2.0
/*
 * Clock Enable generator to be used with the internal oscillator of the
 * MAX V. The frequency is about ~4.6MHz.
 *
 * Copyright (c) 2020-2022 Michael Walle <michael@walle.cc>
 */

module clockgen (
	input clk,
	output reg ce_32khz,
	output reg ce_8hz,
	output reg ce_1hz
);

reg [7:0] cnt_32khz;
initial cnt_32khz = 8'd0;
always @(posedge clk) begin
	ce_32khz <= 1'b0;
	cnt_32khz <= cnt_32khz + 8'd1;
	if (cnt_32khz == 8'h90) begin
		ce_32khz <= 1'b1;
		cnt_32khz <= 8'd0;
	end
end

reg [11:0] cnt_8hz;
initial cnt_8hz = 12'd0;
always @(posedge clk) begin
	ce_8hz <= 1'b0;
	if (ce_32khz)
		cnt_8hz <= cnt_8hz + 12'd1;
	if (cnt_8hz == 12'hfa0) begin
		cnt_8hz <= 12'd0;
		ce_8hz <= 1'b1;
	end
end

reg [3:0] cnt_1hz;
initial cnt_1hz = 4'd0;
always @(posedge clk) begin
	ce_1hz <= 1'd0;
	if (ce_8hz)
		cnt_1hz <= cnt_1hz + 4'd1;
	if (cnt_1hz == 4'h8) begin
		cnt_1hz <= 4'd0;
		ce_1hz <= 1'd1;
	end
end

endmodule
