// SPDX-License-Identifier: CERN-OHL-P-2.0
/*
 * Simple 16bit UFM storage module for Altera CPLDs.
 *
 * Copyright (c) 2020-2022 Michael Walle <michael@walle.cc>
 */

module cfg_ctrl_altera_ufm #(
	parameter BASE_ADDR = 5'h0
) (
	input rst,
	input clk,

	input [4:0] csr_a,
	input [7:0] csr_di,
	input csr_we,
	output reg [7:0] csr_do,

	input start,
	input failsafe_mode,
	output done,
	output [15:0] cfg,
	output osc
);

localparam R_UFM_DATA0 = 5'h0;
localparam R_UFM_DATA1 = 5'h1;
localparam R_UFM_CTRL = 5'h2;

wire busy;
wire drdout;
wire arclk;
wire drclk;
wire drshft;
reg drclk_r;
reg drdin;
reg drshft_r;
reg erase;
reg program_r;

altufm_none ufm(
	.arclk(arclk),
	.ardin(1'b0),
	.arshft(1'b1),
	.drclk(drclk),
	.drdin(drdin),
	.drshft(drshft),
	.erase(erase),
	.oscena(1'b1),
	.program(program_r),

	.busy(busy),
	.drdout(drdout),
	.osc(osc)
);

localparam [3:0] STARTING     = 4'b0000,
		 SHIFT_ADDR   = 4'b0001,
		 LOAD_DATA    = 4'b0010,
		 SHIFT_DATA   = 4'b0110,
		 DONE         = 4'b1000;

assign arclk = clk & state[0];
assign drclk = done ? drclk_r : (clk & state[1]);
assign drshft = done ? drshft_r : state[2];
assign done = state[3];
assign cfg = ufm_data;

reg [3:0] state;
reg [15:0] ufm_data;
reg [3:0] counter;
initial state = STARTING;
always @(posedge clk) begin
	case (state)
	STARTING: begin
		counter <= 4'd0;
		state <= SHIFT_ADDR;
	end

	SHIFT_ADDR: begin
		if (counter == 4'd9)
			state <= LOAD_DATA;
		else
			counter <= counter + 4'd1;
	end

	LOAD_DATA: begin
		counter <= 4'd0;
		state <= SHIFT_DATA;
	end

	SHIFT_DATA: begin
		ufm_data <= {ufm_data[14:0], failsafe_mode ? 1'b1 : drdout};
		if (counter == 4'd15)
			state <= DONE;
		else
			counter <= counter + 4'd1;
	end

	DONE:
		if (start)
			state <= LOAD_DATA;
	endcase
end

always @(posedge clk) begin
	if (rst || !done) begin
		drclk_r <= 1'b1;
		drdin <= 1'b0;
		program_r <= 1'b0;
		erase <= 1'b0;
		drshft_r <= 1'b1;
	end else
		if (csr_we && csr_a == BASE_ADDR + R_UFM_CTRL)
			{drshft_r, erase, program_r, drdin, drclk_r} <= csr_di[5:1];
end

always @(*) begin
	case (csr_a)
	BASE_ADDR + R_UFM_DATA0:
		csr_do = cfg[15:8];
	BASE_ADDR + R_UFM_DATA1:
		csr_do = cfg[7:0];
	BASE_ADDR + R_UFM_CTRL:
		csr_do = {busy, drdout, drshft_r, erase, program_r, drdin, drclk_r, 1'b0};
	default:
		csr_do = 8'b0;
	endcase
end

endmodule
