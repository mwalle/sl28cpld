// SPDX-License-Identifier: CERN-OHL-P-2.0
/*
 * Power state machine.
 *
 * Rather board specific. Will control the power state of the board during
 * power-up and runtime.
 *
 * Copyright (c) 2020-2022 Michael Walle <michael@walle.cc>
 */

module power_fsm #(
	parameter LONG_PRESS_DELAY = 3'd0
) (
	input clk,
	input ce_1hz,
	input ce_8hz,

	input start,
	input initial_pwr_off,
	input pwr_off,
	input pwr_on,
	input pwr_btn,
	output pwr_enable
);

localparam [2:0] WAIT               = 3'b001,
		 POWER_ON           = 3'b111,
		 POWER_OFF_PENDING  = 3'b101,
		 POWER_OFF_WAIT1    = 3'b110,
		 POWER_OFF_WAIT2    = 3'b100,
		 POWER_OFF_WAIT3    = 3'b010,
		 POWER_OFF          = 3'b000;
reg [2:0] state;
assign pwr_enable = state[0];
reg [2:0] pwr_btn_cnt;
initial pwr_btn_cnt = 3'b0;
initial state = WAIT;
always @(posedge clk) begin
	case (state)
	WAIT:
		if (start)
			if (initial_pwr_off)
				state <= POWER_OFF;
			else
				state <= POWER_ON;

	POWER_ON:
		if (pwr_off)
			state <= POWER_OFF;
		else if (pwr_btn) begin
			pwr_btn_cnt <= 3'd0;
			state <= POWER_OFF_PENDING;
		end

	POWER_OFF_PENDING:
		if (pwr_off)
			state <= POWER_OFF;
		else if (!pwr_btn)
			state <= POWER_ON;
		else if (pwr_btn_cnt == LONG_PRESS_DELAY)
			state <= POWER_OFF_WAIT1;
		else if (ce_1hz)
			pwr_btn_cnt <= pwr_btn_cnt + 3'd1;

	POWER_OFF_WAIT1:
		if (!pwr_btn)
			state <= POWER_OFF_WAIT2;

	POWER_OFF_WAIT2:
		if (ce_8hz)
			state <= POWER_OFF_WAIT3;

	POWER_OFF_WAIT3:
		if (ce_8hz)
			state <= POWER_OFF;

	POWER_OFF:
		if (pwr_on || pwr_btn)
			state <= POWER_ON;
	endcase
end

endmodule
