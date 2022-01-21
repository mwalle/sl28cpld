// SPDX-License-Identifier: CERN-OHL-P-2.0
/*
 * I2C bus recovery module.
 *
 * I2C busses might become stuck on a board reset. That is, when the
 * a board reset occurs during an I2C transaction and the I2C slave pulls
 * the SDA line low. In this case we have to do an I2C bus recovery.
 *
 * Copyright (c) 2020-2022 Michael Walle <michael@walle.cc>
 */

module i2c_bus_reset (
	input clk,
	input ce,

	input sda,
	output sda_out,
	output scl_out,

	input start
);

localparam [2:0] IDLE        = 3'b011,
		 CLOCK_SCL0  = 3'b110,
		 CLOCK_SCL1  = 3'b111,
		 STROBE_SDA  = 3'b101;

/* state bits encode the line state */
assign scl_out = state[0];
assign sda_out = state[1];

reg [2:0] state;
initial state <= IDLE;
always @(posedge clk) begin
	if (start)
		state <= CLOCK_SCL1;
	else if (ce)
		case (state)
		CLOCK_SCL0:
			state <= CLOCK_SCL1;

		CLOCK_SCL1:
			if (sda)
				state <= STROBE_SDA;
			else
				state <= CLOCK_SCL0;

		STROBE_SDA:
			state <= IDLE;
		endcase
end

endmodule
