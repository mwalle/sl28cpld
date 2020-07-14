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
	input force_recovery,
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
reg program;

altufm_none ufm(
	.arclk(arclk),
	.ardin(1'b0),
	.arshft(1'b1),
	.drclk(drclk),
	.drdin(drdin),
	.drshft(drshft),
	.erase(erase),
	.oscena(1'b1),
	.program(program),

	.busy(busy),
	.drdout(drdout),
	.osc(osc)
);

localparam [4:0] STARTING     = 5'b00000,
		 SHIFT_ADDR0  = 5'b01110,
		 SHIFT_ADDR1  = 5'b01111,
		 LOAD_DATA0   = 5'b00001,
		 LOAD_DATA1   = 5'b00011,
		 SHIFT_DATA0  = 5'b00101,
		 SHIFT_DATA1  = 5'b00111,
		 DONE         = 5'b10111;

assign arclk = state[0];
assign drclk = done ? drclk_r : state[1];
assign drshft = done ? drshft_r : state[2];
assign done = state[4];
assign cfg = (done & ~force_recovery) ? ufm_data : 16'hffff;

reg [4:0] state;
reg [15:0] ufm_data;
reg [3:0] counter;
initial state = STARTING;
always @(posedge clk) begin
	case (state)
	STARTING: begin
		counter[3:0] <= 4'd0;
		state <= SHIFT_ADDR1;
	end

	SHIFT_ADDR0: begin
		counter[3:0] <= counter[3:0] + 4'd1;
		state <= SHIFT_ADDR1;
	end

	SHIFT_ADDR1:
		if (counter[3:0] == 4'd9)
			state <= LOAD_DATA0;
		else
			state <= SHIFT_ADDR0;

	LOAD_DATA0:
		state <= LOAD_DATA1;

	LOAD_DATA1: begin
		counter[3:0] <= 4'd0;
		state <= SHIFT_DATA0;
	end

	SHIFT_DATA0: begin
		ufm_data <= {ufm_data[14:0], drdout};
		if (counter[3:0] == 4'd15)
			state <= DONE;
		else
			state <= SHIFT_DATA1;
	end

	SHIFT_DATA1: begin
		counter[3:0] <= counter[3:0] + 4'd1;
		state <= SHIFT_DATA0;
	end

	DONE:
		if (start)
			state <= LOAD_DATA0;
	endcase
end

always @(posedge clk) begin
	if (rst || !done) begin
		drclk_r <= 1'b1;
		drdin <= 1'b0;
		program <= 1'b0;
		erase <= 1'b0;
		drshft_r <= 1'b1;
	end else
		if (csr_we && csr_a == BASE_ADDR + R_UFM_CTRL)
			{drshft_r, erase, program, drdin, drclk_r} <= csr_di[5:1];
end

always @(*) begin
	csr_do = 8'b0;
	case (csr_a)
	BASE_ADDR + R_UFM_DATA0:
		csr_do = cfg[15:8];
	BASE_ADDR + R_UFM_DATA1:
		csr_do = cfg[7:0];
	BASE_ADDR + R_UFM_CTRL:
		csr_do = {busy, drdout, drshft_r, erase, program, drdin, drclk_r, 1'b0};
	endcase
end

endmodule
