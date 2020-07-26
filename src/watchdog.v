module watchdog #(
	parameter BASE_ADDR = 5'h0,
	parameter DEFAULT_OE = 2'b00,
	parameter DEFAULT_TIMEOUT = 8'hff,
	parameter KICK_VALUE = 8'h6b
) (
	input rst,
	input clk,
	input ce,

	input [4:0] csr_a,
	input [7:0] csr_di,
	input csr_we,
	output reg [7:0] csr_do,

	input [1:0] wdt_en_default,
	output [1:0] wdt_out,
	output [1:0] wdt_out_strobe,
	output force_recovery_mode,
	output irq
);

localparam R_CTRL = 5'h0;
localparam R_TOUT = 5'h1;
localparam R_KICK = 5'h2;
localparam R_CNT = 5'h3;

reg wdt_locked;
reg [1:0] wdt_oe;
reg [1:0] wdt_en;
reg [7:0] wdt_tout;

reg [7:0] wdt_cnt;
always @(posedge clk) begin
	/* counter reset is gated and only allowed in non-failsafe mode */
	if (rst & ~wdt_en[1])
		wdt_cnt <= DEFAULT_TIMEOUT;
	else if (wdt_kick)
		wdt_cnt <= wdt_tout;
	else if (ce & !wdt_bite & |wdt_en)
		wdt_cnt <= wdt_cnt - 8'd1;
end
wire wdt_bite = |wdt_en & (wdt_cnt == 8'd0);
assign force_recovery_mode = wdt_bite & wdt_en[1];
assign wdt_out = wdt_oe & {wdt_bite, wdt_bite};

reg wdt_bite0;
always @(posedge clk)
	wdt_bite0 <= wdt_bite;
wire wdt_bite_pulse = !wdt_bite0 & wdt_bite;

assign irq = wdt_bite_pulse;
assign wdt_out_strobe = wdt_oe & {wdt_bite_pulse, wdt_bite_pulse};

always @(posedge clk) begin
	if (rst) begin
		wdt_en <= wdt_en_default;
		wdt_oe <= DEFAULT_OE;
		wdt_tout <= DEFAULT_TIMEOUT;
		wdt_locked <= 1'b0;
	end else begin
		if (csr_we & !wdt_locked) begin
			case (csr_a)
				BASE_ADDR + R_CTRL: {wdt_oe, wdt_locked, wdt_en} <= {csr_di[7:6], csr_di[2:0]};
				BASE_ADDR + R_TOUT: wdt_tout <= csr_di;
			endcase
		end
	end
end
wire wdt_kick = csr_we & (csr_a == BASE_ADDR + R_KICK) & (csr_di == KICK_VALUE);

always @(*) begin
	csr_do = 8'b0;
	case (csr_a)
		BASE_ADDR + R_CTRL: csr_do = {wdt_oe, 3'b0, wdt_locked, wdt_en};
		BASE_ADDR + R_TOUT: csr_do = wdt_tout;
		BASE_ADDR + R_CNT: csr_do = wdt_cnt;
	endcase
end

endmodule
