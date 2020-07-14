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

/* synchonize & edge detect */
reg in2, in1, in0;
always @(posedge clk) begin
	if (rst)
		{in2, in1, in0} <= 3'b0;
	else
		{in2, in1, in0} <= {in1, in0, tacho_in};
end
wire rising_edge = in2 & ~in1;

reg [9:0] counter;
reg [9:0] counter0;
always @(posedge clk) begin
	if (rst) begin
		counter <= 10'd0;
		counter0 <= 10'd0;
	end else begin
		if (rising_edge)
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
