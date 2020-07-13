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

/* synchonize */
reg [NUM_GPIOS-1:0] in0;
always @(posedge clk) begin
	if (rst)
		in0 <= {NUM_GPIOS {1'b0}};
	else
		in0 <= in;
end

always @(posedge clk) begin
	if (rst)
		csr_do <= {NUM_GPIOS {1'b0}};
	else begin
		csr_do <= 8'b0;;
		if (csr_a == BASE_ADDR)
			csr_do <= in0;
	end
end

endmodule
