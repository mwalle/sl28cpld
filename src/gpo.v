module gpo #(
	parameter BASE_ADDR = 5'b0,
	parameter NUM_GPIOS = 8
) (
	input rst,
	input clk,

	input [4:0] csr_a,
	input [7:0] csr_di,
	input csr_we,
	output reg [7:0] csr_do,

	output reg [NUM_GPIOS-1:0] out
);

always @(posedge clk) begin
	if (rst)
		csr_do <= {NUM_GPIOS {1'b0}};
	else begin
		csr_do <= 8'b0;
		if (csr_a == BASE_ADDR)
			csr_do <= out;
	end
end

always @(posedge clk) begin
	if (rst)
		out <= {NUM_GPIOS {1'b0}};
	else if (csr_a == BASE_ADDR & csr_we)
		out <= csr_di[NUM_GPIOS-1:0];
end

endmodule
