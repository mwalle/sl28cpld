module ro_reg #(
	parameter BASE_ADDR = 5'b0
) (
	input rst,
	input clk,

	input [4:0] csr_a,
	input [7:0] csr_di,
	input csr_we,
	output reg [7:0] csr_do,

	input [7:0] in
);

always @(*) begin
	csr_do = 8'b0;
	if (csr_a == BASE_ADDR)
		csr_do = in;
end

endmodule
