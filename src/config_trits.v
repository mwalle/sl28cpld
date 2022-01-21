module config_trits #(
	parameter BASE_ADDR = 5'h0
) (
	input clk,
	input rst,
	inout ce,

	input [4:0] csr_a,
	input [7:0] csr_di,
	input csr_we,
	output reg [7:0] csr_do,

	inout [3:0] trits
);

reg oe;
reg outval;
assign trits = oe ? {4 {outval}} : 4'bzzzz;

always @(posedge clk) begin
	if (rst) begin
		oe <= 1'b0;
		outval <= 1'b0;
	end else begin
		if (csr_we && csr_a == BASE_ADDR)
			{oe, outval} <= csr_di[7:6];
	end
end

always @(*) begin
	csr_do = 8'b0;
	if (csr_a == BASE_ADDR)
		csr_do = {oe, outval, 2'b00, trits};
end

endmodule
