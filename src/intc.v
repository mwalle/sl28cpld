module intc #(
	parameter BASE_ADDR = 5'b0,
	parameter NUM_INTS = 8
) (
	input rst,
	input clk,

	input [4:0] csr_a,
	input [7:0] csr_di,
	input csr_we,
	output reg [7:0] csr_do,

	input [NUM_INTS-1:0] int,
	output reg irq
);

reg [NUM_INTS-1:0] ie;
reg [NUM_INTS-1:0] ip;
always @(*) begin
	csr_do = 8'b0;
	case (csr_a)
		BASE_ADDR + 5'h0: csr_do = {{8-NUM_INTS {1'b0}}, ie};
		BASE_ADDR + 5'h1: csr_do = {{8-NUM_INTS {1'b0}}, ip};
	endcase
end

always @(posedge clk) begin
	if (rst) begin
		ie <= {NUM_INTS {1'b0}};
		ip <= {NUM_INTS {1'b0}};
		irq <= 1'b0;
	end else begin
		irq <= 1'b0;
		ip <= ip | int;
		if (|(int & ie))
			irq <= 1'b1;
		if (csr_we) begin
			case (csr_a)
				BASE_ADDR + 5'h0: begin
					ie <= csr_di[NUM_INTS-1:0];
					irq <= |(ie & ip);
				end
				BASE_ADDR + 5'h1: ip <= ip & ~csr_di[NUM_INTS-1:0];
			endcase
		end
	end
end

endmodule
