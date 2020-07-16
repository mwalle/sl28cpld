module gpio #(
	parameter BASE_ADDR = 5'b0,
	parameter NUM_GPIOS = 8,
	parameter DFL_STATE = {NUM_GPIOS {1'b0}},
	parameter DFL_OE = {NUM_GPIOS {1'b0}}
) (
	input rst,
	input clk,

	input [4:0] csr_a,
	input [7:0] csr_di,
	input csr_we,
	output reg [7:0] csr_do,

	input [NUM_GPIOS-1:0] in,
	output reg [NUM_GPIOS-1:0] out,
	output reg [NUM_GPIOS-1:0] oe,
	output reg irq
);

/* synchonize & edge detect */
wire [NUM_GPIOS-1:0] in_edge;
wire [NUM_GPIOS-1:0] in_sync;
sync_edge #(
	.WIDTH(NUM_GPIOS)
) sync_edge_gpio (
	.clk(clk),

	.in(in),
	.out(in_sync),
	.out_edge(in_edge)
);

reg [NUM_GPIOS-1:0] ie;
reg [NUM_GPIOS-1:0] ip;
always @(*) begin
	csr_do = 8'b0;
	case (csr_a)
		BASE_ADDR + 5'h0: csr_do = {{8-NUM_GPIOS {1'b0}}, oe};
		BASE_ADDR + 5'h1: csr_do = {{8-NUM_GPIOS {1'b0}}, out};
		BASE_ADDR + 5'h2: csr_do = {{8-NUM_GPIOS {1'b0}}, in_sync};
		BASE_ADDR + 5'h3: csr_do = {{8-NUM_GPIOS {1'b0}}, ie};
		BASE_ADDR + 5'h4: csr_do = {{8-NUM_GPIOS {1'b0}}, ip};
	endcase
end

always @(posedge clk) begin
	if (rst) begin
		oe <= DFL_OE;
		out <= DFL_STATE;
		ie <= {NUM_GPIOS {1'b0}};
		ip <= {NUM_GPIOS {1'b0}};
		irq <= 1'b0;
	end else begin
		irq <= 1'b0;
		ip <= ip | in_edge;
		if (|(in_edge & ie))
			irq <= 1'b1;
		if (csr_we) begin
			case (csr_a)
				BASE_ADDR + 5'h0: oe <= csr_di[NUM_GPIOS-1:0];
				BASE_ADDR + 5'h1: out <= csr_di[NUM_GPIOS-1:0];
				BASE_ADDR + 5'h3: begin
					ie <= csr_di[NUM_GPIOS-1:0];
					irq <= |(ie & ip);
				end
				BASE_ADDR + 5'h4: ip <= ip & ~csr_di[NUM_GPIOS-1:0];
			endcase
		end
	end
end

endmodule
