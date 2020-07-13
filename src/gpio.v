module gpio #(
	parameter BASE_ADDR = 5'b0,
	parameter NUM_GPIOS = 8
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
reg [NUM_GPIOS-1:0] in0, in1, in2;
always @(posedge clk) begin
	if (rst)
		{in2, in1, in0} <= {NUM_GPIOS {3'b0}};
	else
		{in2, in1, in0} <= {in1, in0, in};
end
wire [NUM_GPIOS-1:0] in_edge = in2 ^ in1;

reg [NUM_GPIOS-1:0] ie;
reg [NUM_GPIOS-1:0] ip;
always @(posedge clk) begin
	if (rst)
		csr_do <= {NUM_GPIOS {1'b0}};
	else begin
		csr_do <= 8'b0;
		case (csr_a)
			BASE_ADDR + 5'h0: csr_do <= oe;
			BASE_ADDR + 5'h1: csr_do <= out;
			BASE_ADDR + 5'h2: csr_do <= in1;
			BASE_ADDR + 5'h3: csr_do <= ie;
			BASE_ADDR + 5'h4: csr_do <= ip;
		endcase
	end
end

always @(posedge clk) begin
	if (rst) begin
		oe <= {NUM_GPIOS {1'b0}};
		out <= {NUM_GPIOS {1'b0}};
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
