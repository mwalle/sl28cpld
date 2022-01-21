module i2c_slave #(
	parameter I2C_ADDRESS = 7'h4a
) (
	input rst,
	input clk,

	input sda,
	output reg sda_out,
	input scl,

	output [7:0] csr_a,
	input [7:0] csr_di,
	output csr_we,
	output [7:0] csr_do
);

/* synchronize inputs & edge detect*/
wire sda_in, sda_posedge, sda_negedge;
sync_edge sync_edge_sda (
	.clk(clk),

	.in(sda),
	.out(sda_in),
	.out_posedge(sda_posedge),
	.out_negedge(sda_negedge)
);

wire scl_in, scl_posedge, scl_negedge;
sync_edge sync_edge_scl (
	.clk(clk),

	.in(scl),
	.out(scl_in),
	.out_posedge(scl_posedge),
	.out_negedge(scl_negedge)
);

wire start   = sda_negedge & scl_in;
wire stop    = sda_posedge & scl_in;
wire capture = scl_posedge;
wire update  = scl_negedge;

reg [3:0] bit_counter;
always @(posedge clk) begin
	if (rst || start) begin
		bit_counter <= 4'd0;
	end else if (capture) begin
		bit_counter <= bit_counter + 4'b1;
		if (bit_counter == 4'd8)
			bit_counter <= 4'd0;
	end
end
wire ack_bit = (bit_counter == 4'd8) & capture;

reg [7:0] input_shift;
always @(posedge clk) begin
	if (rst)
		input_shift <= 8'h0;
	else if (capture)
		input_shift <= {input_shift[6:0], sda_in};
end

/* decode data */
wire address_match = input_shift[7:1] == I2C_ADDRESS;
wire read_write = input_shift[0];
wire master_ack = ~sda_in;

/* I2C state machine */
localparam [2:0] IDLE     = 3'd0,
		 DEV_ADDR = 3'd1,
		 READ     = 3'd2,
		 IDX_PTR  = 3'd3,
		 WRITE    = 3'd4;
reg [2:0] state, next_state;
always @(posedge clk) begin
	if (rst)
		state <= IDLE;
	else
		state <= next_state;
end

always @(*) begin
	next_state = state;

	/* A start or stop condition will always reset our FSM */
	if (start)
		next_state = DEV_ADDR;
	else if (stop)
		next_state = IDLE;
	else if (ack_bit) begin
		case (state)
		DEV_ADDR:
			if (!address_match)
				next_state = IDLE;
			else if (read_write)
				next_state = READ;
			else
				next_state = IDX_PTR;

		IDX_PTR:
			next_state = WRITE;

		READ:
			if (!master_ack)
				next_state = IDLE;
		endcase
	end
end

reg [7:0] idx_ptr;
always @(posedge clk) begin
	if (rst)
		idx_ptr <= 8'd0;
	else if (ack_bit) begin
		case (state)
		IDX_PTR:
			idx_ptr <= input_shift[7:0];

		READ,
		WRITE:
			idx_ptr <= idx_ptr + 8'd1;
		endcase
	end
end

reg [7:0] output_shift;
always @(posedge clk) begin
	if (rst)
		output_shift <= 8'h0;
	else if (ack_bit)
		output_shift <= csr_di;
	else if (capture)
		output_shift <= {output_shift[6:0], 1'b0};
end

wire need_ack = (state == DEV_ADDR && address_match) ||
		(state == IDX_PTR) ||
		(state == WRITE);

always @(posedge clk) begin
	if (rst)
		sda_out <= 1'b1;
	else if (start)
		sda_out <= 1'b1;
	if (update) begin
		sda_out <= 1'b1;
		if (bit_counter == 4'd8)
			sda_out <= !need_ack;
		else if (state == READ)
			sda_out <= output_shift[7];
	end
end

assign csr_a = idx_ptr;
assign csr_we = (state == WRITE) & ack_bit;
assign csr_do = input_shift;

endmodule
