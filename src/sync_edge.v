module sync_edge #(
	parameter WIDTH = 1
) (
	input clk,

	input [WIDTH-1:0] in,
	output [WIDTH-1:0] out,
	output [WIDTH-1:0] out0,
	output [WIDTH-1:0] out_edge,
	output [WIDTH-1:0] out_negedge,
	output [WIDTH-1:0] out_posedge
);

/* synchonize & edge detect */
reg [WIDTH-1:0] in1, in0;
initial in0 = {WIDTH {1'b0}};
initial in1 = {WIDTH {1'b0}};
always @(posedge clk) begin
	{in1, in0} <= {in0, in};
end
assign out = in0;
assign out0 = in1;
assign out_edge = in1 ^ in0;
assign out_negedge = in1 & ~in0;
assign out_posedge = ~in1 & in0;

endmodule
