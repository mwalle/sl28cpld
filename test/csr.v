reg [4:0] csr_a;
reg [7:0] csr_di;
reg csr_we;
wire [7:0] csr_do;

task csrwrite;
	input [4:0] address;
	input [7:0] data;
begin
	csr_a = address;
	csr_di = data;
	$display("CSR write @%02x: %02x", csr_a, csr_di);
	csr_we = 1'b1;
	waitclock;
	csr_we = 1'b0;
end
endtask

task csrread;
	input [4:0] address;
begin
	csr_a = address;
	waitclock;
	$display("CSR read  @%02x: %02x", csr_a, csr_do);
end
endtask
