module im_4k( clk,RAdr,WrAdr,MemWr,Din,Dout );
    input [11:2]	RAdr;
	input [11:2]	WrAdr;
	input			MemWr;
	input [31:0]	Din;
    input 			clk;
	output [31:0] 	Dout;
    reg [31:0] imem[1023:0];
	always @(posedge clk) begin
		if (MemWr) imem[WrAdr] <= Din;
	end
    assign Dout = imem[RAdr];
endmodule    
