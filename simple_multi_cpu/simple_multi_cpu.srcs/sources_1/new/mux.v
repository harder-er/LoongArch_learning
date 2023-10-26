`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/10/11 16:40:29
// Design Name: 
// Module Name: mux
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

//二选一 32bit
module mutiplexer32_1(
    input s,
        
    input [31:0] d0,
    input [31:0] d1,
        
    output reg [31:0] out32_1
);
	always @(s,d0,d1) begin
        if(s == 0) out32_1 <= d0;
        else out32_1 <= d1;
    end
endmodule

module mutiplexer5_1(
    input s,
        
    input [4:0] d0,
    input [4:0] d1,
        
    output reg [4:0] out5_1
);
	always @(s,d0,d1) begin
        if(s == 0) out5_1 <= d0;
        else out5_1 <= d1;
    end
endmodule

//四选一 32bit  00ALU运算结果 01数据存储器的输出 10JAL跳转的目的地址 11没用
module mutiplexer32_2(
    input [1 :0]    s,	//MemToReg
        
    input [31:0]    d0,	
    input [31:0]    d1, 	
    input [31:0]    d2,	
    input [31:0]    d3,	
        
    output reg [31:0] out32_2
);
    always @(s,d0,d1,d2,d3) begin
        if(s == 2'b00) 			out32_2 <= d0;
        else if(s == 2'b01) out32_2 <= d1;	//lw
        else if(s == 2'b10) out32_2 <= d2;
        else                 out32_2 <= d3;	//lb
    end
endmodule





