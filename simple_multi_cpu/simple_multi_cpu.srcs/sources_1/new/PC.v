`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/10/11 09:15:18
// Design Name: 
// Module Name: PC
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
module PC(	
	input PCWr,			//控制PC是否更改
	input [31:0] NPC,		
	input clk,				
	input reset,			
	output reg [31:0] PC	
);

	// always@(posedge clk, posedge reset) begin
	// 	if(reset) PC <= 32'h0000_3000;//PC复位后初值为0x0000_3000
	// 	else if(PCWr) begin
	// 		PC <= NPC;
	// 	end else begin
	// 		PC <= PC;
	// 	end
	// end
	always @(posedge clk or posedge reset) begin
		if (reset) PC <= 32'h0000_3000;
		else if (PCWr)
			PC <= NPC;
	end
endmodule

