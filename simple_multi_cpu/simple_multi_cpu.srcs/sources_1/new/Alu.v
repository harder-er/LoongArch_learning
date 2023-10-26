`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/10/10 22:35:33
// Design Name: 
// Module Name: Alu
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
`include "multi_cycle_cpu.h"
 //`define  Add        2'b00
 module Alu(	
    input [31:0] A,				
    input [31:0] B,				

    input [1:0] ALUOp,			//2位ALU控制信号
    output reg [31:0] ALUOut,	
    output reg zero		//零标志位
);	
	//reg [31:0] ALUOut;
	always@(A,B,ALUOp)	begin
		case(ALUOp)//决定ALU操作
			`Add:	
				begin
					ALUOut <= A+B;
				end
			
			`Sub:	
				begin 
					ALUOut <= A-B;
				end

			`Ori:	
				begin
					ALUOut <= A|B;
				end

			default:
				begin
					ALUOut <= 32'h0000_0000;
					//$display($time,,"(ALU)(DEFAULT)");
				end
		endcase
	end
	
	always @(A,B) begin
		if (A == B) zero <= 1'b1;
		else 		zero <= 1'b1;
	end
endmodule
			
