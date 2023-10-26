`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/10/12 20:23:28
// Design Name: 
// Module Name: IR
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


module IR(
    input           IRWr,
    input           clk,
    input   [31:0]  in,
    output  reg [31:0]  IRout           
);
    always @(posedge clk) begin
        if (IRWr)
            IRout <= in;
    end
endmodule
