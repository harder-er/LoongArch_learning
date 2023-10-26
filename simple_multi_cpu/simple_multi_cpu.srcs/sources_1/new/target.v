`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/10/13 10:18:28
// Design Name: 
// Module Name: target
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


module target(
    input           clk,
    input           reset,
    input   [31:0]  tar_in,
    input           BrWr,
    output reg[31:0]  tar_out
);
    always @(posedge clk) begin
        if (!BrWr)
            tar_out <= tar_in;
    end

endmodule
