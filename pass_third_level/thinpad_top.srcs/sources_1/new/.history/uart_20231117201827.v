`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/11/17 19:10:35
// Design Name: 
// Module Name: uart
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


module uart(
    input         clk,
    input         resetn,
    // inst sram interface
    input [ 3:0] inst_sram_en,         
    input [ 3:0] inst_sram_we,
    input [31:0] inst_sram_addr,
    // output [31:0] inst_sram_wdata,
    output [31:0] inst_sram_rdata,
    // data sram interface
    input [ 3:0] data_sram_en,
    input [ 3:0] data_sram_we,
    input [31:0] data_sram_addr,
    input [31:0] data_sram_wdata,
    output  [31:0] data_sram_rdata, 
    );
endmodule
