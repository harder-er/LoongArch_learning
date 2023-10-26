`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/10/11 15:00:00
// Design Name: 
// Module Name: regfile
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


module regfile(
    input         clk,
    // READ PORT 1
    input  [ 4:0] raddr1,
    output [31:0] rdata1,
    // READ PORT 2
    input  [ 4:0] raddr2,
    output [31:0] rdata2,
    // WRITE PORT
    input         RFWr,       //write enable, HIGH valid
    input  [ 4:0] waddr,
    input  [31:0] wdata
);
    reg [31:0] rf[31:0];

    //WRITE
    always @(posedge clk) begin
        if (RFWr) rf[waddr] <= wdata;
    end

    //READ OUT 1
    assign rdata1 = (raddr1 == 5'b0) ? 32'b0 : rf[raddr1];

    //READ OUT 2
    assign rdata2 = (raddr2 == 5'b0) ? 32'b0 : rf[raddr2];

endmodule
