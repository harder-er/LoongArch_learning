`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/10/11 16:18:42
// Design Name: 
// Module Name: EXT
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


module EXT(
    input   [15:0]      Imm16,
    input   [1 :0]      ExtOp,
    output  reg [31:0]      Imm32
);
   // wire [1 :0]      ExtOp;
    always @(ExtOp,Imm16) begin
        case (ExtOp)
            2'b00:
                Imm32 <= {{16{1'b0}},Imm16};
            2'b01://符号扩展
                Imm32 <= {{16{Imm16[15]}},Imm16};
            2'b10://将立即数扩展到高位
                Imm32 <= {Imm16,{16{1'b0}}};
            default:
                Imm32 <= {{16{1'b0}},Imm16};
        endcase
    end          
endmodule
