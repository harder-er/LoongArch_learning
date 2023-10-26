`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/10/11 18:36:00
// Design Name: 
// Module Name: ctrl
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

module ctrl(
    input           clk,
    input           reset,
    input [5:0]     opcode,
    output  reg     PCWr,
    output  reg     PCWrCond,

    output  reg     IorD,
    output  reg     MemWr,
    output  reg     IRWr,
    output  reg     RegDst,
    output  reg     RegWr,
    output  reg     ALUSelA,
    output reg [1:0]     ALUSelB,
    output  reg     PCSrc,
    output  reg     BrWr,
    output reg [1:0]    ExtOp,
    output  reg     MemtoReg,
    output reg [1:0]     ALUOp         
);
    reg     [3:0]   CurState,NextState;
    
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            CurState <= 4'b0000; // 初始化状态
        end else begin
            CurState <= NextState;
        end
    end


    always @(CurState,opcode,NextState) begin
        case(CurState)
            4'b0000: NextState <= `Ifetch;
            `Ifetch: begin
                NextState <= `Rfetch_Decode;
            end
            `Rfetch_Decode:begin
                if (opcode == `BEQ) begin
                    NextState <= `BrComplete;
                end else if (opcode == `ORI) begin
                    NextState <= `OriExec;
                end else if (opcode == `RType) begin
                    NextState <= `RExec;
                end else if (opcode == `LW || opcode == `SW) begin
                    NextState <= `AdrCal;
                end
            end
            `BrComplete:
                NextState <= `Ifetch;
            `OriExec:
                NextState <= `OriFinish;
            `RExec:
                NextState <= `Rfinish;
            `AdrCal: begin
                if (opcode == `SW) 
                    NextState <= `SWMem;
                else if(opcode == `LW) 
                    NextState <= `LWmem;
            end
            `OriFinish: 
                NextState <= `Ifetch;
            `Rfinish:
                NextState <= `Ifetch;
            `SWMem:
                NextState <= `Ifetch;
            `LWmem:
                NextState <= `LWwr;
            `LWwr: 
                NextState <= `Ifetch;
            default:
                NextState <= CurState;
        endcase
    end
    always @(posedge reset) begin
        if (reset) begin
            ALUOp       <=  2'b00;
            PCWr        <=  1'b0;
            PCWrCond    <=  1'b0;
            IorD        <=  1'b0;
            MemWr       <=  1'b0;
            IRWr        <=  1'b0;
            RegDst      <=  1'b0;
            RegWr       <=  1'b0;
            ALUSelA     <=  1'b0;
            ALUSelB     <=  2'b00;
            PCSrc       <=  1'b0;
            BrWr        <=  1'b0;
            ExtOp       <=  2'b00;
            MemtoReg    <=  1'b0;
            BrWr        <=  1'b0;
        end 
    end
    always @(CurState) begin
        case(CurState)
            `Ifetch: begin
                ALUOp       <=  `Add;
                PCWr        <=  1'b1;
                MemWr       <=  1'b0;
                RegWr       <=  1'b0;
                ALUSelA     <=  1'b0;
                ALUSelB     <=  2'b00;
                PCSrc       <=  1'b0;
                IRWr        <=  1'b1;
                PCWrCond    <=  1'bx;
                IorD        <=  1'b0;
                RegDst      <=  1'bx;
                MemtoReg    <=  1'bx;
                BrWr        <=  1'b0;
                ExtOp       <=  2'b00;
            end 
            `Rfetch_Decode: begin
                PCWr        <=  1'b0;
                PCWrCond    <=  1'b0;
                MemWr       <=  1'b0;
                IRWr        <=  1'b0;
                RegWr       <=  1'b0;
                ALUOp       <=  `Add;
                BrWr        <=  1'b1;
                ExtOp       <=  1'b1;
                ALUSelA     <=  1'b0;
                ALUSelB     <=  2'b10;
                RegDst      <=  1'bx;
                PCSrc       <=  1'bx;
                IorD        <=  1'bx;
                MemtoReg    <=  1'bx;      
                BrWr        <=  1'b0;  
            end
            `BrComplete: begin
                PCWr        <=  1'b0;
                MemWr       <=  1'b0;
                IRWr        <=  1'b0;
                RegWr       <=  1'b0;
                ALUOp       <=  `Sub;
                ALUSelB     <=  2'b01;
                PCWrCond    <=  1'b1;
                ALUSelA     <=  1'b1;
                PCSrc       <=  1'b1;
                IorD        <=  1'bx;
                MemtoReg     <=  1'bx;
                RegDst      <=  1'bx;
                ExtOp       <=  1'bx; 
                BrWr        <=  1'b0;
            end
            `OriExec: begin
                PCWr        <=  1'b0;
                MemWr       <=  1'b0;
                IRWr        <=  1'b0;
                RegDst      <=  1'b0;
                RegWr       <=  1'b0;
                ALUOp       <=  `Ori;
                ALUSelA     <=  1'b1;
                ALUSelB     <=  2'b11;
                MemtoReg    <=  1'bx;
                IorD        <=  1'bx;
                PCSrc       <=  1'bx;
                PCWrCond    <=  1'b0;
                BrWr        <=  1'b0;
                ExtOp       <=  2'b00;
            end 
            `RExec: begin
                PCWr        <=  1'b0;
                PCWrCond    <=  1'b0;
                MemWr       <=  1'b0;
                RegDst      <=  1'b1;
                IRWr        <=  1'b0;
                RegWr       <=  1'b0;
                ALUSelA     <=  1'b1;
                ALUSelB     <=  2'b01;
                ALUOp       <=  `RType;
                PCSrc       <=  1'bx;
                IorD        <=  1'bx;
                MemtoReg    <=  1'bx;
                ExtOp       <=  1'bx;
                BrWr        <=  1'b0;
            end
            `AdrCal: begin
                PCWr        <=  1'b0;
                MemWr       <=  1'b0;
                PCWrCond    <=  1'b0;
                IRWr        <=  1'b0;
                RegDst      <=  1'b0;
                RegWr       <=  1'b0;
                IorD        <=  1'b0;
                ExtOp       <=  1'b1;
                ALUSelA     <=  1'b1;
                ALUSelB     <=  2'b11;
                ALUOp       <=  `Add;
                MemtoReg    <=  1'bx;
                PCSrc       <=  1'bx;
                BrWr        <=  1'b0;
            end
            `OriFinish: begin
                PCWr        <=  1'b0;
                MemWr       <=  1'b0;
                IRWr        <=  1'b0;
                RegDst      <=  1'b0;
                PCWrCond    <=  1'b0;
                ALUOp       <=  `Ori;
                IorD        <=  1'bx;
                PCSrc       <=  1'bx;
                ALUSelB     <=  2'b11;
                ALUSelA     <=  1'b1;
                RegWr       <=  1'b1;
                BrWr        <=  1'b0;
                ExtOp       <=  2'b00;
                MemtoReg    <=  1'b0;
            end
            `Rfinish: begin
                PCWr        <=  1'b0;
                PCWrCond    <=  1'b0;
                MemWr       <=  1'b0;
                IRWr        <=  1'b0;
                ALUOp       <=  `RType;
                RegDst      <=  1'b1;
                RegWr       <=  1'b1;
                ALUSelA     <=  1'b1;
                ALUSelB     <=  2'b01;
                IorD        <=  1'bx;
                PCSrc       <=  1'bx;
                ExtOp       <=  1'bx;
                BrWr        <=  1'b0;
                MemtoReg    <=  1'b0;
            end
            `SWMem: begin
                PCWr        <=  1'b0;
                PCWrCond    <=  1'b0;
                IorD        <=  1'b0;
                IRWr        <=  1'b0;
                ExtOp       <=  1'b1;
                MemWr       <=  1'b1;
                RegWr       <=  1'b0;
                ALUSelA     <=  1'b1;
                ALUSelB     <=  2'b11;
                ALUOp       <=  `Add;
                PCSrc       <=  1'bx;
                RegDst      <=  1'bx;
                MemtoReg    <=  1'bx;
                BrWr        <=  1'b0;
            end 
            `LWmem: begin
                PCWr        <=  1'b0;
                PCWrCond    <=  1'b0;
                MemWr       <=  1'b0;
                IRWr        <=  1'b0;
                RegDst      <=  1'b0;
                RegWr       <=  1'b0;
                ExtOp       <=  1'b1;
                ALUSelA     <=  1'b1;
                IorD        <=  1'b1;
                ALUSelB     <=  2'b11;
                ALUOp       <=  `Add;
                MemtoReg    <=  1'bx;
                PCSrc       <=  1'bx;
                BrWr        <=  1'b0;
            end 
            `LWwr: begin
                PCWr        <=  1'b0;
                PCWrCond    <=  1'b0;
                MemWr       <=  1'b0;
                IRWr        <=  1'b0;
                RegDst      <=  1'b0;
                ALUSelA     <=  1'b1;
                RegWr       <=  1'b1;
                ExtOp       <=  1'b1;
                MemtoReg    <=  1'b1;
                ALUSelB     <=  2'b11;
                ALUOp       <=  `Add;
                PCSrc       <=  1'bx;
                BrWr        <=  1'b0;
                IorD        <=  1'bx;
            end
            default: begin
                ALUOp       <=  `Add;
                PCWr        <=  1'b0;
                PCWrCond    <=  1'b0;
                IorD        <=  1'b0;
                MemWr       <=  1'b0;
                IRWr        <=  1'b0;
                RegDst      <=  1'b0;
                RegWr       <=  1'b0;
                ALUSelA     <=  1'b0;
                ALUSelB     <=  2'b00;
                PCSrc       <=  1'b0;
                BrWr        <=  1'b0;
                ExtOp       <=  2'b00;
                MemtoReg    <=  1'b0;
            end
                
        endcase

    end
endmodule
