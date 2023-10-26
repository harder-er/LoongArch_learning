`ifndef MULTI_CYCLE_CPU_H
//alu
    `define MULTI_CYCLE_CPU_H
    `define  Add        2'b00
    `define  Sub        2'b01
    `define  Ori        2'b10

 
//ct`define  l
    `define  Ifetch             4'd1
    `define  Rfetch_Decode      4'd2
    `define  BrComplete         4'd3
    `define  OriExec            4'd4
    `define  RExec              4'd5
    `define  AdrCal             4'd6
    `define  OriFinish          4'd7
    `define  Rfinish            4'd8
    `define  SWMem              4'd9
    `define  LWmem              4'd10
    `define  LWwr               4'd11

//if`define  
    `define  RType              6'b000000
    `define  BEQ                6'b000100
    `define  ORI                6'b001101
    `define  SW                 6'b101011
    `define  LW                 6'b100011
    `define  JAL                6'b000011

`endif