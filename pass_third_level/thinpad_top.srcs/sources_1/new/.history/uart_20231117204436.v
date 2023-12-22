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

    inout wire[31:0] base_ram_data,  //BaseRAM数据，低8位与CPLD串口控制器共享
    output wire[19:0] base_ram_addr, //BaseRAM地址
    output wire[3:0] base_ram_be_n,  //BaseRAM字节使能，低有效。如果不使用字节使能，请保持为0
    output wire base_ram_ce_n,       //BaseRAM片选，低有效
    output wire base_ram_oe_n,       //BaseRAM读使能，低有效
    output wire base_ram_we_n,       //BaseRAM写使能，低有效

    //ExtRAM信号
    inout wire[31:0] ext_ram_data,  //ExtRAM数据
    output wire[19:0] ext_ram_addr, //ExtRAM地址
    output wire[3:0] ext_ram_be_n,  //ExtRAM字节使能，低有效。如果不使用字节使能，请保持为0
    output wire ext_ram_ce_n,       //ExtRAM片选，低有效
    output wire ext_ram_oe_n,       //ExtRAM读使能，低有效
    output wire ext_ram_we_n,       //ExtRAM写使能，低有效

    output wire txd,  //直连串口发送端
    input  wire rxd
    );

    assign base_ram_ce_n = 1'b0;
    assign base_ram_we_n = 1'b1;

    assign ext_ram_ce_n  = 1'b0;// 片选信号，低电平有效 

// assign ext_ram_oe_n  = ~es_load_valid;
    assign ext_ram_oe_n  = ~ext_ram_we_n;
    always @(posedge clk) begin
        if (~ext_ram_oe_n) 
            data_sram_rdata <= ext_ram_data;
    end
    // assign data_sram_rdata  = ext_ram_data;
    assign ext_ram_data  = (~ext_ram_we_n)?data_sram_wdata:32'bz;

    
    //直连串口接收发送演示，从直连串口收到的数据再发送出去
    wire [7:0] ext_uart_rx;
    reg  [7:0] ext_uart_buffer, ext_uart_tx;
    wire ext_uart_ready, ext_uart_clear, ext_uart_busy;
    reg ext_uart_start, ext_uart_avai;
        
    assign number = ext_uart_buffer;

    async_receiver #(.ClkFrequency(50000000),.Baud(9600)) //接收模块，9600无检验位
        ext_uart_r(
            .clk(clk_50M),                       //外部时钟信号
            .RxD(rxd),                           //外部串行信号输入
            .RxD_data_ready(ext_uart_ready),  //数据接收到标志
            .RxD_clear(ext_uart_clear),       //清除接收标志
            .RxD_data(ext_uart_rx)             //接收到的一字节数据
        );

    assign ext_uart_clear = ext_uart_ready; //收到数据的同时，清除标志，因为数据已取到ext_uart_buffer中
    always @(posedge clk_50M) begin //接收到缓冲区ext_uart_buffer
        if(ext_uart_ready)begin
            ext_uart_buffer <= ext_uart_rx;
            ext_uart_avai <= 1;
        end else if(!ext_uart_busy && ext_uart_avai)begin 
            ext_uart_avai <= 0;
        end
    end
    always @(posedge clk_50M) begin //将缓冲区ext_uart_buffer发送出去
        if(!ext_uart_busy && ext_uart_avai)begin 
            ext_uart_tx <= ext_uart_buffer;
            ext_uart_start <= 1;
        end else begin 
            ext_uart_start <= 0;
        end
    end

    async_transmitter #(.ClkFrequency(50000000),.Baud(9600)) //发送模块，9600无检验位
        ext_uart_t(
            .clk(clk_50M),                  //外部时钟信号
            .TxD(txd),                      //串行信号输出
            .TxD_busy(ext_uart_busy),       //发送器忙状态指示
            .TxD_start(ext_uart_start),    //开始发送信号
            .TxD_data(ext_uart_tx)        //待发送的数据
        );
endmodule
