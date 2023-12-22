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
`define SerialStat 32'hBFD003FC
`define SerialDate 32'hBFD003F8

module uart(
    input wire        clk,
    input wire        rst,
    // inst sram interface
    input wire       inst_sram_en,         
    input wire[ 3:0] inst_sram_we,
    input wire[31:0] inst_sram_addr,
    input wire[31:0] inst_sram_wdata,
    output wire [31:0] inst_sram_rdata,
    // data sram interface
    input wire      data_sram_en,
    input wire[ 3:0] data_sram_we,
    input wire[31:0] data_sram_addr,
    input wire[31:0] data_sram_wdata,
    output  wire[31:0] data_sram_rdata, 

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
    // wire clk,resetn;
    assign base_ram_be_n = ~inst_sram_we;
    assign ext_ram_be_n = ~data_sram_we;
    reg [31:0] inst_sram_Rdata,data_sram_Rdata;
    
    assign inst_sram_rdata = inst_sram_Rdata;
    assign data_sram_rdata = data_sram_Rdata;
    assign base_ram_ce_n = 1'b0;
    assign base_ram_we_n = ~inst_sram_en;
    assign base_ram_oe_n = inst_sram_en;
    assign ext_ram_ce_n  = 1'b0;// 片选信号，低电平有效 

    assign base_ram_addr = is_base_ram==1'b1?:inst_sram_addr[21:2];
    assign ext_ram_addr = data_sram_addr[21:2];
    always @(base_ram_oe_n or base_ram_data) begin
        if (~base_ram_oe_n)
            inst_sram_Rdata <= base_ram_data;
    end
    assign base_ram_data = (~base_ram_we_n)?inst_sram_wdata:32'bz;
// assign ext_ram_oe_n  = ~es_load_valid;
    assign ext_ram_we_n = data_sram_en;
    assign ext_ram_oe_n  = ~ext_ram_we_n;
    // always @(posedge clk) begin
    //     if (~ext_ram_oe_n) 
    //         data_sram_rdata <= ext_ram_data;
    // end
    // assign data_sram_rdata  = ext_ram_data;
    assign ext_ram_data  = (~ext_ram_we_n)?data_sram_wdata:32'bz;

    // //直连串口接收发送演示，从直连串口收到的数据再发送出去
    // wire [7:0] ext_uart_rx;
    // reg  [7:0] ext_uart_buffer, ext_uart_tx;
    // wire ext_uart_ready, ext_uart_clear, ext_uart_busy;
    // reg ext_uart_start, ext_uart_avai;
        
    // assign number = ext_uart_buffer;

    // async_receiver #(.ClkFrequency(50000000),.Baud(9600)) //接收模块，9600无检验位
    //     ext_uart_r(
    //         .clk(clk),                       //外部时钟信号
    //         .RxD(rxd),                           //外部串行信号输入
    //         .RxD_data_ready(ext_uart_ready),  //数据接收到标志
    //         .RxD_clear(ext_uart_clear),       //清除接收标志
    //         .RxD_data(ext_uart_rx)             //接收到的一字节数据
    //     );

    // assign ext_uart_clear = ext_uart_ready; //收到数据的同时，清除标志，因为数据已取到ext_uart_buffer中
    // always @(posedge clk) begin //接收到缓冲区ext_uart_buffer
    //     if(ext_uart_ready)begin
    //         ext_uart_buffer <= ext_uart_rx;
    //         ext_uart_avai <= 1;
    //     end else if(!ext_uart_busy && ext_uart_avai)begin 
    //         ext_uart_avai <= 0;
    //     end
    // end
    // always @(posedge clk) begin //将缓冲区ext_uart_buffer发送出去
    //     if(!ext_uart_busy && ext_uart_avai)begin 
    //         ext_uart_tx <= ext_uart_buffer;
    //         ext_uart_start <= 1;
    //     end else begin 
    //         ext_uart_start <= 0;
    //     end
    // end

    // async_transmitter #(.ClkFrequency(50000000),.Baud(9600)) //发送模块，9600无检验位
    //     ext_uart_t(
    //         .clk(clk),                  //外部时钟信号
    //         .TxD(txd),                      //串行信号输出
    //         .TxD_busy(ext_uart_busy),       //发送器忙状态指示
    //         .TxD_start(ext_uart_start),    //开始发送信号
    //         .TxD_data(ext_uart_tx)        //待发送的数据
    //     );

    (*mark_debug = "true"*)wire [7:0]  ext_uart_rx;             ///< 接收到的数据线路
    (*mark_debug = "true"*)reg  [7:0]  ext_uart_tx;                    ///< 发送数据的线路
    (*mark_debug = "true"*)wire        ext_uart_ready,          ///< 接收器收到数据完成之后，置为1
    ext_uart_busy;           ///< 发送器状态是否忙碌，1为忙碌，0为不忙碌
    (*mark_debug = "true"*)reg         ext_uart_start,          ///< 传递给发送器，为1时，代表可以发送，为0时，代表不发送
    ext_uart_clear,          ///< 置1，在下次时钟有效的时候，会清楚接收器的标志位
        ext_uart_avai;           ///< 代表缓冲区是否可用，是否存有数据

    reg  [7:0]  ext_uart_buffer_recive,     ///< 接受数据缓冲区
        ext_uart_buffer_send;              ///< 发送数据缓冲区

    reg         ext_uart_buffer_send_ok;    ///< 发送数据缓冲区已经可以发送，1为可以，0为不可以

// assign number = ext_uart_buffer;

    async_receiver #(.ClkFrequency(50000000),.Baud(9600)) //接收模块，9600无检验位
               ext_uart_r(
                   .clk(clk),                       //外部时钟信号
                   .RxD(rxd),                           //外部串行信号输入
                   .RxD_data_ready(ext_uart_ready),     //数据接收到标志
                   .RxD_clear(ext_uart_clear),          //清除接收标志
                   .RxD_data(ext_uart_rx)               //接收到的一字节数据
               );

    // assign ext_uart_clear = ext_uart_ready;                 //收到数据的同时，清除标志，因为数据已取到ext_uart_buffer中

    // always @(posedge clk) begin                         //接收到缓冲区ext_uart_buffer
    //     if(ext_uart_ready) begin
    //         ext_uart_buffer_recive <= ext_uart_rx;
    //     end
    // end

    // always @(posedge clk) begin                         //将缓冲区ext_uart_buffer发送出去
    //     if(!ext_uart_busy && ext_uart_buffer_send_ok) begin
    //         ext_uart_tx <= ext_uart_buffer_send;
    //         ext_uart_start <= 1;
    //     end
    //     else begin
    //         ext_uart_start <= 0;
    //     end
    // end

    async_transmitter #(.ClkFrequency(50000000),.Baud(9600)) //发送模块，9600无检验位
                    ext_uart_t(
                        .clk(clk),                  //外部时钟信号
                        .TxD(txd),                      //串行信号输出
                        .TxD_busy(ext_uart_busy),       //发送器忙状态指示
                        .TxD_start(ext_uart_start),     //开始发送信号
                        .TxD_data(ext_uart_tx)          //待发送的数据
                    );


    /*****************************************************************************
    CPU 连接协同模块
    *****************************************************************************/

    /// 处理读取或者写入的数据范围
    wire is_SerialStat = (data_sram_addr ==  `SerialStat);
    wire is_SerialDate = (data_sram_addr == `SerialDate);
    // wire is_base_ram = is_SerialStat != 1'b1 && is_SerialDate != 1'b1 && (data_sram_addr >= 32'h80000000) &&   (data_sram_addr < 32'h80400000);
    wire is_base_ram = is_SerialStat != 1'b1 && is_SerialDate != 1'b1 && (data_sram_addr[22] == 1'b0);
    wire is_ext_ram = is_SerialStat != 1'b1 && is_SerialDate != 1'b1 &&  (data_sram_addr[22] == 1'b1);

    (*mark_debug = "true"*)reg[31:0] serial_o;
    // wire [31:0] base_ram_o;
    // wire  [31:0] ext_ram_o;

    /// 处理串口
    always @(posedge clk) begin
        begin
            if(is_SerialStat) begin                                     /// 获取串口状态
                serial_o <= {{30{1'b0}}, {ext_uart_ready, !ext_uart_busy}};
                ext_uart_start <= 1'b0;
                ext_uart_tx <= 8'h00;
            end
            else if(is_SerialDate) begin                   /// 获取（或发送）串口数据
                if(~data_sram_en) begin                                     /// 读数据，即接收串口数据
                    serial_o <= {24'h000000, ext_uart_rx};
                    ext_uart_start <= 1'b0;
                    ext_uart_tx <= 8'h00;
                end
                else begin                                              /// 写数据，即发送串口数据
                    ext_uart_tx <= data_sram_wdata[7:0];
                    ext_uart_start <= 1'b1;
                    serial_o <= 32'h0000_0000;
                end
            end
            else begin
                ext_uart_start <= 1'b0;
                serial_o <= 32'h0000_0000;
                ext_uart_tx <= 8'h00;
            end
        end
    end

    /// 处理串口接收的clear
    reg     ext_uart_clear_next;
    reg[3:0] ext_uart_clear_para;

    always @(negedge clk) begin
        begin
            if(ext_uart_ready && data_sram_addr == `SerialDate && ~ext_ram_we_n && ext_uart_clear_next == 1'b0) begin
                ext_uart_clear_next <= 1'b1;
            end
            else if (ext_uart_clear == 1'b1) begin
                ext_uart_clear_next <= 1'b0;
            end
            else begin
                ext_uart_clear_next <= ext_uart_clear_next;
            end
        end
    end

    always @(posedge clk) begin
        begin
            if(ext_uart_clear_next) begin
                ext_uart_clear <= 1'b1;
            end
            else begin
                ext_uart_clear <= 1'b0;
            end
        end
    end

    /// BaseRam 管理指令或者数据的存取
    // assign base_ram_data = is_base_ram ? ((ram2_we_i) ? 32'hzzzzzzzz : ram2_data_i) : 32'hzzzzzzzz;
    // assign base_ram_o = base_ram_data;      /// 在读取模式下，读取到的BaseRam数据

    /// 处理BaseRam
    /// 在需要从BaseRam中获取或者写入数据的时候，往往认为CPU会暂停流水线（1个时钟周期）
    // always @(*) begin
    //     if(rst) begin
    //         base_ram_addr <= 20'h0000_0;
    //         base_ram_be_n <= 4'b1111;
    //         base_ram_ce_n <= 1'b1;
    //         base_ram_oe_n <= 1'b1;
    //         base_ram_we_n <= 1'b1;
    //         rom_data_o <= 32'h0000_0000;
    //     end
    //     else begin
    //         if(is_base_ram) begin           /// 涉及到BaseRam的相关数据操作，默认暂停流水线
    //             base_ram_addr <= ram2_addr_i[21:2];
    //             base_ram_be_n <= ram2_sel_i;
    //             base_ram_ce_n <= 1'b0;
    //             base_ram_oe_n <= !ram2_we_i;
    //             base_ram_we_n <= ram2_we_i;
    //         end
    //         else begin                      /// 不涉及到BaseRam的相关数据操作，继续取指令
    //             base_ram_addr <= rom_addr_i[21:2];
    //             base_ram_be_n <= 4'b0000;
    //             base_ram_ce_n <= 1'b0;
    //             base_ram_oe_n <= 1'b0;
    //             base_ram_we_n <= 1'b1;
    //         end
    //         rom_data_o <= base_ram_o;
    //     end
    // end


    /// 处理ExtRam
    // assign ext_ram_data = (ram2_we_i) ? 32'hzzzzzzzz : ram2_data_i;
    // assign ext_ram_o = ext_ram_data;

    // always @(*) begin
    //     if(rst) begin
    //         ext_ram_addr <= 20'h00000;
    //         ext_ram_be_n <= 4'b1111;
    //         ext_ram_ce_n <= 1'b1;
    //         ext_ram_oe_n <= 1'b1;
    //         ext_ram_we_n <= 1'b1;
    //     end
    //     else begin
    //         if(is_ext_ram) begin           /// 涉及到extRam的相关数据操作
    //             ext_ram_addr <= ram2_addr_i[21:2];
    //             ext_ram_be_n <= ram2_sel_i;
    //             ext_ram_ce_n <= 1'b0;
    //             ext_ram_oe_n <= !ram2_we_i;
    //             ext_ram_we_n <= ram2_we_i;
    //         end
    //         else begin                      ///
    //             ext_ram_addr <= 20'h00000;
    //             ext_ram_be_n <= 4'b1111;
    //             ext_ram_ce_n <= 1'b1;
    //             ext_ram_oe_n <= 1'b1;
    //             ext_ram_we_n <= 1'b1;
    //         end
    //     end
    // end


    /// 模块，确认输出的数据
    always @(posedge clk) begin
        begin
            if(is_SerialStat || is_SerialDate ) begin
                data_sram_Rdata = serial_o;
            end
            else if (is_ext_ram && ~ext_ram_oe_n) begin
                // ram2_data_o <= ext_ram_o;
                data_sram_Rdata =  ext_ram_data;
            end
            else begin
                data_sram_Rdata <= 32'h0000_0000;
            end
        end
    end
endmodule
