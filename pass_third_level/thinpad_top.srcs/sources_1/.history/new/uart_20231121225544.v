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
    

    reg [31:0] data_sram_Rdata;
    // assign inst_sram_rdata = inst_sram_Rdata;
    assign data_sram_rdata = data_sram_Rdata;

    
    assign ext_ram_ce_n  = 1'b0;// 片选信号，低电平有效 
    
    // assign ext_ram_addr = data_sram_addr[21:2];
    assign inst_sram_rdata = base_ram_data;
    assign base_ram_data = (~base_ram_we_n)?inst_sram_wdata:32'bz;

   
    /*****************************************************************************
    CPU 连接协同模块
    *****************************************************************************/

    /// 处理读取或者写入的数据范围
    (*mark_debug="true"*)wire is_SerialStat = (data_sram_addr ==  `SerialStat);
    (*mark_debug="true"*)wire is_SerialDate = (data_sram_addr == `SerialDate);
    reg is_base_ram;
    always @(posedge clk) begin
        if (!rst) 
            is_base_ram = 1'b0;
        else if (!is_SerialDate&&!is_SerialStat&&data_sram_addr[31:22] == 10'b1000_0000_00)
            is_base_ram = 1'b1;
            base_ram_ADDR = data_sram_addr[21:2];
        else 
            is_base_ram = 1'b0;  
            base_ram_ADDR = inst_sram_addr[21:2];
    end
    // assign base_ram_ce_n = ~(is_base_ram || ~inst_sram_en);
    assign base_ram_ce_n = 1'b0;
    // wire is_base_ram = rst?is_SerialStat != 1'b1 && is_SerialDate != 1'b1 && (data_sram_addr[22] == 1'b0&&data_sram_addr[31] == 1'b1):1'b0;
    (*mark_debug="true"*)wire is_ext_ram = rst?(is_SerialStat != 1'b1 )&& (is_SerialDate != 1'b1) &&  (data_sram_addr[31:22] == 10'b1000_0000_01):1'b0;
    reg base_ram_BE_N; 
    assign base_ram_be_n = base_ram_BE_N;
    always @(posedge clk) begin
        if (!rst) 
            base_ram_BE_N = 4'b0;
        else if (is_base_ram||inst_sram_en) 
            base_ram_BE_N = ~inst_sram_we;
        else if (is_ext_ram)
            base_ram_BE_N = ~data_sram_we;
        else 
            base_ram_BE_N = base_ram_BE_N;
    end
    assign ext_ram_be_n = ~data_sram_we;
    //这块仲裁有问题
    assign base_ram_addr = is_base_ram&&inst_sram_en?data_sram_addr[21:2]:inst_sram_addr[21:2];
    assign base_ram_oe_n = (~inst_sram_en||is_base_ram)?1'b0:1'b1;
    assign base_ram_we_n = ~base_ram_oe_n;

    
    assign ext_ram_addr = data_sram_addr[21:2];
    assign ext_ram_we_n = is_ext_ram?data_sram_en:1'b1;
    assign ext_ram_oe_n = ~ext_ram_we_n;
    assign ext_ram_data = (~ext_ram_we_n)?data_sram_wdata:32'bz;

    // reg[31:0] serial_o;

    
    

    wire conf_we;
    assign conf_we = ~data_sram_en & (|data_sram_we);

    (*mark_debug="true"*)wire [7:0] ext_uart_rx;
    (*mark_debug="true"*)reg  [7:0] ext_uart_tx;
    (*mark_debug="true"*)reg  [7:0] ext_uart_buffer;
    // wire write_uart_valid  = conf_we & (conf_addr==`UART_ADDR);
    wire ext_uart_ready, ext_uart_clear, ext_uart_busy;
    reg ext_uart_start, ext_uart_avai;

    wire [7:0] write_uart_data;
    wire write_uart_valid  = conf_we & (is_SerialDate);
    assign write_uart_data = data_sram_wdata[7:0];

    always @(posedge clk) begin //将缓冲区ext_uart_buffer发送出去
        if (!rst) begin
            ext_uart_tx <= 8'b0;
            ext_uart_start <= 1'b0;
        end
        else if(write_uart_valid)begin 
            ext_uart_tx <= write_uart_data;
            ext_uart_start <= 1;
        end else begin 
            ext_uart_start <= 0;
        end
    end
    (*mark_debug="true"*)wire [1:0] uart_flag;
    (*mark_debug="true"*)wire [7:0] uart_data;
    assign ext_uart_clear = ext_uart_ready; //收到数据的同时，清除标志，因为数据已取到ext_uart_buffer中
    assign uart_data = ext_uart_buffer;
    assign uart_flag = {ext_uart_avai,~ext_uart_busy};
    always @(posedge clk) begin //接收到缓冲区ext_uart_buffer
        if (!rst) begin
            ext_uart_buffer <= 8'b0;
            ext_uart_avai <= 1'b0;
        end
        else if(ext_uart_ready)begin
            ext_uart_buffer <= ext_uart_rx;
            ext_uart_avai <= 1'b1;
        end 
        else if(is_SerialDate && (data_sram_en & ~conf_we) && ext_uart_avai)begin 
            ext_uart_avai <= 1'b0;
        end
    end

    async_receiver #(.ClkFrequency(64000000),.Baud(9600)) //接收模块 9600无检验位
        ext_uart_r(
            .clk(clk),                          //外部时钟信号
            .RxD(rxd),                          //外部串行信号输入
            .RxD_data_ready(ext_uart_ready),    //数据接收到标志
            .RxD_clear(ext_uart_clear),         //清除接收标志
            .RxD_data(ext_uart_rx)              //接收到的一字节数据
        );

    async_transmitter #(.ClkFrequency(64000000),.Baud(9600)) //发送模块 9600无检验位
        ext_uart_t(
            .clk(clk),                          //外部时钟信号
            .TxD(txd),                          //串行信号输出
            .TxD_busy(ext_uart_busy),           //发送器忙状态指示
            .TxD_start(ext_uart_start),         //开始发送信号
            .TxD_data(ext_uart_tx)              //待发送的数据
        );
    always @(posedge clk) begin
        begin
            if(is_SerialStat ) begin
                data_sram_Rdata = {30'd0,uart_flag};
            end
            else if (is_SerialDate)
                data_sram_Rdata = {24'd0,uart_data};
            else if (is_ext_ram && ~ext_ram_oe_n) begin
                // ram2_data_o <= ext_ram_o;
                data_sram_Rdata =  ext_ram_data;
            end
            else if (is_base_ram && ~base_ram_oe_n) begin
                data_sram_Rdata = base_ram_data;
            end
            else begin
                data_sram_Rdata <= 32'h0000_0000;
            end
        end
    end
    // 处理串口
    // always @(posedge clk) begin
    //     begin
    //         if(is_SerialStat) begin                                     /// 获取串口状态
    //             serial_o <= {{30{1'b0}}, {ext_uart_ready, !ext_uart_busy}};
    //             // ext_uart_start <= 1'b0;
    //             // ext_uart_tx <= 8'h00;
    //         end
    //         else if(is_SerialDate) begin                   /// 获取（或发送）串口数据
    //             if(~data_sram_en) begin                                     /// 读数据，即接收串口数据
    //                 serial_o <= {24'h000000, ext_uart_rx};
    //                 // ext_uart_start <= 1'b0;
    //                 // ext_uart_tx <= 8'h00;
    //             end
    //             else begin                                              /// 写数据，即发送串口数据
    //                 ext_uart_buffer <= data_sram_wdata[7:0];
    //                 // ext_uart_start <= 1'b1;
    //                 serial_o <= 32'h0000_0000;
    //             end
    //         end
    //         else begin
    //             // ext_uart_start <= 1'b0;
    //             serial_o <= 32'h0000_0000;
    //             ext_uart_tx <= 8'h00;
    //         end
    //     end
    // end
endmodule
