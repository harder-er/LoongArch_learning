`define UART_ADDR 32'h1fd003f8
`define FLAG_ADDR 32'h1fd003fc

// | 0xBFD003F8 | [7:0] | 串口数据，读、写地址分别表示串口接收、发送一个字节 |
// | 0xBFD003FC | [0]   | 只读，为1时表示串口空闲，可发送数据                |
// | 0xBFD003FC | [1]   | 只读，为1时表示串口收到数据                        |
module uart(
    input wire clk,
    input wire resetn,

    // read and write from cpu
    input   wire          conf_en,     
    input   wire          conf_re,conf_we,
    input   wire   [3 :0] conf_wen,      
    input   wire   [31:0] conf_addr,    
    input   wire   [31:0] conf_wdata,   
    output  wire   [31:0] conf_rdata,
    // read and write to device on board
    //直连串口信号
    output wire txd,  //直连串口发送端
    input  wire rxd,  //直连串口接收端
    // output reg [15:0] led,
    input wire [7:0] switch
);
wire read_flag;
wire write_uart;
wire read_uart;
assign read_flag = ((conf_addr == 32'h1fd003fc)&&(conf_en)&&(conf_re)) ? 1'b1:1'b0;
assign read_uart = ((conf_addr == 32'h1fd003f8) && (conf_en) && (conf_re)) ? 1'b1:1'b0;
assign write_uart = ((conf_addr == 32'h1fd003f8) && (conf_en) && (conf_we)) ? 1'b1:1'b0;

// uart
// values
wire [7:0] ext_uart_rdata;// read data
wire [7:0] ext_uart_wdata;// write data
// assign ext_uart_wdata = conf_wdata[7:0];

// buffers
reg[7:0] ext_uart_rbuffer;// read buffer
reg[7:0] ext_uart_flag; // uart flag for read and write at addr 0xbfd003fc
assign conf_rdata = (conf_addr == `FLAG_ADDR)?{24'h0,ext_uart_flag}:{24'h0,ext_uart_rdata};

// assign ext_uart_rdata = ext_uart_rbuffer;
assign ext
wire oe_to_ext_uart_rx;
assign oe_to_ext_uart_rx = read_uart;
wire [7:0] ext_uart_rx;
wire ext_uart_ready,ext_uart_clear;

// uart transmitter
wire we_to_ext_uart_tx;
assign we_to_ext_uart_tx = write_uart;
// assign we_to_ext_uart_tx = conf_en&&(conf_addr == `UART_ADDR);
wire ext_uart_busy; // transmitter busy flag
reg ext_uart_start; // transmitter start work signal
always @(posedge clk) begin
    if(!resetn)begin
        ext_uart_flag <= 8'h01;
    end else begin
        // read flag
        if(ext_uart_ready)begin
            ext_uart_flag[1] <= 1;
        end else if(oe_to_ext_uart_rx) begin
            ext_uart_flag[1] <= 0;
        end

        // write flag
        if(we_to_ext_uart_tx)begin
            ext_uart_flag[0] <= 0;
        end else if(!ext_uart_busy)begin
            ext_uart_flag[0] <= 1;
        end
    end
end

//  uart reciever

async_receiver #(.ClkFrequency(50000000),.Baud(9600)) 
    ext_uart_r(
        .clk(clk),
        .RxD(rxd),
        .RxD_data_ready(ext_uart_ready),
        .RxD_clear(ext_uart_clear),
        .RxD_data(ext_uart_rx)
    );
// store RxD_data to read buffer and clear RxD_data after store
assign ext_uart_clear = ext_uart_ready;
always @(posedge clk) begin
    if(ext_uart_ready)begin
        ext_uart_rbuffer <= ext_uart_rx;
    end
end


always @(posedge clk) begin
    if(!ext_uart_busy && we_to_ext_uart_tx)begin
        ext_uart_start <= 1;
    end else begin
        ext_uart_start <= 0;
    end
end
// wire [7:0] ext_uart_tx;
wire [7:0] ext_uart_tx;// write data
assign ext_uart_tx = !resetn?32'h0:write_uart?conf_wdata[7:0]:32'h0;

async_transmitter #(.ClkFrequency(50000000),.Baud(9600)) 
    ext_uart_t(
        .clk(clk),
        .TxD(txd),
        .TxD_busy(ext_uart_busy),
        .TxD_start(ext_uart_start),
        .TxD_data(ext_uart_tx) // transmit the data in buffer to txd
    );


//     wire [1:0] uart_flag;
//     wire [7:0] uart_data;
    
//     // read data has one cycle delay
//     reg [31:0] conf_rdata_reg;
//     assign conf_rdata = conf_rdata_reg;
//     always @ (posedge clk) begin
//         if (!resetn) begin
//             conf_rdata_reg <= 32'b0;
//         end
//         else if (conf_en) begin
//             case(conf_addr)
//                 `FLAG_ADDR : conf_rdata_reg <= {30'd0,uart_flag};
//                 `UART_ADDR : conf_rdata_reg <= {24'd0,uart_data};
//                 default:     conf_rdata_reg <= 32'b0;
//             endcase
//         end
//     end

//     // assign conf_rdata = (conf_addr == `FLAG_ADDR)?{30'd0,uart_flag}:
//     //                     conf_addr == `UART_ADDR? {24'd0,uart_flag}: 32'd0; 

//     //conf write, only support a word write
//     wire conf_we;
//     assign conf_we = conf_en & (|conf_wen);

// //---------------------------{uart}begin-------------------------//
// wire [7:0] ext_uart_rx;
// reg  [7:0] ext_uart_tx;
// reg  [7:0] ext_uart_buffer;
// wire ext_uart_ready, ext_uart_clear, ext_uart_busy;
// reg ext_uart_start, ext_uart_avai;

// wire [7:0] write_uart_data;
// wire write_uart_valid  = conf_we & (conf_addr==`UART_ADDR);
// // reg [7:0] write_uart_DATA;
// // always @(posedge clk) begin
// //     if (!resetn) 
// //         write_uart_DATA <= 8'h0;
// //     else 
// //         write_uart_DATA <= conf_wdata[7:0];
// // end
// // assign write_uart_data = write_uart_DATA;
// assign write_uart_data = conf_wdata[7:0];

// always @(posedge clk) begin //将缓冲区ext_uart_buffer发送出去
//     if (!resetn) begin
//         ext_uart_tx <= 8'b0;
//         ext_uart_start <= 1'b0;
//     end
//     else if(write_uart_valid)begin 
//         ext_uart_tx <= write_uart_data;
//         ext_uart_start <= 1;
//     end else begin 
//         ext_uart_start <= 0;
//     end
// end

// assign ext_uart_clear = ext_uart_ready; //收到数据的同时，清除标志，因为数据已取到ext_uart_buffer中
// assign uart_data = ext_uart_buffer;
// assign uart_flag = {ext_uart_avai,~ext_uart_busy};
// always @(posedge clk) begin //接收到缓冲区ext_uart_buffer
//     if (!resetn) begin
//         ext_uart_buffer <= 8'b0;
//         ext_uart_avai <= 1'b0;
//     end
//     else if(ext_uart_ready)begin
//         ext_uart_buffer <= ext_uart_rx;
//         ext_uart_avai <= 1'b1;
//     end 
//     else if(conf_addr == `UART_ADDR && (conf_en & ~conf_we) && ext_uart_avai)begin 
//         ext_uart_avai <= 1'b0;
//     end
// end

// async_receiver #(.ClkFrequency(64000000),.Baud(9600)) //接收模块 9600无检验位
//     ext_uart_r(
//         .clk(clk),                          //外部时钟信号
//         .RxD(rxd),                          //外部串行信号输入
//         .RxD_data_ready(ext_uart_ready),    //数据接收到标志
//         .RxD_clear(ext_uart_clear),         //清除接收标志
//         .RxD_data(ext_uart_rx)              //接收到的一字节数据
//     );

// async_transmitter #(.ClkFrequency(64000000),.Baud(9600)) //发送模块 9600无检验位
//     ext_uart_t(
//         .clk(clk),                          //外部时钟信号
//         .TxD(txd),                          //串行信号输出
//         .TxD_busy(ext_uart_busy),           //发送器忙状态指示
//         .TxD_start(ext_uart_start),         //开始发送信号
//         .TxD_data(ext_uart_tx)              //待发送的数据
//     );

// //直连串口接收发送演示，从直连串口收到的数据再发送出去
// wire [7:0] ext_uart_rx;
// reg  [7:0] ext_uart_buffer, ext_uart_tx;
// wire ext_uart_ready, ext_uart_clear, ext_uart_busy;
// reg ext_uart_start, ext_uart_avai;
    


// async_receiver #(.ClkFrequency(50000000),.Baud(9600)) //接收模块，9600无检验位
//     ext_uart_r(
//         .clk(clk_50M),                       //外部时钟信号
//         .RxD(rxd),                           //外部串行信号输入
//         .RxD_data_ready(ext_uart_ready),  //数据接收到标志
//         .RxD_clear(ext_uart_clear),       //清除接收标志
//         .RxD_data(ext_uart_rx)             //接收到的一字节数据
//     );

// assign ext_uart_clear = ext_uart_ready; //收到数据的同时，清除标志，因为数据已取到ext_uart_buffer中
// always @(posedge clk_50M) begin //接收到缓冲区ext_uart_buffer
//     if(ext_uart_ready)begin
//         ext_uart_buffer <= ext_uart_rx;
//         ext_uart_avai <= 1;
//     end else if(!ext_uart_busy && ext_uart_avai)begin 
//         ext_uart_avai <= 0;
//     end
// end
// always @(posedge clk_50M) begin //将缓冲区ext_uart_buffer发送出去
//     if(!ext_uart_busy && ext_uart_avai)begin 
//         ext_uart_tx <= ext_uart_buffer;
//         ext_uart_start <= 1;
//     end else begin 
//         ext_uart_start <= 0;
//     end
// end

// async_transmitter #(.ClkFrequency(50000000),.Baud(9600)) //发送模块，9600无检验位
//     ext_uart_t(
//         .clk(clk_50M),                  //外部时钟信号
//         .TxD(txd),                      //串行信号输出
//         .TxD_busy(ext_uart_busy),       //发送器忙状态指示
//         .TxD_start(ext_uart_start),    //开始发送信号
//         .TxD_data(ext_uart_tx)        //待发送的数据
//     );

endmodule