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

    wire [1:0] uart_flag;
    wire [7:0] uart_data;
    
    // read data has one cycle delay
    // reg [31:0] conf_rdata_reg;
    // assign conf_rdata = conf_rdata_reg;
    // always @ (posedge clk) begin
    //     if (!resetn) begin
    //         conf_rdata_reg <= 32'b0;
    //     end
    //     else if (conf_en) begin
    //         case(conf_addr)
    //             `FLAG_ADDR : conf_rdata_reg <= {30'd0,uart_flag};
    //             `UART_ADDR : conf_rdata_reg <= {24'd0,uart_data};
    //             default: conf_rdata_reg <= 32'b0;
    //         endcase
    //     end
    // end

    assign conf_rdata = (conf_addr == `FLAG_ADDR)?{30'd0,uart_flag}:
                        conf_addr == `UART_ADDR? {24'd0,uart_flag}: 32'd0; 

    //conf write, only support a word write
    wire conf_we;
    assign conf_we = conf_en & (|conf_wen);

//---------------------------{uart}begin-------------------------//
wire [7:0] ext_uart_rx;
reg  [7:0] ext_uart_tx;
reg  [7:0] ext_uart_buffer;
wire ext_uart_ready, ext_uart_clear, ext_uart_busy;
reg ext_uart_start, ext_uart_avai;

wire [7:0] write_uart_data;
wire write_uart_valid  = conf_we & (conf_addr==`UART_ADDR);
reg [7:0] write_uart_VALID;
always @(posedge clk) begin
    if (!resetn) 
        write_uart_VALID <= 8'h0;
    else 
        write_uart_VALID <= conf_wdata[7:0];
end
assign write_uart_data <
// assign write_uart_data = conf_wdata[7:0];

always @(posedge clk) begin //将缓冲区ext_uart_buffer发送出去
    if (!resetn) begin
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

assign ext_uart_clear = ext_uart_ready; //收到数据的同时，清除标志，因为数据已取到ext_uart_buffer中
assign uart_data = ext_uart_buffer;
assign uart_flag = {ext_uart_avai,~ext_uart_busy};
always @(posedge clk) begin //接收到缓冲区ext_uart_buffer
    if (!resetn) begin
        ext_uart_buffer <= 8'b0;
        ext_uart_avai <= 1'b0;
    end
    else if(ext_uart_ready)begin
        ext_uart_buffer <= ext_uart_rx;
        ext_uart_avai <= 1'b1;
    end 
    else if(conf_addr == `UART_ADDR && (conf_en & ~conf_we) && ext_uart_avai)begin 
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

endmodule