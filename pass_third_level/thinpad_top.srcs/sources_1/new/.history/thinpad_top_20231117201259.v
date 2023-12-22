`default_nettype none

thinpad_top u_thinpad_top(
    .clk_50M              ( clk_50M              ),
    .clk_11M0592          ( clk_11M0592          ),
    .clock_btn            ( clock_btn            ),
    .reset_btn            ( reset_btn            ),
    .touch_btn            ( touch_btn            ),
    .dip_sw               ( dip_sw               ),
    .leds                 ( leds                 ),
    .dpy0                 ( dpy0                 ),
    .dpy1                 ( dpy1                 ),
    .inout  base_ram_data ( inout  base_ram_data ),
    .base_ram_addr        ( base_ram_addr        ),
    .base_ram_be_n        ( base_ram_be_n        ),
    .base_ram_ce_n        ( base_ram_ce_n        ),
    .base_ram_oe_n        ( base_ram_oe_n        ),
    .base_ram_we_n        ( base_ram_we_n        ),
    .inout  ext_ram_data  ( inout  ext_ram_data  ),
    .ext_ram_addr         ( ext_ram_addr         ),
    .ext_ram_be_n         ( ext_ram_be_n         ),
    .ext_ram_ce_n         ( ext_ram_ce_n         ),
    .ext_ram_oe_n         ( ext_ram_oe_n         ),
    .ext_ram_we_n         ( ext_ram_we_n         ),
    .txd                  ( txd                  ),
    .rxd                  ( rxd                  ),
    .flash_a              ( flash_a              ),
    .inout  flash_d       ( inout  flash_d       ),
    .flash_rp_n           ( flash_rp_n           ),
    .flash_vpen           ( flash_vpen           ),
    .flash_ce_n           ( flash_ce_n           ),
    .flash_oe_n           ( flash_oe_n           ),
    .flash_we_n           ( flash_we_n           ),
    .flash_byte_n         ( flash_byte_n         ),
    .video_red            ( video_red            ),
    .video_green          ( video_green          ),
    .video_blue           ( video_blue           ),
    .video_hsync          ( video_hsync          ),
    .video_vsync          ( video_vsync          ),
    .video_clk            ( video_clk            ),
    .video_de             ( video_de             )
);


/* =========== Demo code begin =========== */

// PLL分频示例
wire locked, clk_10M, clk_20M;
pll_example clock_gen 
 (
  // Clock in ports
  .clk_in1(clk_50M),  // 外部时钟输入
  // Clock out ports
  .clk_out1(clk_10M), // 时钟输出1，频率在IP配置界面中设置
  .clk_out2(clk_20M), // 时钟输出2，频率在IP配置界面中设置
  // Status and control signals
  .reset(reset_btn), // PLL复位输入
  .locked(locked)    // PLL锁定指示输出，"1"表示时钟稳定，
                     // 后级电路复位信号应当由它生成（见下）
 );

reg reset_of_clk10M;
// 异步复位，同步释放，将locked信号转为后级电路的复位reset_of_clk10M
always@(posedge clk_10M or negedge locked) begin
    if(~locked) reset_of_clk10M <= 1'b1;
    else        reset_of_clk10M <= 1'b0;
end

// always@(posedge clk_10M or negedge locked) begin
//     if(~locked) begin 
//         reset_of_test <= 1'b1;
//     end
//     else begin
//         reset_of_test <= 1'b0;
//     end
// end

// always@(posedge clk_10M or posedge reset_of_clk10M) begin
//     if(reset_of_clk10M)begin
//         // Your Code
//     end
//     else begin
//         // Your Code
//     end
// end
//cpu inst sram

mycpu_top u_mycpu_top(
    .clk             ( clk_50M         ),
    .resetn          ( ~reset_btn      ),

    .inst_sram_en    ( inst_sram_en    ),
    .inst_sram_we    ( inst_sram_we    ),
    .inst_sram_addr  ( inst_sram_addr  ),
    .inst_sram_wdata ( inst_sram_wdata ),
    .inst_sram_rdata ( inst_sram_rdata ),
    .data_sram_en    ( data_sram_en    ),
    .data_sram_we    ( data_sram_we    ),
    .data_sram_addr  ( data_sram_addr  ),
    .data_sram_wdata ( data_sram_wdata ),
    .data_sram_rdata ( data_sram_rdata ) 
);

uart uart_1(
    .clk             ( clk_50M         ),
    .resetn          ( ~reset_btn      ),
    .inst_sram_en    ( inst_sram_en    ),
    .inst_sram_we    ( inst_sram_we    ),
    .inst_sram_addr  ( inst_sram_addr  ),
    .inst_sram_wdata ( inst_sram_wdata ),
    .inst_sram_rdata ( inst_sram_rdata ),
    .data_sram_en    ( data_sram_en    ),
    .data_sram_we    ( data_sram_we    ),
    .data_sram_addr  ( data_sram_addr  ),
    .data_sram_wdata ( data_sram_wdata ),
    .data_sram_rdata ( data_sram_rdata ),

    
)
// 不使用内存、串口时，禁用其使能信号
// assign base_ram_ce_n = 1'b1;
// assign base_ram_oe_n = 1'b1;
// assign base_ram_we_n = 1'b1;

// // assign ext_ram_ce_n = 1'b1;
// assign ext_ram_oe_n = 1'b1;
// assign ext_ram_we_n = 1'b1;

// 数码管连接关系示意图，dpy1同理
// p=dpy0[0] // ---a---
// c=dpy0[1] // |     |
// d=dpy0[2] // f     b
// e=dpy0[3] // |     |
// b=dpy0[4] // ---g---
// a=dpy0[5] // |     |
// f=dpy0[6] // e     c
// g=dpy0[7] // |     |
//           // ---d---  p

// 7段数码管译码器演示，将number用16进制显示在数码管上面
wire[7:0] number;
SEG7_LUT segL(.oSEG1(dpy0), .iDIG(number[3:0])); //dpy0是低位数码管
SEG7_LUT segH(.oSEG1(dpy1), .iDIG(number[7:4])); //dpy1是高位数码管

reg[15:0] led_bits;
assign leds = led_bits;

always@(posedge clock_btn or posedge reset_btn) begin
    if(reset_btn)begin //复位按下，设置LED为初始值
        led_bits <= 16'h1;
    end
    else begin //每次按下时钟按钮，LED循环左移
        led_bits <= {led_bits[14:0],led_bits[15]};
    end
end

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

//图像输出演示，分辨率800x600@75Hz，像素时钟为50MHz
// wire [11:0] hdata;
// assign video_red = hdata < 266 ? 3'b111 : 0; //红色竖条
// assign video_green = hdata < 532 && hdata >= 266 ? 3'b111 : 0; //绿色竖条
// assign video_blue = hdata >= 532 ? 2'b11 : 0; //蓝色竖条
// assign video_clk = clk_50M;
// vga #(12, 800, 856, 976, 1040, 600, 637, 643, 666, 1, 1) vga800x600at75 (
//     .clk(clk_50M), 
//     .hdata(hdata), //横坐标
//     .vdata(),      //纵坐标
//     .hsync(video_hsync),
//     .vsync(video_vsync),
//     .data_enable(video_de)
// );
/* =========== Demo code end =========== */

endmodule
